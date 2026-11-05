#include "conversation.h"

#include <atomic>
#include <cstring>

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>

#include "audio.h"
#include "face.h"
#include "opus_codec.h"
#include "pins.h"
#include "server_link.h"
#include "status.h"
#include "tools.h"
#include "wake_word.h"

#define TAG "convo"

namespace {

enum class Phase { kIdle, kConnecting, kListening, kSpeaking };
enum Event : uint8_t { kWakeWord, kWakeButton, kSpeakStart, kSpeakStop, kClosed };

struct Packet {
    uint16_t len;
    uint8_t data[];
};

ServerLink* g_link = nullptr;
std::atomic<Phase> g_phase{Phase::kIdle};
QueueHandle_t g_events = nullptr;   // Event
QueueHandle_t g_packets = nullptr;  // Packet* (PSRAM), server audio
std::atomic<bool> g_speaker_busy{false};
std::atomic<uint32_t> g_sent{0}, g_received{0}, g_dropped{0};
bool g_wake_ready = false;
int g_wake_chunk = 0;

const char* PhaseName(Phase p) {
    switch (p) {
        case Phase::kIdle: return "idle";
        case Phase::kConnecting: return "connecting";
        case Phase::kListening: return "listening";
        case Phase::kSpeaking: return "speaking";
    }
    return "?";
}

void SetPhase(Phase p) {
    g_phase.store(p);
    status::Set(p == Phase::kIdle        ? RobotState::kIdle
                : p == Phase::kConnecting ? RobotState::kThinking
                : p == Phase::kListening  ? RobotState::kListening
                                          : RobotState::kSpeaking);
    ESP_LOGI(TAG, "-> %s", PhaseName(p));
}

void Post(Event e) { xQueueSend(g_events, &e, 0); }

void ClearPackets() {
    Packet* p = nullptr;
    while (xQueueReceive(g_packets, &p, 0) == pdTRUE) heap_caps_free(p);
}

void StartListening() {
    audio::FlushMic();
    g_link->StartListening();
    SetPhase(Phase::kListening);
}

// ---- control ----------------------------------------------------------------------------

void ControlTask(void*) {
    Event e;
    while (true) {
        if (xQueueReceive(g_events, &e, portMAX_DELAY) != pdTRUE) continue;
        const Phase phase = g_phase.load();
        switch (e) {
            case kWakeWord:
            case kWakeButton:
                if (phase == Phase::kIdle) {
                    SetPhase(Phase::kConnecting);
                    if (!g_link->OpenSession() || !opus::OpenDecoder(g_link->SpeakerSampleRate())) {
                        ESP_LOGE(TAG, "could not open a session");
                        face::SetCaption("system", "No connection to the server");
                        audio::Beep(440, 200);
                        SetPhase(Phase::kIdle);
                        break;
                    }
                    if (e == kWakeWord) g_link->SendWakeWord("Jarvis");
                    StartListening();
                } else if (phase == Phase::kSpeaking) {
                    // Interrupt the answer and listen again.
                    g_link->AbortSpeaking();
                    ClearPackets();
                    StartListening();
                }
                break;
            case kSpeakStart:
                if (phase == Phase::kListening || phase == Phase::kConnecting) SetPhase(Phase::kSpeaking);
                break;
            case kSpeakStop:
                if (phase == Phase::kSpeaking) {
                    // Let the queued audio finish before listening again (half duplex).
                    for (int i = 0; i < 100 && (uxQueueMessagesWaiting(g_packets) || g_speaker_busy.load()); ++i) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    audio::SetStreaming(false);
                    if (g_link->SessionOpen()) StartListening();
                }
                break;
            case kClosed:
                ClearPackets();
                audio::SetStreaming(false);
                SetPhase(Phase::kIdle);
                break;
        }
    }
}

// ---- microphone ---------------------------------------------------------------------------

void MicTask(void*) {
    int16_t* pcm = (int16_t*)heap_caps_malloc(opus::kMicFrameSamples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    uint8_t packet[400];
    bool button_was_down = false;
    int16_t frame[320];
    int16_t* wake = (int16_t*)heap_caps_malloc((g_wake_chunk + 320) * sizeof(int16_t) + 4, MALLOC_CAP_INTERNAL);
    int wake_fill = 0;
    while (true) {
        // BOOT button: start (or interrupt) a conversation.
        const bool down = gpio_get_level(PIN_BOOT_BUTTON) == 0;
        if (down && !button_was_down) Post(kWakeButton);
        button_was_down = down;

        if (g_phase.load() == Phase::kIdle && g_wake_ready) {
            // Idle: feed the microphone to the wake word model in its chunk size.
            if (!audio::ReadMic(frame, 320)) continue;
            memcpy(wake + wake_fill, frame, 320 * sizeof(int16_t));
            wake_fill += 320;
            while (wake_fill >= g_wake_chunk) {
                if (wake_word::Detect(wake)) {
                    ESP_LOGI(TAG, "wake word");
                    Post(kWakeWord);
                    wake_fill = 0;
                    break;
                }
                wake_fill -= g_wake_chunk;
                memmove(wake, wake + g_wake_chunk, wake_fill * sizeof(int16_t));
            }
            continue;
        }
        if (g_phase.load() != Phase::kListening) {
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        if (!audio::ReadMic(pcm, opus::kMicFrameSamples)) continue;
        if (g_phase.load() != Phase::kListening) continue;  // the server started speaking meanwhile
        const size_t n = opus::Encode(pcm, packet, sizeof(packet));
        if (n > 0 && g_link->SendAudio(packet, n)) g_sent.fetch_add(1);
    }
}

// ---- speaker ------------------------------------------------------------------------------

void SpeakerTask(void*) {
    const size_t max_samples = 48000 * opus::kFrameMs / 1000;  // enough for any server rate
    int16_t* pcm = (int16_t*)heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    Packet* p = nullptr;
    while (true) {
        if (xQueueReceive(g_packets, &p, portMAX_DELAY) != pdTRUE) continue;
        g_speaker_busy.store(true);
        audio::SetStreaming(true);
        const size_t n = opus::Decode(p->data, p->len, pcm, max_samples);
        heap_caps_free(p);
        if (n > 0) audio::PlayPcm(pcm, n, opus::DecoderRate());
        g_speaker_busy.store(uxQueueMessagesWaiting(g_packets) > 0);
    }
}

}  // namespace

namespace conversation {

void Start(ServerLink* link) {
    g_link = link;
    g_events = xQueueCreate(16, sizeof(Event));
    g_packets = xQueueCreate(60, sizeof(Packet*));  // 3.6 s of speech
    opus::OpenEncoder();
    g_wake_ready = wake_word::Start();
    g_wake_chunk = g_wake_ready ? wake_word::ChunkSamples() : 0;
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << PIN_BOOT_BUTTON;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    ServerLink::Events ev;
    ev.on_audio = [](const uint8_t* data, size_t len) {
        g_received.fetch_add(1);
        if (g_phase.load() != Phase::kSpeaking && g_phase.load() != Phase::kListening) return;
        auto* p = (Packet*)heap_caps_malloc(sizeof(Packet) + len, MALLOC_CAP_SPIRAM);
        if (!p) return;
        p->len = len;
        memcpy(p->data, data, len);
        if (xQueueSend(g_packets, &p, 0) != pdTRUE) {
            heap_caps_free(p);
            g_dropped.fetch_add(1);
        }
    };
    ev.on_user_text = [](const std::string& t) { face::SetCaption("user", t); ESP_LOGI(TAG, ">> %s", t.c_str()); };
    ev.on_robot_text = [](const std::string& t) { face::SetCaption("assistant", t); ESP_LOGI(TAG, "<< %s", t.c_str()); };
    ev.on_emotion = [](const std::string& e) { face::SetEmotion(e.c_str()); };
    ev.on_speak_start = [] { Post(kSpeakStart); };
    ev.on_speak_stop = [] { Post(kSpeakStop); };
    ev.on_session_closed = [] { Post(kClosed); };
    ev.on_mcp = [](const std::string& payload) { tools::HandleMcp(payload); };
    tools::Start([link](const std::string& payload) { link->SendMcp(payload); });

    // Stacks in PSRAM: none of these tasks writes flash, and internal RAM is kept for DMA
    // (camera, I2S, Wi-Fi). Audio on core 1, control on core 0.
    xTaskCreatePinnedToCoreWithCaps(ControlTask, "convo", 6144, nullptr, 5, nullptr, 0, MALLOC_CAP_SPIRAM);
    // libopus keeps large arrays on the stack: 6 and 24 KB overflowed for the encoder.
    xTaskCreatePinnedToCoreWithCaps(MicTask, "mic", 32768, nullptr, 7, nullptr, 1, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCoreWithCaps(SpeakerTask, "speaker", 16384, nullptr, 7, nullptr, 1, MALLOC_CAP_SPIRAM);

    // The link's start (HTTPS configuration request, TLS to the MQTT broker) needs a large
    // stack: it runs in its own task, never in app_main (whose small stack overflowed).
    static ServerLink::Events events;
    events = ev;
    xTaskCreatePinnedToCore(
        [](void*) {
            if (!g_link->Start(events)) {
                ESP_LOGE(TAG, "server link did not start");
                face::SetCaption("system", "Server not reachable");
            }
            vTaskDelete(nullptr);
        },
        "link_start", 8192, nullptr, 4, nullptr, 0);
}

void Wake(bool by_wake_word) { Post(by_wake_word ? kWakeWord : kWakeButton); }

std::string Describe() {
    char s[160];
    snprintf(s, sizeof(s), "conversation %s, audio up %u / down %u (dropped %u), queue %u",
             PhaseName(g_phase.load()), (unsigned)g_sent.load(), (unsigned)g_received.load(),
             (unsigned)g_dropped.load(), g_packets ? (unsigned)uxQueueMessagesWaiting(g_packets) : 0u);
    return s + std::string("; ") + (g_link ? g_link->Describe() : "");
}

}  // namespace conversation
