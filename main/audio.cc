#include "audio.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pins.h"
#include "settings.h"

#define TAG "audio"

namespace {
constexpr int kFrame = 320;          // 20 ms at 16 kHz
constexpr float kMarginDb = 10.0f;   // speech must be this far above the noise floor
constexpr int kStartFrames = 12;     // 240 ms of speech starts an utterance
constexpr int kEndFrames = 30;       // 600 ms of silence ends it
constexpr int kPrerollFrames = 15;   // 300 ms kept from before the start

i2s_chan_handle_t g_rx = nullptr, g_tx = nullptr;
std::mutex g_tx_mutex;
std::atomic<bool> g_playing{false}, g_stop_play{false};
std::atomic<int> g_volume{80};
int g_tx_rate = 0;
float g_floor_db = -60.0f;

// PSRAM allocator so long recordings do not use internal RAM.
template <class T>
struct PsramAlloc {
    using value_type = T;
    PsramAlloc() = default;
    template <class U> PsramAlloc(const PsramAlloc<U>&) {}
    T* allocate(size_t n) { return (T*)heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM); }
    void deallocate(T* p, size_t) { heap_caps_free(p); }
};

float FrameDb(const int16_t* s, int n) {
    double sum = 0;
    for (int i = 0; i < n; ++i) sum += (double)s[i] * s[i];
    const double rms = std::sqrt(sum / n) + 1e-3;
    return 20.0f * std::log10(rms / 32768.0);
}

// 32-bit I2S words from the INMP441 (24-bit, left justified) -> int16 with a high-pass
// (removes the DC offset / low rumble).
int ReadFrame(int16_t* out) {
    static int32_t raw[kFrame];
    static float hp_x = 0, hp_y = 0;
    size_t got = 0;
    if (i2s_channel_read(g_rx, raw, sizeof(raw), &got, 200) != ESP_OK) return 0;
    const int n = got / sizeof(int32_t);
    for (int i = 0; i < n; ++i) {
        const float x = (float)(raw[i] >> 14);
        hp_y = 0.97f * (hp_y + x - hp_x);  // ~80 Hz high-pass
        hp_x = x;
        // Soft limiter above 24000: loud speech close to the mic bends instead of clipping.
        float y = hp_y;
        const float ay = std::fabs(y);
        if (ay > 24000.0f) y = std::copysign(24000.0f + 8767.0f * std::tanh((ay - 24000.0f) / 8767.0f), y);
        out[i] = (int16_t)y;
    }
    return n;
}

void SetTxRate(int rate) {
    if (rate == g_tx_rate) return;
    i2s_channel_disable(g_tx);
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)rate);
    i2s_channel_reconfig_std_clock(g_tx, &clk);
    i2s_channel_enable(g_tx);
    g_tx_rate = rate;
}

void Write(const int16_t* s, size_t n) {
    size_t written = 0;
    i2s_channel_write(g_tx, s, n * sizeof(int16_t), &written, portMAX_DELAY);
}
}  // namespace

namespace audio {

void Start() {
    g_volume.store(settings::GetInt("audio", "volume", 80));

    i2s_chan_config_t rx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    rx_chan.dma_desc_num = 6;
    rx_chan.dma_frame_num = kFrame;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan, nullptr, &g_rx));
    i2s_std_config_t rx = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {I2S_GPIO_UNUSED, PIN_MIC_SCK, PIN_MIC_WS, I2S_GPIO_UNUSED, PIN_MIC_DIN, {}},
    };
    rx.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_rx, &rx));
    ESP_ERROR_CHECK(i2s_channel_enable(g_rx));

    i2s_chan_config_t tx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_chan.dma_desc_num = 6;
    tx_chan.dma_frame_num = 240;
    tx_chan.auto_clear = true;  // silence when nothing is written
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan, &g_tx, nullptr));
    i2s_std_config_t tx = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {I2S_GPIO_UNUSED, PIN_SPK_BCLK, PIN_SPK_LRCK, PIN_SPK_DOUT, I2S_GPIO_UNUSED, {}},
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_tx, &tx));
    ESP_ERROR_CHECK(i2s_channel_enable(g_tx));
    g_tx_rate = 24000;

    // Initial noise floor: 1 s of the room.
    int16_t f[kFrame];
    float sum = 0;
    int n = 0;
    for (int i = 0; i < 50; ++i) {
        if (ReadFrame(f) == kFrame && i >= 10) {  // skip the INMP441 start-up
            sum += FrameDb(f, kFrame);
            ++n;
        }
    }
    if (n) g_floor_db = sum / n;
    ESP_LOGI(TAG, "mic noise floor %.1f dBFS, volume %d%%", g_floor_db, g_volume.load());
}

bool Listen(std::vector<int16_t>& out, int timeout_ms, int max_ms) {
    out.clear();
    // Drop what queued up while we were speaking or moving (half duplex).
    int16_t f[kFrame];
    for (int i = 0; i < 8; ++i) ReadFrame(f);

    std::vector<int16_t, PsramAlloc<int16_t>> buf;
    buf.reserve(max_ms * 16);
    int voiced = 0, silent = 0, frames = 0;
    bool speaking = false;
    const int timeout_frames = timeout_ms / 20, max_frames = max_ms / 20;
    while (true) {
        if (ReadFrame(f) != kFrame) continue;
        ++frames;
        const float db = FrameDb(f, kFrame);
        const bool loud = db > g_floor_db + kMarginDb;
        // Follow the room slowly while nobody speaks (fast down, slow up).
        if (!speaking && !loud) g_floor_db += (db - g_floor_db) * (db < g_floor_db ? 0.1f : 0.01f);

        buf.insert(buf.end(), f, f + kFrame);
        if (!speaking) {
            voiced = loud ? voiced + 1 : std::max(0, voiced - 1);
            if (voiced >= kStartFrames) {
                speaking = true;
                silent = 0;
            } else if ((int)buf.size() > (kPrerollFrames + voiced) * kFrame) {
                buf.erase(buf.begin(), buf.begin() + kFrame);  // keep only the preroll
            }
            if (!speaking && timeout_frames > 0 && frames >= timeout_frames) return false;
        } else {
            silent = loud ? 0 : silent + 1;
            if (silent >= kEndFrames || (int)(buf.size() / kFrame) >= max_frames) break;
        }
    }
    const size_t keep = buf.size() - std::min<size_t>(buf.size(), (kEndFrames - 10) * kFrame);
    out.assign(buf.begin(), buf.begin() + keep);
    int peak = 0, loud = 0;
    for (int16_t v : out) {
        peak = std::max(peak, std::abs((int)v));
        loud += std::abs((int)v) > 30000;
    }
    ESP_LOGI(TAG, "utterance %.1f s, peak %d (%.1f dBFS), %.2f%% near full scale, floor %.1f dBFS",
             out.size() / 16000.0, peak, 20 * std::log10(peak / 32768.0 + 1e-6), 100.0 * loud / std::max<size_t>(1, out.size()),
             g_floor_db);
    return true;
}

float NoiseFloorDb() { return g_floor_db; }

bool PlayWav(const uint8_t* d, size_t size) {
    if (size < 44 || std::memcmp(d, "RIFF", 4) != 0) return false;
    int rate = 24000, channels = 1, bits = 16;
    size_t pos = 12, data_at = 0, data_len = 0;
    while (pos + 8 <= size) {
        const uint32_t len = d[pos + 4] | d[pos + 5] << 8 | d[pos + 6] << 16 | (uint32_t)d[pos + 7] << 24;
        if (!std::memcmp(d + pos, "fmt ", 4) && pos + 24 <= size) {
            channels = d[pos + 10] | d[pos + 11] << 8;
            rate = d[pos + 12] | d[pos + 13] << 8 | d[pos + 14] << 16;
            bits = d[pos + 22] | d[pos + 23] << 8;
        } else if (!std::memcmp(d + pos, "data", 4)) {
            data_at = pos + 8;
            data_len = std::min<size_t>(len, size - data_at);  // streamed WAVs may say 0xFFFFFFFF
            break;
        }
        pos += 8 + len + (len & 1);
    }
    if (!data_at || bits != 16) return false;

    std::lock_guard<std::mutex> lock(g_tx_mutex);
    SetTxRate(rate);
    g_stop_play.store(false);
    g_playing.store(true);
    const int16_t* s = reinterpret_cast<const int16_t*>(d + data_at);
    const size_t samples = data_len / 2 / channels;
    const int vol = g_volume.load();
    int16_t chunk[256];
    for (size_t i = 0; i < samples && !g_stop_play.load();) {
        const size_t n = std::min<size_t>(256, samples - i);
        for (size_t k = 0; k < n; ++k) chunk[k] = (int16_t)(s[(i + k) * channels] * vol / 100);
        Write(chunk, n);
        i += n;
    }
    std::memset(chunk, 0, sizeof(chunk));
    for (int k = 0; k < 8; ++k) Write(chunk, 256);  // flush the DMA tail
    g_playing.store(false);
    return true;
}

void FlushMic() {
    int16_t f[kFrame];
    for (int i = 0; i < 8; ++i) ReadFrame(f);
}

bool ReadMic(int16_t* pcm, int samples) {
    for (int done = 0; done < samples; done += kFrame) {
        if (ReadFrame(pcm + done) != kFrame) return false;
    }
    return true;
}

void PlayPcm(const int16_t* pcm, size_t samples, int rate) {
    std::lock_guard<std::mutex> lock(g_tx_mutex);
    SetTxRate(rate);
    const int vol = g_volume.load();
    int16_t chunk[256];
    for (size_t i = 0; i < samples;) {
        const size_t n = std::min<size_t>(256, samples - i);
        for (size_t k = 0; k < n; ++k) chunk[k] = (int16_t)(pcm[i + k] * vol / 100);
        Write(chunk, n);
        i += n;
    }
}

void SetStreaming(bool on) { g_playing.store(on); }

void StopPlayback() { g_stop_play.store(true); }
bool Playing() { return g_playing.load(); }

void Beep(int hz, int ms) {
    std::lock_guard<std::mutex> lock(g_tx_mutex);
    SetTxRate(24000);
    int16_t chunk[240];
    const int vol = g_volume.load();
    for (int t = 0; t < ms * 24;) {
        for (int k = 0; k < 240; ++k, ++t) chunk[k] = (int16_t)(std::sin(2 * M_PI * hz * t / 24000.0) * 8000 * vol / 100);
        Write(chunk, 240);
    }
}

int Volume() { return g_volume.load(); }
void SetVolume(int percent) {
    g_volume.store(std::clamp(percent, 0, 100));
    settings::SetInt("audio", "volume", g_volume.load());
}

}  // namespace audio
