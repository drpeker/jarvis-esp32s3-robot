#include "xiaozhi_link.h"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

#include <cJSON.h>
#include <esp_app_desc.h>
#include <esp_crt_bundle.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <mbedtls/aes.h>
#include <mqtt_client.h>

#include "face.h"
#include "identity.h"
#include "settings.h"

#define TAG "xiaozhi"

namespace {

constexpr const char* kDefaultOtaUrl = "https://api.tenclass.net/xiaozhi/ota/";
constexpr const char* kBoardType = "robot-n16r8";
constexpr const char* kBoardName = "Robot";
constexpr const char* kLanguage = "en-US";
constexpr EventBits_t kHelloBit = 1, kConnectedBit = 2;
constexpr int kHeader = 16;

std::string JsonStr(const cJSON* o, const char* k) {
    const cJSON* v = cJSON_GetObjectItem(o, k);
    return cJSON_IsString(v) ? v->valuestring : "";
}

int HexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// No exceptions in this build: parse by hand, reject anything that is not n bytes of hex.
bool Hex(const std::string& hex, uint8_t* out, size_t n) {
    if (hex.size() != n * 2) return false;
    for (size_t i = 0; i < n; ++i) {
        const int hi = HexDigit(hex[i * 2]), lo = HexDigit(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return true;
}

class XiaozhiLink : public ServerLink {
public:
    bool Start(const Events& events) override;
    bool Connected() const override { return xEventGroupGetBits(events_bits_) & kConnectedBit; }
    bool OpenSession() override;
    void CloseSession() override;
    bool SessionOpen() const override { return udp_fd_.load() >= 0; }
    int SpeakerSampleRate() const override { return speaker_rate_; }
    void SendWakeWord(const std::string& word) override;
    void StartListening() override;
    void StopListening() override;
    void AbortSpeaking() override;
    bool SendAudio(const uint8_t* packet, size_t len) override;
    void SendMcp(const std::string& payload) override;
    std::string Describe() const override;

private:
    Events ev_;
    EventGroupHandle_t events_bits_ = xEventGroupCreate();
    esp_mqtt_client_handle_t mqtt_ = nullptr;
    std::string publish_topic_;
    std::string session_id_;
    std::mutex text_mutex_;
    char* rx_buf_ = nullptr;  // MQTT messages may arrive in several parts (PSRAM)
    size_t rx_cap_ = 0, rx_len_ = 0;

    // UDP audio channel
    // Session socket: -1 when no session. The UDP task owns the socket it was started with
    // and closes it itself once udp_fd_ no longer names it (lwip's shutdown() does nothing
    // on UDP sockets, so a blocked recv() can only end by its timeout).
    std::atomic<int> udp_fd_{-1};
    std::atomic<int> udp_tasks_{0};
    std::string udp_host_;
    int udp_port_ = 0;
    uint8_t key_[16] = {}, nonce_[16] = {};
    mbedtls_aes_context aes_tx_, aes_rx_;
    uint32_t tx_seq_ = 0, rx_seq_ = 0;
    int speaker_rate_ = 24000;
    int64_t session_start_us_ = 0;

    bool FetchConfig(std::string& endpoint, std::string& client_id, std::string& user, std::string& pass,
                     int& keepalive);
    std::string DeviceJson() const;
    void Publish(const std::string& text);
    void OnMessage(const char* data, size_t len);
    void ParseHello(const cJSON* root);
    void UdpLoop();
    static void MqttEvent(void* arg, esp_event_base_t, int32_t id, void* data);
};

std::string XiaozhiLink::DeviceJson() const {
    const esp_app_desc_t* app = esp_app_get_description();
    wifi_ap_record_t ap = {};
    esp_wifi_sta_get_ap_info(&ap);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", 2);
    cJSON_AddStringToObject(root, "language", kLanguage);
    cJSON_AddNumberToObject(root, "flash_size", 16 * 1024 * 1024);
    cJSON_AddStringToObject(root, "mac_address", identity::Mac().c_str());
    cJSON_AddStringToObject(root, "uuid", identity::Uuid().c_str());
    cJSON_AddStringToObject(root, "chip_model_name", "esp32s3");
    cJSON* a = cJSON_AddObjectToObject(root, "application");
    cJSON_AddStringToObject(a, "name", app->project_name);
    cJSON_AddStringToObject(a, "version", app->version);
    cJSON_AddStringToObject(a, "idf_version", app->idf_ver);
    cJSON* b = cJSON_AddObjectToObject(root, "board");
    cJSON_AddStringToObject(b, "type", kBoardType);
    cJSON_AddStringToObject(b, "name", kBoardName);
    cJSON_AddStringToObject(b, "ssid", (const char*)ap.ssid);
    cJSON_AddNumberToObject(b, "rssi", ap.rssi);
    cJSON_AddStringToObject(b, "mac", identity::Mac().c_str());
    char* s = cJSON_PrintUnformatted(root);
    std::string out = s;
    cJSON_free(s);
    cJSON_Delete(root);
    return out;
}

// POST the device description; keep asking while the server shows an activation code.
bool XiaozhiLink::FetchConfig(std::string& endpoint, std::string& client_id, std::string& user,
                              std::string& pass, int& keepalive) {
    const std::string url = settings::GetStr("xiaozhi", "ota_url", kDefaultOtaUrl);
    const std::string body = DeviceJson();
    const std::string agent = std::string(kBoardName) + "/" + esp_app_get_description()->version;
    for (int attempt = 0;; ++attempt) {
        esp_http_client_config_t cfg = {};
        cfg.url = url.c_str();
        cfg.method = HTTP_METHOD_POST;
        cfg.crt_bundle_attach = esp_crt_bundle_attach;
        cfg.timeout_ms = 15000;
        esp_http_client_handle_t c = esp_http_client_init(&cfg);
        esp_http_client_set_header(c, "Activation-Version", "1");
        esp_http_client_set_header(c, "Device-Id", identity::Mac().c_str());
        esp_http_client_set_header(c, "Client-Id", identity::Uuid().c_str());
        esp_http_client_set_header(c, "User-Agent", agent.c_str());
        esp_http_client_set_header(c, "Accept-Language", kLanguage);
        esp_http_client_set_header(c, "Content-Type", "application/json");
        std::string resp;
        int status = -1;
        if (esp_http_client_open(c, body.size()) == ESP_OK &&
            esp_http_client_write(c, body.data(), body.size()) == (int)body.size() &&
            esp_http_client_fetch_headers(c) >= 0) {
            status = esp_http_client_get_status_code(c);
            char buf[512];
            int n;
            while ((n = esp_http_client_read(c, buf, sizeof(buf))) > 0) resp.append(buf, n);
        }
        esp_http_client_close(c);
        esp_http_client_cleanup(c);
        cJSON* root = status == 200 ? cJSON_Parse(resp.c_str()) : nullptr;
        if (!root) {
            ESP_LOGW(TAG, "configuration request failed (HTTP %d), retry in 10 s", status);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        const cJSON* act = cJSON_GetObjectItem(root, "activation");
        if (cJSON_IsObject(act)) {
            // Not bound to an account yet: show the code, ask again until it is entered.
            const std::string code = JsonStr(act, "code");
            ESP_LOGW(TAG, "activation code %s", code.c_str());
            face::SetCaption("system", "Activation code: " + code);
            cJSON_Delete(root);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        const cJSON* mqtt = cJSON_GetObjectItem(root, "mqtt");
        endpoint = JsonStr(mqtt, "endpoint");
        client_id = JsonStr(mqtt, "client_id");
        user = JsonStr(mqtt, "username");
        pass = JsonStr(mqtt, "password");
        publish_topic_ = JsonStr(mqtt, "publish_topic");
        const cJSON* ka = cJSON_GetObjectItem(mqtt, "keepalive");
        keepalive = cJSON_IsNumber(ka) ? ka->valueint : 240;
        cJSON_Delete(root);
        if (endpoint.empty()) {
            ESP_LOGE(TAG, "the server sent no MQTT configuration");
            return false;
        }
        ESP_LOGI(TAG, "configuration ok: MQTT %s", endpoint.c_str());
        return true;
    }
}

bool XiaozhiLink::Start(const Events& events) {
    ev_ = events;
    mbedtls_aes_init(&aes_tx_);
    mbedtls_aes_init(&aes_rx_);
    std::string endpoint, client_id, user, pass;
    int keepalive = 240;
    if (!FetchConfig(endpoint, client_id, user, pass, keepalive)) return false;
    std::string host = endpoint;
    int port = 8883;
    const size_t colon = endpoint.find(':');
    if (colon != std::string::npos) {
        host = endpoint.substr(0, colon);
        port = std::atoi(endpoint.c_str() + colon + 1);
    }
    esp_mqtt_client_config_t cfg = {};
    const std::string uri = "mqtts://" + host + ":" + std::to_string(port);
    cfg.broker.address.uri = uri.c_str();
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.credentials.client_id = client_id.c_str();
    cfg.credentials.username = user.c_str();
    cfg.credentials.authentication.password = pass.c_str();
    cfg.session.keepalive = keepalive;
    cfg.network.reconnect_timeout_ms = 5000;
    cfg.buffer.size = 2048;
    cfg.task.stack_size = 6144;
    mqtt_ = esp_mqtt_client_init(&cfg);  // copies the strings
    if (!mqtt_) return false;
    esp_mqtt_client_register_event(mqtt_, MQTT_EVENT_ANY, MqttEvent, this);
    esp_mqtt_client_start(mqtt_);
    return xEventGroupWaitBits(events_bits_, kConnectedBit, pdFALSE, pdTRUE, pdMS_TO_TICKS(15000)) & kConnectedBit;
}

void XiaozhiLink::MqttEvent(void* arg, esp_event_base_t, int32_t id, void* data) {
    auto* self = static_cast<XiaozhiLink*>(arg);
    auto* e = static_cast<esp_mqtt_event_handle_t>(data);
    if (id == MQTT_EVENT_CONNECTED) {
        xEventGroupSetBits(self->events_bits_, kConnectedBit);
        ESP_LOGI(TAG, "MQTT connected");
    } else if (id == MQTT_EVENT_DISCONNECTED) {
        xEventGroupClearBits(self->events_bits_, kConnectedBit);
        ESP_LOGW(TAG, "MQTT disconnected");
    } else if (id == MQTT_EVENT_DATA) {
        // Reassemble messages longer than the MQTT buffer.
        if (e->current_data_offset == 0) self->rx_len_ = 0;
        const size_t need = e->total_data_len + 1;
        if (need > self->rx_cap_) {
            heap_caps_free(self->rx_buf_);
            self->rx_buf_ = (char*)heap_caps_malloc(need, MALLOC_CAP_SPIRAM);
            self->rx_cap_ = self->rx_buf_ ? need : 0;
        }
        if (!self->rx_buf_ || e->current_data_offset + e->data_len > (int)self->rx_cap_ - 1) return;
        memcpy(self->rx_buf_ + e->current_data_offset, e->data, e->data_len);
        self->rx_len_ = e->current_data_offset + e->data_len;
        if ((int)self->rx_len_ == e->total_data_len) {
            self->rx_buf_[self->rx_len_] = 0;
            self->OnMessage(self->rx_buf_, self->rx_len_);
        }
    }
}

void XiaozhiLink::OnMessage(const char* data, size_t len) {
    cJSON* root = cJSON_ParseWithLength(data, len);
    if (!root) return;
    const std::string type = JsonStr(root, "type");
    if (type == "hello") {
        ParseHello(root);
    } else if (type == "goodbye") {
        if (JsonStr(root, "session_id") == session_id_) {
            ESP_LOGI(TAG, "server closed the session");
            const int fd = udp_fd_.exchange(-1);
            (void)fd;  // the UDP task sees this within 200 ms and closes the socket
            if (ev_.on_session_closed) ev_.on_session_closed();
        }
    } else if (type == "tts") {
        const std::string state = JsonStr(root, "state");
        if (state == "start" && ev_.on_speak_start) ev_.on_speak_start();
        else if (state == "stop" && ev_.on_speak_stop) ev_.on_speak_stop();
        else if (state == "sentence_start" && ev_.on_robot_text) ev_.on_robot_text(JsonStr(root, "text"));
    } else if (type == "stt") {
        if (ev_.on_user_text) ev_.on_user_text(JsonStr(root, "text"));
    } else if (type == "llm") {
        if (ev_.on_emotion) ev_.on_emotion(JsonStr(root, "emotion"));
    } else if (type == "mcp") {
        const cJSON* p = cJSON_GetObjectItem(root, "payload");
        if (p && ev_.on_mcp) {
            char* s = cJSON_PrintUnformatted(p);
            ev_.on_mcp(s);
            cJSON_free(s);
        }
    } else {
        ESP_LOGI(TAG, "message %s ignored", type.c_str());
    }
    cJSON_Delete(root);
}

void XiaozhiLink::ParseHello(const cJSON* root) {
    if (JsonStr(root, "transport") != "udp") return;
    session_id_ = JsonStr(root, "session_id");
    const cJSON* ap = cJSON_GetObjectItem(root, "audio_params");
    const cJSON* sr = cJSON_GetObjectItem(ap, "sample_rate");
    if (cJSON_IsNumber(sr)) speaker_rate_ = sr->valueint;
    const cJSON* udp = cJSON_GetObjectItem(root, "udp");
    const cJSON* port = cJSON_GetObjectItem(udp, "port");
    udp_host_ = JsonStr(udp, "server");
    udp_port_ = cJSON_IsNumber(port) ? port->valueint : 0;
    if (udp_host_.empty() || udp_port_ <= 0 || !Hex(JsonStr(udp, "key"), key_, 16) ||
        !Hex(JsonStr(udp, "nonce"), nonce_, 16)) {
        ESP_LOGE(TAG, "server hello without a usable UDP channel");
        return;
    }
    mbedtls_aes_setkey_enc(&aes_tx_, key_, 128);
    mbedtls_aes_setkey_enc(&aes_rx_, key_, 128);  // CTR decrypts with the encryption key
    xEventGroupSetBits(events_bits_, kHelloBit);
}

void XiaozhiLink::Publish(const std::string& text) {
    std::lock_guard<std::mutex> lock(text_mutex_);
    if (mqtt_ && !publish_topic_.empty()) esp_mqtt_client_publish(mqtt_, publish_topic_.c_str(), text.data(), text.size(), 0, 0);
}

bool XiaozhiLink::OpenSession() {
    if (SessionOpen()) return true;
    if (!Connected()) return false;
    xEventGroupClearBits(events_bits_, kHelloBit);
    session_id_.clear();
    Publish("{\"type\":\"hello\",\"version\":3,\"transport\":\"udp\",\"features\":{\"mcp\":true},"
            "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,\"channels\":1,\"frame_duration\":60}}");
    if (!(xEventGroupWaitBits(events_bits_, kHelloBit, pdTRUE, pdTRUE, pdMS_TO_TICKS(10000)) & kHelloBit)) {
        ESP_LOGE(TAG, "no hello from the server");
        return false;
    }
    // The previous session's UDP task must be gone first: one socket and one task at a time.
    for (int i = 0; i < 50 && udp_tasks_.load() > 0; ++i) vTaskDelay(pdMS_TO_TICKS(20));
    if (udp_tasks_.load() > 0) {
        ESP_LOGE(TAG, "previous UDP task still running");
        return false;
    }
    addrinfo hints = {}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(udp_host_.c_str(), std::to_string(udp_port_).c_str(), &hints, &res) != 0 || !res) return false;
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    const bool ok = fd >= 0 && connect(fd, res->ai_addr, res->ai_addrlen) == 0;
    freeaddrinfo(res);
    if (!ok) {
        if (fd >= 0) close(fd);
        return false;
    }
    timeval tv = {0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    tx_seq_ = rx_seq_ = 0;
    session_start_us_ = esp_timer_get_time();
    udp_fd_.store(fd);
    udp_tasks_.fetch_add(1);
    if (xTaskCreatePinnedToCore([](void* a) { static_cast<XiaozhiLink*>(a)->UdpLoop(); }, "udp_rx", 4096, this, 6,
                                nullptr, 0) != pdPASS) {
        udp_tasks_.fetch_sub(1);
        udp_fd_.store(-1);
        close(fd);
        return false;
    }
    ESP_LOGI(TAG, "session %s open, speaker %d Hz", session_id_.c_str(), speaker_rate_);
    return true;
}

void XiaozhiLink::UdpLoop() {
    const int fd = udp_fd_.load();
    uint8_t* buf = (uint8_t*)heap_caps_malloc(1500, MALLOC_CAP_INTERNAL);
    uint8_t* pcm = (uint8_t*)heap_caps_malloc(1500, MALLOC_CAP_INTERNAL);
    while (buf && pcm && udp_fd_.load() == fd) {
        const int n = recv(fd, buf, 1500, 0);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;  // timeout: check the session
        if (n <= 0) break;
        if (n < kHeader || buf[0] != 0x01) continue;
        const uint16_t plen = (buf[2] << 8) | buf[3];
        const uint32_t seq = (uint32_t)buf[12] << 24 | buf[13] << 16 | buf[14] << 8 | buf[15];
        if (n != kHeader + plen || seq <= rx_seq_) continue;  // malformed or old
        rx_seq_ = seq;
        uint8_t counter[16], stream[16];
        size_t off = 0;
        memcpy(counter, buf, 16);
        mbedtls_aes_crypt_ctr(&aes_rx_, plen, &off, counter, stream, buf + kHeader, pcm);
        if (ev_.on_audio) ev_.on_audio(pcm, plen);
    }
    heap_caps_free(buf);
    heap_caps_free(pcm);
    close(fd);
    int expected = fd;
    udp_fd_.compare_exchange_strong(expected, -1);  // socket error: the session is over too
    udp_tasks_.fetch_sub(1);
    vTaskDelete(nullptr);
}

bool XiaozhiLink::SendAudio(const uint8_t* packet, size_t len) {
    const int fd = udp_fd_.load();
    if (fd < 0 || len > 1400) return false;
    uint8_t out[kHeader + 1400];
    memcpy(out, nonce_, 16);
    out[2] = len >> 8;
    out[3] = len & 0xFF;
    const uint32_t ts = (uint32_t)((esp_timer_get_time() - session_start_us_) / 1000);
    const uint32_t seq = ++tx_seq_;
    for (int i = 0; i < 4; ++i) {
        out[8 + i] = ts >> (24 - 8 * i);
        out[12 + i] = seq >> (24 - 8 * i);
    }
    uint8_t counter[16], stream[16];
    size_t off = 0;
    memcpy(counter, out, 16);
    mbedtls_aes_crypt_ctr(&aes_tx_, len, &off, counter, stream, packet, out + kHeader);
    return send(fd, out, kHeader + len, 0) == (int)(kHeader + len);
}

void XiaozhiLink::CloseSession() {
    if (!session_id_.empty()) Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"goodbye\"}");
    udp_fd_.store(-1);  // the UDP task closes its socket within 200 ms
    session_id_.clear();
}

void XiaozhiLink::SendWakeWord(const std::string& word) {
    Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"listen\",\"state\":\"detect\",\"text\":\"" + word + "\"}");
}
void XiaozhiLink::StartListening() {
    Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"listen\",\"state\":\"start\",\"mode\":\"auto\"}");
}
void XiaozhiLink::StopListening() {
    Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"listen\",\"state\":\"stop\"}");
}
void XiaozhiLink::AbortSpeaking() {
    Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"abort\"}");
}
void XiaozhiLink::SendMcp(const std::string& payload) {
    Publish("{\"session_id\":\"" + session_id_ + "\",\"type\":\"mcp\",\"payload\":" + payload + "}");
}

std::string XiaozhiLink::Describe() const {
    return std::string("xiaozhi: MQTT ") + (Connected() ? "up" : "down") + ", session " +
           (SessionOpen() ? session_id_ : std::string("closed")) + ", speaker " + std::to_string(speaker_rate_) + " Hz, UDP tasks " +
           std::to_string(udp_tasks_.load());
}

}  // namespace

ServerLink* CreateXiaozhiLink() { return new XiaozhiLink(); }
