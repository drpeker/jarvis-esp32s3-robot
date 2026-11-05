#include "wifi.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <esp_event.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "face.h"
#include "settings.h"
#include "status.h"
#include "wifi_portal.h"

#define TAG "wifi"

namespace {

constexpr int kSlots = 10;                  // ssid, ssid1 .. ssid9
constexpr int kFallbackMs = 30000;          // no connection by then: open the setup network
constexpr int kRetryMs = 300;               // next attempt after a failure
constexpr int kPortalRetryMs = 60000;       // ... while the setup network is open
constexpr int kJoinAttempts = 3;            // tries for a network chosen on the page

enum Cmd : uint8_t { kTryKnown, kScanForList, kScanDone, kDisconnected, kGotIp, kFallback, kClosePortal, kJoin };
struct Msg {
    Cmd cmd;
    int reason;
};

QueueHandle_t g_queue;
EventGroupHandle_t g_events;
constexpr EventBits_t kConnected = 1;
esp_timer_handle_t g_retry_timer, g_fallback_timer, g_close_timer;

// Shared with the portal (httpd task): under g_lock.
std::mutex g_lock;
std::vector<std::pair<std::string, std::string>> g_networks;
std::vector<wifi::Network> g_scan;
std::string g_join_ssid, g_join_pass, g_error;

// Only the wifi task touches these.
bool g_scanning = false;
bool g_scan_to_connect = false;
bool g_connecting = false;
int g_join_left = 0;  // > 0: a portal network is being tried

void Post(Cmd c, int reason = 0) {
    Msg m{c, reason};
    xQueueSend(g_queue, &m, 0);
}

void Arm(esp_timer_handle_t t, int ms) {
    esp_timer_stop(t);
    esp_timer_start_once(t, (uint64_t)ms * 1000);
}

bool Connected() { return xEventGroupGetBits(g_events) & kConnected; }

void LoadNetworks() {
    std::lock_guard<std::mutex> l(g_lock);
    g_networks.clear();
    for (int i = 0; i < kSlots; ++i) {
        const std::string sfx = i == 0 ? "" : std::to_string(i);
        std::string ssid = settings::GetStr("wifi", ("ssid" + sfx).c_str());
        if (!ssid.empty()) g_networks.push_back({ssid, settings::GetStr("wifi", ("password" + sfx).c_str())});
    }
}

// Same name: new password in its slot; otherwise the first free slot (slot 0 if full).
void StoreNetwork(const std::string& ssid, const std::string& pass) {
    int slot = -1, free_slot = -1;
    for (int i = 0; i < kSlots; ++i) {
        const std::string sfx = i == 0 ? "" : std::to_string(i);
        const std::string s = settings::GetStr("wifi", ("ssid" + sfx).c_str());
        if (s == ssid) slot = i;
        if (s.empty() && free_slot < 0) free_slot = i;
    }
    if (slot < 0) slot = free_slot >= 0 ? free_slot : 0;
    const std::string sfx = slot == 0 ? "" : std::to_string(slot);
    settings::SetStr("wifi", ("ssid" + sfx).c_str(), ssid);
    settings::SetStr("wifi", ("password" + sfx).c_str(), pass);
    ESP_LOGI(TAG, "stored network %s in slot %d", ssid.c_str(), slot);
}

void ForgetNetwork(const std::string& ssid) {
    for (int i = 0; i < kSlots; ++i) {
        const std::string sfx = i == 0 ? "" : std::to_string(i);
        if (settings::GetStr("wifi", ("ssid" + sfx).c_str()) == ssid) {
            settings::SetStr("wifi", ("ssid" + sfx).c_str(), "");
            settings::SetStr("wifi", ("password" + sfx).c_str(), "");
        }
    }
}

void StartScan(bool to_connect) {
    g_scan_to_connect = g_scan_to_connect || to_connect;
    if (g_scanning) return;
    wifi_scan_config_t sc = {};
    sc.show_hidden = false;
    if (esp_wifi_scan_start(&sc, false) == ESP_OK) {
        g_scanning = true;
    } else {
        Arm(g_retry_timer, 1000);  // busy (connecting): again shortly
    }
}

void OpenPortal() {
    if (wifi_portal::Running()) return;
    ESP_LOGW(TAG, "no network joined: opening the setup network %s", wifi_portal::kSsid);
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    wifi_config_t ap = {};
    std::strncpy((char*)ap.ap.ssid, wifi_portal::kSsid, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len = std::strlen(wifi_portal::kSsid);
    ap.ap.authmode = WIFI_AUTH_OPEN;  // no password: join from the phone at once
    ap.ap.max_connection = 2;
    ap.ap.channel = 1;  // follows the station's channel while it is connecting
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    wifi_portal::Start();
    face::SetCaption("system", std::string("Wi-Fi kurulumu: telefonla ") + wifi_portal::kSsid +
                                   " ağına bağlan (şifresiz)");
    StartScan(false);
}

void ClosePortal() {
    if (!wifi_portal::Running()) return;
    ESP_LOGI(TAG, "connected: closing the setup network");
    wifi_portal::Stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    face::SetCaption("system", "");
}

void OnScanDone() {
    g_scanning = false;
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    auto* aps = n ? (wifi_ap_record_t*)heap_caps_calloc(n, sizeof(wifi_ap_record_t), MALLOC_CAP_SPIRAM) : nullptr;
    if (!aps) n = 0;
    esp_wifi_scan_get_ap_records(&n, aps);  // also frees the driver's list
    std::sort(aps, aps + n, [](const wifi_ap_record_t& a, const wifi_ap_record_t& b) { return a.rssi > b.rssi; });

    std::string want_ssid, want_pass;
    int best = -1;
    {
        std::lock_guard<std::mutex> l(g_lock);
        g_scan.clear();
        for (int i = 0; i < n; ++i) {
            const std::string ssid((const char*)aps[i].ssid);
            if (ssid.empty()) continue;
            if (std::none_of(g_scan.begin(), g_scan.end(), [&](const wifi::Network& x) { return x.ssid == ssid; })) {
                g_scan.push_back({ssid, aps[i].rssi, aps[i].authmode != WIFI_AUTH_OPEN});
            }
        }
        // Strongest AP of: the network chosen on the page, else any known network.
        for (int i = 0; i < n && best < 0; ++i) {
            const char* s = (const char*)aps[i].ssid;
            if (g_join_left > 0) {
                if (g_join_ssid == s) best = i, want_ssid = g_join_ssid, want_pass = g_join_pass;
            } else {
                for (auto& [ssid, pass] : g_networks) {
                    if (ssid == s) {
                        best = i, want_ssid = ssid, want_pass = pass;
                        break;
                    }
                }
            }
        }
    }

    const bool to_connect = g_scan_to_connect;
    g_scan_to_connect = false;
    if (to_connect && !Connected() && !g_connecting) {
        if (best < 0) {
            if (g_join_left > 0) {
                std::lock_guard<std::mutex> l(g_lock);
                g_error = "Ağ bulunamadı: " + g_join_ssid;
                g_join_left = 0;
            }
            Arm(g_retry_timer, wifi_portal::Running() ? kPortalRetryMs : 2000);
        } else {
            const wifi_ap_record_t& ap = aps[best];
            wifi_config_t cfg = {};
            std::strncpy((char*)cfg.sta.ssid, want_ssid.c_str(), sizeof(cfg.sta.ssid) - 1);
            std::strncpy((char*)cfg.sta.password, want_pass.c_str(), sizeof(cfg.sta.password) - 1);
            std::memcpy(cfg.sta.bssid, ap.bssid, 6);
            cfg.sta.bssid_set = true;
            cfg.sta.channel = ap.primary;
            cfg.sta.failure_retry_cnt = 0;
            cfg.sta.threshold.authmode = want_pass.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
            cfg.sta.listen_interval = 10;
            ESP_LOGI(TAG, "connecting to %s %02x:%02x:%02x:%02x:%02x:%02x ch %d rssi %d", want_ssid.c_str(),
                     ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5], ap.primary,
                     ap.rssi);
            esp_wifi_set_config(WIFI_IF_STA, &cfg);
            esp_wifi_disable_pmf_config(WIFI_IF_STA);
            if (esp_wifi_connect() == ESP_OK) {
                g_connecting = true;
            } else {
                Arm(g_retry_timer, 1000);
            }
        }
    }
    heap_caps_free(aps);
}

void OnDisconnected(int reason) {
    const bool was_connected = Connected();
    xEventGroupClearBits(g_events, kConnected);
    status::SetWifi(false, -100);
    const bool was_attempt = g_connecting;
    g_connecting = false;
    ESP_LOGW(TAG, "disconnected, reason %d", reason);
    if (was_attempt && g_join_left > 0 && --g_join_left == 0) {
        std::lock_guard<std::mutex> l(g_lock);
        const bool bad_password = reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT || reason == WIFI_REASON_HANDSHAKE_TIMEOUT ||
                                  reason == WIFI_REASON_AUTH_FAIL || reason == WIFI_REASON_MIC_FAILURE;
        g_error = (bad_password ? "Şifre yanlış olabilir: " : "Bağlanılamadı: ") + g_join_ssid;
        if (bad_password) {
            ForgetNetwork(g_join_ssid);
            g_networks.erase(std::remove_if(g_networks.begin(), g_networks.end(),
                                            [](auto& x) { return x.first == g_join_ssid; }),
                             g_networks.end());
        }
        ESP_LOGW(TAG, "%s", g_error.c_str());
    }
    if (was_connected && !wifi_portal::Running()) Arm(g_fallback_timer, kFallbackMs);
    Arm(g_retry_timer, wifi_portal::Running() && g_join_left == 0 ? kPortalRetryMs : kRetryMs);
}

void WifiTask(void*) {
    Msg m;
    while (true) {
        if (xQueueReceive(g_queue, &m, portMAX_DELAY) != pdTRUE) continue;
        switch (m.cmd) {
            case kTryKnown:
                if (Connected() || g_connecting) break;
                if (wifi_portal::InUse() && g_join_left == 0) {
                    Arm(g_retry_timer, kPortalRetryMs);  // don't hop channels under a phone
                    break;
                }
                StartScan(true);
                break;
            case kScanForList:
                StartScan(false);
                break;
            case kScanDone:
                OnScanDone();
                break;
            case kDisconnected:
                OnDisconnected(m.reason);
                break;
            case kGotIp: {
                g_connecting = false;
                xEventGroupSetBits(g_events, kConnected);
                esp_timer_stop(g_fallback_timer);
                wifi_ap_record_t ap = {};
                esp_wifi_sta_get_ap_info(&ap);
                status::SetWifi(true, ap.rssi);
                ESP_LOGI(TAG, "connected to %s, RSSI %d", (const char*)ap.ssid, ap.rssi);
                if (g_join_left > 0) {
                    g_join_left = 0;
                    std::lock_guard<std::mutex> l(g_lock);
                    g_error.clear();
                }
                // Leave the page a moment to show the result, then close the setup network.
                if (wifi_portal::Running()) Arm(g_close_timer, 5000);
                break;
            }
            case kFallback:
                if (!Connected()) OpenPortal();
                break;
            case kClosePortal:
                if (Connected()) ClosePortal();
                break;
            case kJoin: {
                std::string ssid, pass;
                {
                    std::lock_guard<std::mutex> l(g_lock);
                    ssid = g_join_ssid, pass = g_join_pass;
                    g_error.clear();
                }
                StoreNetwork(ssid, pass);
                LoadNetworks();
                g_join_left = kJoinAttempts;
                if (g_connecting || Connected()) {
                    esp_wifi_disconnect();  // the disconnect event starts the attempt
                } else {
                    StartScan(true);
                }
                break;
            }
        }
    }
}

void OnEvent(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        Post(kTryKnown);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
        Post(kScanDone);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* d = static_cast<wifi_event_sta_disconnected_t*>(data);
        Post(kDisconnected, d ? d->reason : 0);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        Post(kGotIp);
    }
}

void RssiTask(void*) {  // RSSI for the face icon
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        wifi_ap_record_t ap = {};
        if (Connected() && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) status::SetWifi(true, ap.rssi);
    }
}

esp_timer_handle_t MakeTimer(const char* name, Cmd cmd) {
    esp_timer_create_args_t a = {};
    a.callback = [](void* arg) { Post((Cmd)(uintptr_t)arg); };
    a.arg = (void*)(uintptr_t)cmd;
    a.name = name;
    esp_timer_handle_t t = nullptr;
    esp_timer_create(&a, &t);
    return t;
}

}  // namespace

namespace wifi {

void Start() {
    LoadNetworks();
    ESP_LOGI(TAG, "%u known network(s)", (unsigned)g_networks.size());
    g_queue = xQueueCreate(16, sizeof(Msg));
    g_events = xEventGroupCreate();
    g_retry_timer = MakeTimer("wifi_retry", kTryKnown);
    g_fallback_timer = MakeTimer("wifi_fallback", kFallback);
    g_close_timer = MakeTimer("wifi_close", kClosePortal);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, OnEvent, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, OnEvent, nullptr);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);

    // Internal-RAM stack: this task writes NVS (networks chosen on the page).
    xTaskCreatePinnedToCore(WifiTask, "wifi", 4096, nullptr, 5, nullptr, 0);
    xTaskCreate(RssiTask, "rssi", 2048, nullptr, 1, nullptr);

    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);  // low latency for voice; the robot is always powered
    if (g_networks.empty()) {
        Post(kFallback);  // nothing stored: setup network at once
    } else {
        Arm(g_fallback_timer, kFallbackMs);
    }
}

bool WaitConnected(int timeout_ms) {
    return xEventGroupWaitBits(g_events, kConnected, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)) & kConnected;
}

std::vector<Network> LastScan() {
    std::lock_guard<std::mutex> l(g_lock);
    return g_scan;
}

void RequestScan() { Post(kScanForList); }

void Join(const std::string& ssid, const std::string& password) {
    {
        std::lock_guard<std::mutex> l(g_lock);
        g_join_ssid = ssid;
        g_join_pass = password;
    }
    Post(kJoin);
}

std::string LastError() {
    std::lock_guard<std::mutex> l(g_lock);
    return g_error;
}

}  // namespace wifi
