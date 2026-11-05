#include "wifi_portal.h"

#include <atomic>
#include <cstring>
#include <string>

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include "wifi.h"

#define TAG "portal"

namespace {

constexpr const char* kIp = "192.168.4.1";  // default address of the ESP-IDF access point
constexpr const char* kUrl = "http://192.168.4.1/";

httpd_handle_t g_http = nullptr;
std::atomic<bool> g_running{false};
std::atomic<int> g_dns_sock{-1};  // for the console status
TaskHandle_t g_dns_task = nullptr;

// ---- DNS: every name is the robot, so the phone's connectivity check lands on the page ----

void DnsTask(void*) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (sock < 0 || bind(sock, (sockaddr*)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "DNS socket failed");
        if (sock >= 0) close(sock);
        g_dns_task = nullptr;
        vTaskDelete(nullptr);
    }
    timeval tv = {1, 0};  // wake up every second to see whether the portal was closed
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    g_dns_sock = sock;
    const uint32_t ip = inet_addr(kIp);
    uint8_t buf[512];
    while (g_running) {
        sockaddr_in from = {};
        socklen_t from_len = sizeof(from);
        const int n = recvfrom(sock, buf, sizeof(buf) - 16, 0, (sockaddr*)&from, &from_len);
        if (n < 12) continue;  // closed by Stop(), or not a DNS message
        if (buf[2] & 0x80) continue;  // a response, not a query
        // Skip the question (name labels, then type and class).
        int p = 12;
        while (p < n && buf[p] != 0) p += buf[p] + 1;
        p += 5;
        if (p > n) continue;
        const bool type_a = buf[p - 4] == 0 && buf[p - 3] == 1;
        buf[2] = 0x81;  // response, recursion desired
        buf[3] = 0x80;  // recursion available, no error
        buf[6] = 0, buf[7] = type_a ? 1 : 0;  // answers
        buf[8] = buf[9] = buf[10] = buf[11] = 0;
        int len = p;
        if (type_a) {
            const uint8_t answer[] = {0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, 60, 0, 4};
            std::memcpy(buf + len, answer, sizeof(answer));
            len += sizeof(answer);
            std::memcpy(buf + len, &ip, 4);
            len += 4;
        }
        sendto(sock, buf, len, 0, (sockaddr*)&from, from_len);
    }
    close(sock);
    g_dns_sock = -1;
    g_dns_task = nullptr;
    vTaskDelete(nullptr);
}

// ---- page -------------------------------------------------------------------------------

std::string Escape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
            case '&': o += "&amp;"; break;
            case '<': o += "&lt;"; break;
            case '>': o += "&gt;"; break;
            case '"': o += "&quot;"; break;
            case '\'': o += "&#39;"; break;
            default: o += c;
        }
    }
    return o;
}

std::string UrlDecode(const std::string& s) {
    std::string o;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '+') {
            o += ' ';
        } else if (s[i] == '%' && i + 2 < s.size()) {
            o += (char)strtol(s.substr(i + 1, 2).c_str(), nullptr, 16);
            i += 2;
        } else {
            o += s[i];
        }
    }
    return o;
}

std::string FormField(const std::string& body, const char* name) {
    const std::string key = std::string(name) + "=";
    size_t p = 0;
    while (p < body.size()) {
        size_t end = body.find('&', p);
        if (end == std::string::npos) end = body.size();
        if (body.compare(p, key.size(), key) == 0) return UrlDecode(body.substr(p + key.size(), end - p - key.size()));
        p = end + 1;
    }
    return "";
}

const char* kHead =
    "<!doctype html><html lang=\"tr\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Jarvis Wi-Fi</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#0d1117;color:#e6edf3}"
    "h1{font-size:20px;margin:0 0 4px}p{color:#9da7b3;margin:0 0 16px}"
    "label.net{display:flex;justify-content:space-between;padding:12px;border:1px solid #30363d;"
    "border-radius:8px;margin-bottom:8px;background:#161b22}label.net input{margin-right:10px}"
    ".sig{color:#9da7b3;white-space:nowrap}"
    "input[type=password],input[type=text]{width:100%;box-sizing:border-box;padding:12px;font-size:16px;"
    "border-radius:8px;border:1px solid #30363d;background:#0d1117;color:#e6edf3;margin:8px 0}"
    "button{width:100%;padding:14px;font-size:16px;border:0;border-radius:8px;background:#28a8ff;color:#fff}"
    "a.btn{display:block;text-align:center;margin-top:12px;color:#28a8ff}"
    ".err{background:#3d1214;color:#ff9b9b;padding:10px;border-radius:8px;margin-bottom:12px}"
    "</style></head><body>";

esp_err_t SendPage(httpd_req_t* req, const std::string& body) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const std::string page = kHead + body + "</body></html>";
    return httpd_resp_send(req, page.data(), page.size());
}

esp_err_t Root(httpd_req_t* req) {
    std::string b = "<h1>Jarvis Wi-Fi kurulumu</h1><p>Robotun bağlanacağı ağı seç ve şifresini yaz.</p>";
    const std::string err = wifi::LastError();
    if (!err.empty()) b += "<div class=\"err\">" + Escape(err) + "</div>";
    b += "<form method=\"post\" action=\"/baglan\">";
    const auto nets = wifi::LastScan();
    bool first = true;
    for (const auto& n : nets) {
        const int bars = n.rssi >= -55 ? 4 : n.rssi >= -67 ? 3 : n.rssi >= -78 ? 2 : 1;
        static const char* kBars[] = {"", "▂", "▂▄", "▂▄▆", "▂▄▆█"};
        b += "<label class=\"net\"><span><input type=\"radio\" name=\"ssid\" value=\"" + Escape(n.ssid) + "\"" +
             (first ? " checked" : "") + ">" + Escape(n.ssid) + (n.secure ? " 🔒" : "") +
             "</span><span class=\"sig\">" + kBars[bars] + " " + std::to_string(n.rssi) + " dBm</span></label>";
        first = false;
    }
    if (nets.empty()) b += "<p>Ağ listesi hazırlanıyor, birkaç saniye sonra yeniden tara.</p>";
    b += "<label class=\"net\"><span><input type=\"radio\" name=\"ssid\" value=\"__diger__\"" +
         std::string(nets.empty() ? " checked" : "") +
         ">Listede yok (gizli ağ)</span></label>"
         "<input type=\"text\" name=\"other\" placeholder=\"Gizli ağın adı\" autocomplete=\"off\">"
         "<input type=\"password\" name=\"password\" placeholder=\"Şifre (açık ağda boş bırak)\" autocomplete=\"off\">"
         "<button type=\"submit\">Bağlan</button></form>"
         "<a class=\"btn\" href=\"/tara\">Yeniden tara</a>";
    return SendPage(req, b);
}

esp_err_t Scan(httpd_req_t* req) {
    wifi::RequestScan();
    vTaskDelay(pdMS_TO_TICKS(3500));  // a full scan takes about 2.5 s
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t Connect(httpd_req_t* req) {
    const int len = req->content_len;
    if (len <= 0 || len > 512) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad form");
    std::string body(len, '\0');
    int got = 0;
    while (got < len) {
        const int r = httpd_req_recv(req, &body[got], len - got);
        if (r <= 0) return ESP_FAIL;
        got += r;
    }
    std::string ssid = FormField(body, "ssid");
    if (ssid == "__diger__") ssid = FormField(body, "other");
    const std::string pass = FormField(body, "password");
    if (ssid.empty() || ssid.size() > 32 || pass.size() > 63 || (!pass.empty() && pass.size() < 8)) {
        return SendPage(req, "<h1>Geçersiz</h1><p>Ağ adı 1-32, şifre 8-63 karakter olmalı (açık ağda boş).</p>"
                             "<a class=\"btn\" href=\"/\">Geri</a>");
    }
    ESP_LOGI(TAG, "network chosen: %s", ssid.c_str());
    wifi::Join(ssid, pass);
    return SendPage(req, "<h1>Bağlanılıyor: " + Escape(ssid) +
                             "</h1><p>Robot bu ağa geçiyor. Bağlanırsa kurulum ağı kapanır ve telefon ondan kopar; "
                             "robotun ekranında Wi-Fi simgesi görünür.</p><p>Bağlanamazsa bu sayfaya yeniden gir "
                             "(<a href=\"/\" style=\"color:#28a8ff\">yenile</a>), hata mesajını göreceksin.</p>");
}

// Every other address (the phones' connectivity checks: /generate_204, /hotspot-detect.html,
// /connecttest.txt, ...) is sent to the page: that is what makes the phone open it.
esp_err_t Redirect(httpd_req_t* req, httpd_err_code_t) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", kUrl);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, nullptr, 0);
}

}  // namespace

namespace wifi_portal {

void Start() {
    if (g_running) return;
    g_running = true;

    // DHCP option 114 (RFC 8910): phones that support it open the page directly.
    esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap) {
        esp_netif_dhcps_stop(ap);
        esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, (void*)kUrl, std::strlen(kUrl));
        esp_netif_dhcps_start(ap);
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    cfg.max_open_sockets = 5;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&g_http, &cfg) == ESP_OK) {
        httpd_uri_t root = {"/", HTTP_GET, Root, nullptr};
        httpd_uri_t scan = {"/tara", HTTP_GET, Scan, nullptr};
        httpd_uri_t connect = {"/baglan", HTTP_POST, Connect, nullptr};
        httpd_register_uri_handler(g_http, &root);
        httpd_register_uri_handler(g_http, &scan);
        httpd_register_uri_handler(g_http, &connect);
        httpd_register_err_handler(g_http, HTTPD_404_NOT_FOUND, Redirect);
    } else {
        ESP_LOGE(TAG, "HTTP server failed");
    }
    xTaskCreatePinnedToCore(DnsTask, "dns", 3072, nullptr, 3, &g_dns_task, 0);
    ESP_LOGI(TAG, "setup network %s open, page %s", kSsid, kUrl);
}

void Stop() {
    if (!g_running) return;
    g_running = false;
    // The DNS task sees g_running within a second and closes its socket.
    if (g_http) {
        httpd_stop(g_http);
        g_http = nullptr;
    }
    ESP_LOGI(TAG, "setup network closed");
}

bool Running() { return g_running; }

bool InUse() {
    if (!g_running) return false;
    wifi_sta_list_t list = {};
    return esp_wifi_ap_get_sta_list(&list) == ESP_OK && list.num > 0;
}

}  // namespace wifi_portal
