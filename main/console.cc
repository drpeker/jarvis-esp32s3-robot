#include "console.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <esp_crt_bundle.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include "audio.h"
#include "camera.h"
#include "conversation.h"
#include "crash.h"
#include "imu.h"
#include "motors.h"
#include "power.h"
#include "settings.h"
#include "status.h"
#include "tools.h"

#define TAG "console"

namespace {

StreamBufferHandle_t g_log = nullptr;
// Last log lines, always kept (PSRAM ring) so a test can be read after it happened.
constexpr size_t kHistory = 24 * 1024;
char* g_hist = nullptr;
size_t g_hist_pos = 0;
bool g_hist_wrapped = false;
portMUX_TYPE g_hist_lock = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_log_on = false;
int g_client = -1;
vprintf_like_t g_uart_vprintf = nullptr;

// Every log line also goes to the console client when 'log on' (never blocks).
int LogHook(const char* fmt, va_list args) {
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    n = std::min(n, (int)sizeof(line) - 1);
    if (n > 0 && g_hist != nullptr) {
        portENTER_CRITICAL(&g_hist_lock);
        for (int i = 0; i < n; ++i) {
            g_hist[g_hist_pos++] = line[i];
            if (g_hist_pos == kHistory) {
                g_hist_pos = 0;
                g_hist_wrapped = true;
            }
        }
        portEXIT_CRITICAL(&g_hist_lock);
    }
    if (n > 0 && g_log_on && g_log != nullptr) xStreamBufferSend(g_log, line, n, 0);
    return g_uart_vprintf(fmt, args);
}

// The last `lines` lines of the history.
void PrintHistory(int lines) {
    std::string text;
    portENTER_CRITICAL(&g_hist_lock);
    const size_t pos = g_hist_pos;
    const bool wrapped = g_hist_wrapped;
    portEXIT_CRITICAL(&g_hist_lock);
    if (wrapped) text.assign(g_hist + pos, kHistory - pos);
    text.append(g_hist, pos);
    size_t start = text.size();
    for (int count = 0; start > 0 && count <= lines; --start) {
        if (text[start - 1] == '\n') ++count;
        if (count > lines) break;
    }
    for (size_t i = start; i < text.size(); i += 400) {
        const std::string part = text.substr(i, 400);
        send(g_client, part.data(), part.size(), 0);
    }
}

void Print(const char* fmt, ...) {
    if (g_client < 0) return;
    char buf[512];
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) send(g_client, buf, std::min(n, (int)sizeof(buf) - 1), 0);
}

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
        size_t j = i;
        while (j < line.size() && line[j] != ' ' && line[j] != '\t') ++j;
        if (j > i) out.push_back(line.substr(i, j - i));
        i = j;
    }
    return out;
}

int Arg(const std::vector<std::string>& a, size_t i, int fallback) {
    return a.size() > i ? std::atoi(a[i].c_str()) : fallback;
}

std::string Rest(const std::string& line, size_t skip_words) {
    size_t pos = 0;
    for (size_t w = 0; w < skip_words; ++w) {
        pos = line.find_first_not_of(' ', pos);
        pos = line.find(' ', pos);
        if (pos == std::string::npos) return "";
    }
    pos = line.find_first_not_of(' ', pos);
    return pos == std::string::npos ? "" : line.substr(pos);
}

void PrintTuning() {
    const motors::Tuning t = motors::GetTuning();
    Print("min_duty L %d/%d R %d/%d %%, gain L %d R %d, turn_speed %d%%, lead %.1f (learned %.1f) deg,\r\n"
          "kick %d ms, brake %d ms, pwm %d Hz, dist_speed %d%%, 20 cm = fwd %d / back %d ms\r\n",
          t.min_duty[0][0], t.min_duty[0][1], t.min_duty[1][0], t.min_duty[1][1], t.gain[0], t.gain[1],
          t.turn_speed, t.lead_x10 / 10.0f, motors::TurnLead(), t.kick_ms, t.brake_ms, t.pwm_hz, t.dist_speed,
          t.ms20[0], t.ms20[1]);
}

void Ota(const std::string& url) {
    Print("OTA from %s ...\r\n", url.c_str());
    motors::Stop();
    status::Set(RobotState::kUpdating);
    esp_http_client_config_t http = {};
    http.url = url.c_str();
    http.timeout_ms = 15000;
    http.crt_bundle_attach = esp_crt_bundle_attach;
    http.keep_alive_enable = true;
    esp_https_ota_config_t cfg = {};
    cfg.http_config = &http;
    const esp_err_t err = esp_https_ota(&cfg);
    if (err == ESP_OK) {
        Print("OTA done, rebooting\r\n");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }
    Print("OTA FAILED: %s\r\n", esp_err_to_name(err));
    status::Set(RobotState::kIdle);
}

void Help() {
    Print("status | crash [clear] | log on|off | log last [lines] | cam [sweep [upload]] | wake | convo\r\n"
          "turn <l|r> [deg] | dist <f|b> [cm] | drive <f|b> <ms> [speed%%] | stop | imu\r\n"
          "tune | set <key> <value> | save | gyro_scale <value> | volume <0-100>\r\n"
          "wifi <ssid> <password>\r\n"
          "ota <http://host:port/jarvis.bin> | reboot\r\n"
          "set keys: min_lf min_lr min_rf min_rr gain_l gain_r turn_speed lead kick brake pwm dist_speed ms20_fwd ms20_back\r\n");
}

void Handle(const std::string& line) {
    const auto a = Split(line);
    if (a.empty()) return;
    const std::string& c = a[0];
    if (c == "help") {
        Help();
    } else if (c == "status") {
        const power::Reading b = power::Get();
        Print("state %s, reset %s, uptime %d s, Wi-Fi %s %d dBm\r\n", status::Name(status::Get()), crash::ResetReason(),
              (int)(esp_timer_get_time() / 1000000), status::WifiConnected() ? "up" : "down", status::WifiRssi());
        Print("battery %d mV %d mA %d%% %s, motors %s%s\r\n", b.mv, b.ma, b.percent,
              b.critical ? "CRITICAL" : b.low ? "low" : "ok", motors::Moving() ? "moving" : "stopped",
              motors::Locked() ? " LOCKED" : "");
        Print("imu %s heading %.1f tilt %.1f, read errors %d; mic floor %.1f dBFS, volume %d%%\r\n",
              imu::Ready() ? "ok" : "off", imu::Heading(), imu::TiltDegrees(), imu::ReadErrors(), audio::NoiseFloorDb(),
              audio::Volume());
        // Largest DMA block: the camera needs 16 KB in one piece.
        Print("RAM internal %u (min %u, largest DMA block %u), PSRAM %u (min %u)\r\n",
              heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
              heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
              heap_caps_get_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
        Print("%s\r\n", conversation::Describe().c_str());
        PrintTuning();
    } else if (c == "crash") {
        if (a.size() > 1 && a[1] == "clear") {
            crash::Clear();
            Print("cleared\r\n");
        } else {
            Print("%s\r\n", crash::Describe().c_str());
        }
    } else if (c == "log" && a.size() > 1 && a[1] == "last") {
        PrintHistory(Arg(a, 2, 60));
    } else if (c == "log") {
        g_log_on = a.size() < 2 || a[1] != "off";
        Print("log %s\r\n", g_log_on ? "on" : "off");
    } else if (c == "wake") {
        conversation::Wake(false);
        Print("wake sent\r\n");
    } else if (c == "convo") {
        Print("%s\r\n", conversation::Describe().c_str());
    } else if (c == "cam" && a.size() > 1 && a[1] == "sweep") {
        // Every OV2640 size: JPEG bytes, capture time; with 'upload' also the vision upload.
        const bool upload = a.size() > 2 && a[2] == "upload";
        static const struct { framesize_t size; const char* name; } kSizes[] = {
            {FRAMESIZE_QQVGA, "QQVGA"}, {FRAMESIZE_QCIF, "QCIF"},   {FRAMESIZE_HQVGA, "HQVGA"},
            {FRAMESIZE_240X240, "240x240"}, {FRAMESIZE_QVGA, "QVGA"}, {FRAMESIZE_CIF, "CIF"},
            {FRAMESIZE_HVGA, "HVGA"},   {FRAMESIZE_VGA, "VGA"},     {FRAMESIZE_SVGA, "SVGA"},
            {FRAMESIZE_XGA, "XGA"},     {FRAMESIZE_HD, "HD"},       {FRAMESIZE_SXGA, "SXGA"},
            {FRAMESIZE_UXGA, "UXGA"}};
        Print("size      WxH        bytes  capture_ms%s\r\n", upload ? "  upload_ms  answer_ms" : "");
        for (const auto& s : kSizes) {
            std::vector<uint8_t> jpeg;
            int w = 0, h = 0;
            const int64_t t0 = esp_timer_get_time();
            const bool ok = camera::TakePhoto(jpeg, &w, &h, s.size);
            const int cap = (int)((esp_timer_get_time() - t0) / 1000);
            if (!ok) {
                Print("%-8s  FAILED\r\n", s.name);
                continue;
            }
            Print("%-8s  %4dx%-4d  %6u  %5d", s.name, w, h, (unsigned)jpeg.size(), cap);
            if (upload) {
                int up = 0, total = 0;
                const bool uok = tools::MeasureUpload(jpeg, &up, &total);
                Print("  %9d  %9d%s", up, total, uok ? "" : "  (failed)");
            }
            Print("\r\n");
        }
    } else if (c == "cam") {
        std::vector<uint8_t> jpeg;
        int w = 0, h = 0;
        const int64_t t0 = esp_timer_get_time();
        const bool ok = camera::TakePhoto(jpeg, &w, &h);
        Print("photo %s: %dx%d, %u bytes, %d ms\r\n", ok ? "OK" : "FAILED", w, h, (unsigned)jpeg.size(),
              (int)((esp_timer_get_time() - t0) / 1000));
    } else if (c == "turn" && a.size() > 1) {
        const motors::Result r = motors::Turn(a[1][0] == 'l', Arg(a, 2, 90));
        Print("turn -> %.1f deg %s\r\n", r.value, r.note);
    } else if (c == "dist" && a.size() > 1) {
        const motors::Result r = motors::Distance(a[1][0] == 'f', Arg(a, 2, 20));
        Print("dist -> %.1f cm %s\r\n", r.value, r.note);
    } else if (c == "drive" && a.size() > 2) {
        const motors::Result r = motors::Drive(a[1][0] == 'f', Arg(a, 2, 500), Arg(a, 3, 50));
        Print("drive -> %.0f ms %s\r\n", r.value, r.note);
    } else if (c == "stop") {
        motors::Stop();
        Print("stopped\r\n");
    } else if (c == "imu") {
        float up[3], acc[3];
        imu::Debug(up, acc);
        Print("up (%.2f %.2f %.2f), accel now (%.2f %.2f %.2f) g\r\n", up[0], up[1], up[2], acc[0], acc[1], acc[2]);
        for (int i = 0; i < 10; ++i) {
            Print("heading %7.1f deg, rate %6.1f dps, tilt %4.1f\r\n", imu::Heading(), imu::YawRate(), imu::TiltDegrees());
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    } else if (c == "tune") {
        PrintTuning();
    } else if (c == "set" && a.size() > 2) {
        motors::Tuning t = motors::GetTuning();
        const std::string& k = a[1];
        const int v = Arg(a, 2, 0);
        if (k == "min_lf") t.min_duty[0][0] = v;
        else if (k == "min_lr") t.min_duty[0][1] = v;
        else if (k == "min_rf") t.min_duty[1][0] = v;
        else if (k == "min_rr") t.min_duty[1][1] = v;
        else if (k == "gain_l") t.gain[0] = v;
        else if (k == "gain_r") t.gain[1] = v;
        else if (k == "turn_speed") t.turn_speed = v;
        else if (k == "lead") t.lead_x10 = v;
        else if (k == "kick") t.kick_ms = v;
        else if (k == "brake") t.brake_ms = v;
        else if (k == "pwm") t.pwm_hz = v;
        else if (k == "dist_speed") t.dist_speed = v;
        else if (k == "ms20_fwd") t.ms20[0] = v;
        else if (k == "ms20_back") t.ms20[1] = v;
        else return Print("unknown key\r\n");
        motors::SetTuning(t, false);
        PrintTuning();
    } else if (c == "save") {
        motors::SetTuning(motors::GetTuning(), true);
        Print("saved\r\n");
    } else if (c == "gyro_scale" && a.size() > 1) {
        imu::SetYawScale(std::atof(a[1].c_str()));
        Print("yaw scale %.4f\r\n", imu::YawScale());
    } else if (c == "volume" && a.size() > 1) {
        audio::SetVolume(Arg(a, 1, 80));
        Print("volume %d%%\r\n", audio::Volume());
    } else if (c == "wifi" && a.size() > 2) {
        settings::SetStr("wifi", "ssid", a[1]);
        settings::SetStr("wifi", "password", Rest(line, 2));
        Print("stored, reboot to use it\r\n");
    } else if (c == "ota" && a.size() > 1) {
        Ota(a[1]);
    } else if (c == "reboot" && a.size() > 1 && a[1] == "cold") {
        // Like a power cut: reset without stopping Wi-Fi (no deauth sent to the AP).
        Print("cold reset\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_rom_software_reset_system();
    } else if (c == "reboot") {
        Print("rebooting\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        Print("unknown command, 'help' lists them\r\n");
    }
}

void Serve(int client) {
    g_client = client;
    Print("Jarvis console (v1.7). 'help' lists commands.\r\n> ");
    std::string line;
    char buf[128];
    while (true) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(client, &fds);
        timeval tv = {0, 50000};
        const int r = select(client + 1, &fds, nullptr, nullptr, &tv);
        if (r < 0) break;
        if (r > 0) {
            const int n = recv(client, buf, sizeof(buf), 0);
            if (n <= 0) break;
            for (int i = 0; i < n; ++i) {
                if (buf[i] == '\n') {
                    Handle(line);
                    line.clear();
                    Print("> ");
                } else if (buf[i] != '\r' && line.size() < 512) {
                    line += buf[i];
                }
            }
        }
        char out[256];
        size_t got;
        while (g_log_on && (got = xStreamBufferReceive(g_log, out, sizeof(out), 0)) > 0) {
            if (send(client, out, got, 0) < 0) break;
        }
    }
    g_log_on = false;
    g_client = -1;
    close(client);
}

void Task(void*) {
    const int server = socket(AF_INET, SOCK_STREAM, 0);
    const int yes = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(23);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(server, (sockaddr*)&addr, sizeof(addr));
    listen(server, 1);
    while (true) {
        const int client = accept(server, nullptr, nullptr);
        if (client >= 0) Serve(client);
    }
}

}  // namespace

namespace console {

void Start() {
    g_log = xStreamBufferCreate(4096, 1);
    g_hist = (char*)heap_caps_calloc(1, kHistory, MALLOC_CAP_SPIRAM);
    g_uart_vprintf = esp_log_set_vprintf(LogHook);
    xTaskCreatePinnedToCore(Task, "console", 6144, nullptr, 2, nullptr, 0);
}

}  // namespace console
