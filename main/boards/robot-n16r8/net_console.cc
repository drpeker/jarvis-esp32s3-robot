#include "net_console.h"

#include <cstdio>
#include <cstring>
#include <sstream>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/idf_additions.h>
#include <esp_netif.h>
#include <lwip/sockets.h>

#include "wifi_manager.h"

#define TAG "NetConsole"

namespace {
vprintf_like_t g_original_vprintf = nullptr;
constexpr size_t kLogBufferSize = 6144;
}  // namespace

NetConsole& NetConsole::GetInstance() {
    static NetConsole instance;
    return instance;
}

void NetConsole::Start(uint16_t port) {
    if (server_task_ != nullptr) {
        return;
    }
    port_ = port;
    // Internal RAM is tight (TLS, camera and MCP need it), so the log buffer and both task
    // stacks live in PSRAM. Commands that write flash hand the work to the main task.
    log_buffer_ = xStreamBufferCreateWithCaps(kLogBufferSize, 1, MALLOC_CAP_SPIRAM);
    log_mutex_ = xSemaphoreCreateMutex();
    AddCommand("help", "help", [this](const Args&) {
        Print("Commands:\r\n");
        for (const auto& [name, command] : commands_) {
            Print("  %s\r\n", command.usage.c_str());
        }
    });
    AddCommand("log", "log on|off  - live log stream", [this](const Args& args) {
        if (args.size() > 1) {
            log_enabled_ = args[1] == "on";
        }
        Print("log %s\r\n", log_enabled_ ? "on" : "off");
    });
    g_original_vprintf = esp_log_set_vprintf(LogHook);
    xTaskCreateWithCaps([](void* arg) { static_cast<NetConsole*>(arg)->ServerLoop(); },
                        "net_console", 5120, this, 3, &server_task_, MALLOC_CAP_SPIRAM);
    xTaskCreateWithCaps([](void* arg) { static_cast<NetConsole*>(arg)->SenderLoop(); },
                        "net_console_tx", 3072, this, 2, &sender_task_, MALLOC_CAP_SPIRAM);
}

void NetConsole::AddCommand(const std::string& name, const std::string& usage, Handler handler) {
    commands_[name] = {usage, std::move(handler)};
}

int NetConsole::LogHook(const char* format, va_list args) {
    auto& console = GetInstance();
    int result = 0;
    if (g_original_vprintf != nullptr) {
        va_list copy;
        va_copy(copy, args);
        result = g_original_vprintf(format, copy);
        va_end(copy);
    }
    // Never block a logging task; drop the line when busy or nobody is listening. The
    // sender task's own logs are skipped so a send error cannot feed itself.
    if (console.client_fd_ < 0 || !console.log_enabled_ || xPortInIsrContext() ||
        xTaskGetCurrentTaskHandle() == console.sender_task_) {
        return result;
    }
    if (xSemaphoreTake(console.log_mutex_, 0) == pdTRUE) {
        const int n = vsnprintf(console.log_line_, sizeof(console.log_line_), format, args);
        if (n > 0) {
            xStreamBufferSend(console.log_buffer_, console.log_line_,
                              std::min<size_t>(n, sizeof(console.log_line_) - 1), 0);
        }
        xSemaphoreGive(console.log_mutex_);
    }
    return result;
}

void NetConsole::Print(const char* format, ...) {
    char buffer[384];
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (n > 0) {
        SendRaw(buffer, std::min<size_t>(n, sizeof(buffer) - 1));
    }
}

void NetConsole::SendRaw(const char* data, size_t length) {
    const int fd = client_fd_;
    if (fd >= 0) {
        send(fd, data, length, 0);
    }
}

void NetConsole::SenderLoop() {
    char chunk[512];
    while (true) {
        const size_t n = xStreamBufferReceive(log_buffer_, chunk, sizeof(chunk), portMAX_DELAY);
        if (n > 0) {
            SendRaw(chunk, n);
        }
    }
}

void NetConsole::ServerLoop() {
    while (!WifiManager::GetInstance().IsConnected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int reuse = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "Console port %u could not be opened", port_);
        vTaskDelete(nullptr);
        return;
    }
    esp_netif_ip_info_t ip = {};
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);
    ESP_LOGI(TAG, "Console ready: nc " IPSTR " %u", IP2STR(&ip.ip), port_);

    while (true) {
        sockaddr_in peer = {};
        socklen_t peer_length = sizeof(peer);
        const int fd = accept(listen_fd, reinterpret_cast<sockaddr*>(&peer), &peer_length);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        timeval timeout = {.tv_sec = 2, .tv_usec = 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        int no_delay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
        xStreamBufferReset(log_buffer_);
        client_fd_ = fd;
        Print("\r\nJarvis console. 'help' lists commands, 'log off' pauses the live log.\r\n");

        std::string line;
        char buffer[128];
        while (true) {
            const int n = recv(fd, buffer, sizeof(buffer), 0);
            if (n <= 0) {
                break;
            }
            for (int i = 0; i < n; ++i) {
                const char c = buffer[i];
                if (c == '\n' || c == '\r') {
                    if (!line.empty()) {
                        HandleLine(line);
                        line.clear();
                    }
                } else if (line.size() < 200) {
                    line += c;
                }
            }
        }
        client_fd_ = -1;
        close(fd);
    }
}

void NetConsole::HandleLine(const std::string& line) {
    Args args;
    std::istringstream stream(line);
    std::string word;
    while (stream >> word) {
        args.push_back(word);
    }
    if (args.empty()) {
        return;
    }
    auto it = commands_.find(args[0]);
    if (it == commands_.end()) {
        Print("unknown command '%s' (help)\r\n", args[0].c_str());
        return;
    }
    Print("> %s\r\n", line.c_str());
    it->second.handler(args);
}
