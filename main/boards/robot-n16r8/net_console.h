#ifndef _ROBOT_N16R8_NET_CONSOLE_H_
#define _ROBOT_N16R8_NET_CONSOLE_H_

#include <cstdarg>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

// Wi-Fi console on a TCP port (connect with `nc <ip> 23`): mirrors the live ESP log and
// runs line commands. One client at a time; commands run in the console task.
class NetConsole {
public:
    using Args = std::vector<std::string>;
    using Handler = std::function<void(const Args& args)>;

    static NetConsole& GetInstance();

    void Start(uint16_t port);
    void AddCommand(const std::string& name, const std::string& usage, Handler handler);
    // Text to the connected client (not to the serial log).
    void Print(const char* format, ...) __attribute__((format(printf, 2, 3)));
    bool LogEnabled() const { return log_enabled_; }

private:
    NetConsole() = default;

    struct Command {
        std::string usage;
        Handler handler;
    };

    uint16_t port_ = 23;
    std::map<std::string, Command> commands_;
    StreamBufferHandle_t log_buffer_ = nullptr;
    SemaphoreHandle_t log_mutex_ = nullptr;
    TaskHandle_t server_task_ = nullptr;
    TaskHandle_t sender_task_ = nullptr;
    volatile int client_fd_ = -1;
    volatile bool log_enabled_ = true;
    char log_line_[256];

    static int LogHook(const char* format, va_list args);
    void ServerLoop();
    void SenderLoop();
    void HandleLine(const std::string& line);
    void SendRaw(const char* data, size_t length);
};

#endif  // _ROBOT_N16R8_NET_CONSOLE_H_
