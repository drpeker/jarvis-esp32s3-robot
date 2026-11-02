#ifndef BOARD_H
#define BOARD_H

#include <http.h>
#include <mqtt.h>
#include <network_interface.h>
#include <udp.h>
#include <web_socket.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "assets.h"
#include "backlight.h"
#include "camera.h"
#include "led/led.h"

/**
 * Network events for unified callback
 */
enum class NetworkEvent {
    Scanning,             // Network is scanning (WiFi scanning, etc.)
    Connecting,           // Network is connecting (data: SSID/network name)
    Connected,            // Network connected successfully (data: SSID/network name)
    Disconnected,         // Network disconnected
    WifiConfigModeEnter,  // Entered WiFi configuration mode
    WifiConfigModeExit,   // Exited WiFi configuration mode
    // Cellular modem specific events
    ModemDetecting,        // Detecting modem (baud rate, module type)
    ModemErrorNoSim,       // No SIM card detected
    ModemErrorRegDenied,   // Network registration denied
    ModemErrorInitFailed,  // Modem initialization failed
    ModemErrorTimeout      // Operation timeout
};

// Power save level enumeration
enum class PowerSaveLevel {
    LOW_POWER,    // Maximum power saving (lowest power consumption)
    BALANCED,     // Medium power saving (balanced)
    PERFORMANCE,  // No power saving (maximum power consumption / full performance)
};

// Network event callback type (event, data)
// data contains additional info like SSID for Connecting/Connected events
using NetworkEventCallback = std::function<void(NetworkEvent event, const std::string& data)>;

void* create_board();
class AudioCodec;
class Display;

struct StartupAudio {
    const int16_t* samples = nullptr;
    size_t sample_count = 0;
    int sample_rate = 0;
};

class Board {
private:
    Board(const Board&) = delete;             // 禁用拷贝构造函数
    Board& operator=(const Board&) = delete;  // 禁用赋值操作

protected:
    Board();
    std::string GenerateUuid();

    // 软件生成的设备唯一标识
    std::string uuid_;

public:
    static Board& GetInstance() {
        static Board* instance = static_cast<Board*>(create_board());
        return *instance;
    }

    virtual ~Board() = default;
    virtual std::string GetBoardType() = 0;
    virtual std::string GetUuid() { return uuid_; }
    virtual Backlight* GetBacklight() { return nullptr; }
    virtual Led* GetLed();
    virtual AudioCodec* GetAudioCodec() = 0;
    virtual bool GetTemperature(float& esp32temp);
    virtual Display* GetDisplay();
    virtual Camera* GetCamera();
    virtual NetworkInterface* GetNetwork() = 0;
    virtual void StartNetwork() = 0;
    virtual void SetNetworkEventCallback(NetworkEventCallback callback) { (void)callback; }
    virtual const char* GetNetworkStateIcon() = 0;
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging);
    virtual std::string GetSystemInfoJson();
    virtual void SetPowerSaveLevel(PowerSaveLevel level) = 0;
    virtual std::string GetBoardJson() = 0;
    virtual std::string GetDeviceStatusJson() = 0;
    virtual StartupAudio GetStartupAudio() { return {}; }
    virtual void OnStartupReady() {}
    virtual void OnAssistantSpeaking(bool speaking) { (void)speaking; }
    virtual void OnAssistantEmotion(const std::string& emotion) { (void)emotion; }
    // Optional board-local handling for final speech-to-text results. The
    // callback may run outside the application task, so implementations must
    // schedule mutations through Application::Schedule().
    virtual void OnUserText(const std::string& text) { (void)text; }
    // Boards with a dedicated local-control mode can use this hook to keep the
    // shared application state machine from restarting WakeNet while that mode
    // owns the microphone, camera and motion controls.
    virtual bool IsManualControlActive() const { return false; }
};

#define DECLARE_BOARD(BOARD_CLASS_NAME) \
    void* create_board() { return new BOARD_CLASS_NAME(); }

#endif  // BOARD_H
