#pragma once

#include <array>
#include <mutex>
#include <string>

#include <driver/i2c_master.h>

#include "display.h"

// SH1106 stores a 128x64 image and cannot rotate it by 90 degrees. This class
// maps a 64x128 logical canvas into its RAM for Wall-E's portrait mounting.
class WallEPortraitDisplay final : public Display {
public:
    explicit WallEPortraitDisplay(i2c_master_bus_handle_t bus);
    ~WallEPortraitDisplay() override;

    void SetupUI() override;
    void SetStatus(const char* status) override;
    void ShowNotification(const char* notification, int duration_ms = 3000) override;
    void SetChatMessage(const char* role, const char* content) override;
    void ClearChatMessages() override;
    void SetEmotion(const char* emotion) override;

private:
    bool Lock(int timeout_ms = 0) override;
    void Unlock() override;
    void Command(uint8_t command);
    void Render();
    void Pixel(int x, int y);
    void Text(int x, int y, const std::string& text);
    static std::string Normalize(const char* text);

    i2c_master_dev_handle_t device_ = nullptr;
    std::recursive_mutex mutex_;
    std::array<uint8_t, 128 * 8> frame_{};
    std::string status_ = "WALL-E AI";
    std::string message_;
};
