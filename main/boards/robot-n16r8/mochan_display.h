#pragma once

#include "display/lcd_display.h"

#include <esp_timer.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

class MochanDisplay : public SpiLcdDisplay {
public:
    MochanDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                  int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                  bool swap_xy);
    ~MochanDisplay() override;

    void SetupUI() override;
    void SetStatus(const char* status) override;
    void SetEmotion(const char* emotion) override;
    void SetChatMessage(const char* role, const char* content) override;
    void ClearChatMessages() override;
    void SetPreviewImage(std::unique_ptr<LvglImage> image) override;
    void SetTheme(Theme* theme) override;

private:
    static constexpr int kStatusBarHeight = 18;
    static constexpr int kEyeHeight = 100;
    static constexpr int kTypewriterPeriodMs = 35;
    static constexpr int kCursorBlinkPeriodMs = 420;
    static constexpr int kTerminalPopupPeriodMs = 220;

    lv_timer_t* typewriter_timer_ = nullptr;
    lv_timer_t* cursor_timer_ = nullptr;
    lv_obj_t* state_dot_ = nullptr;
    lv_obj_t* camera_box_ = nullptr;
    lv_obj_t* loading_overlay_ = nullptr;
    lv_obj_t* loading_spinner_ = nullptr;
    lv_obj_t* loading_label_ = nullptr;
    esp_timer_handle_t idle_eye_timer_ = nullptr;
    std::string typewriter_target_;
    std::string typewriter_visible_;
    std::string current_eye_;
    std::string last_idle_accent_;
    size_t typewriter_position_ = 0;
    std::atomic_bool preview_visible_ = false;
    std::atomic_bool idle_accent_next_ = true;
    bool boot_message_pending_ = true;
    bool boot_loading_visible_ = true;
    bool terminal_prompt_active_ = false;
    bool terminal_cursor_enabled_ = false;
    bool terminal_cursor_visible_ = true;
    bool terminal_has_content_ = false;
    std::mutex eye_mutex_;
    std::mutex idle_timer_mutex_;

    static void TypewriterTimerCallback(lv_timer_t* timer);
    static void CursorTimerCallback(lv_timer_t* timer);
    static void TerminalScaleAnimCallback(void* object, int32_t value);
    static void IdleEyeTimerCallback(void* arg);

    void AdvanceTypewriter();
    void BlinkTerminalCursor();
    void RenderTerminalTextLocked();
    void ShowTerminalPopupLocked();
    void HideTerminalLocked();
    void HideBootLoadingLocked();
    void ShowTextLocked(const char* content);
    void ScrollTextToBottomLocked();
    void ScheduleIdleEye(uint32_t delay_ms);
    void ShowRandomIdleEye();
    void SetEyeIfChanged(const char* eye, int loop_count = 0, bool restart_if_same = false);
    const char* MapEmotionToEye(const char* emotion) const;
};
