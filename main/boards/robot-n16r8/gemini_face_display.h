#pragma once

#include "display/lcd_display.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

// Robot face ported from NeuroVision-Gemini (nv/display.py): eyes, expressions, blink,
// idle glances, mouth animations, Wi-Fi and battery icons, captions and the "Peker"
// splash, with the same layout rules and colors. Drawn row-span by row-span into a
// full-screen RGB565 canvas, only the rows that change are invalidated.
class GeminiFaceDisplay : public SpiLcdDisplay {
public:
    // percent 0..100, level 0 good / 1 getting low / 2 low / 3 very low.
    using BatteryProvider = std::function<bool(int& percent, int& level)>;

    GeminiFaceDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                      int width, int height, int offset_x, int offset_y, bool mirror_x,
                      bool mirror_y, bool swap_xy);
    ~GeminiFaceDisplay() override;

    void SetBatteryProvider(BatteryProvider provider);

    void SetupUI() override;
    void SetStatus(const char* status) override;
    void ShowNotification(const char* notification, int duration_ms = 3000) override;
    void ShowNotification(const std::string& notification, int duration_ms = 3000) override;
    void SetEmotion(const char* emotion) override;
    void SetChatMessage(const char* role, const char* content) override;
    void ClearChatMessages() override;
    void SetPreviewImage(std::unique_ptr<LvglImage> image) override;
    void SetTheme(Theme* theme) override;

private:
    enum FaceState { kOffline, kLooking, kSpeaking, kThinking, kListening, kIdle };
    enum Emotion { kNeutral, kHappy, kSad, kSurprised, kAngry, kSleepy };
    enum CaptionColor { kCaptionUser, kCaptionRobot, kCaptionWarn };
    struct Span {
        float x0;
        float x1;
    };

    // Shared with the callers' tasks (guarded by state_mutex_).
    std::mutex state_mutex_;
    Emotion emotion_ = kNeutral;
    double emotion_until_ = 0;
    double thinking_since_ = -1;
    double flash_until_ = 0;
    std::string caption_text_;
    CaptionColor caption_color_ = kCaptionUser;
    bool caption_sticky_ = false;  // setup messages stay until replaced
    double caption_at_ = 0;
    bool caption_dirty_ = false;
    std::unique_ptr<LvglImage> pending_preview_;
    bool preview_pending_ = false;
    BatteryProvider battery_provider_;

    // LVGL objects and animation state (LVGL task only).
    uint16_t* buffer_ = nullptr;
    lv_obj_t* canvas_ = nullptr;
    lv_obj_t* caption_box_ = nullptr;
    lv_obj_t* caption_label_ = nullptr;
    lv_obj_t* battery_text_ = nullptr;
    lv_obj_t* preview_box_ = nullptr;
    lv_obj_t* preview_obj_ = nullptr;
    lv_obj_t* splash_ = nullptr;
    lv_timer_t* frame_timer_ = nullptr;
    std::unique_ptr<LvglImage> preview_cached_;

    int cap_top_ = 0;
    int cap_h_ = 0;
    int face_y0_ = 0;
    int face_y1_ = 0;
    float cx_ = 0, cy_ = 0, eye_w_ = 0, eye_h_ = 0, gap_ = 0;
    float cur_[4] = {0, 0, 1, 1};  // dx, dy, sw, sh
    float tgt_[4] = {0, 0, 1, 1};
    double blink_at_ = 0;
    double blink_t_ = -1;
    double look_at_ = 0;
    double splash_until_ = 0;
    double preview_until_ = 0;
    double caption_drawn_at_ = 0;
    double wifi_at_ = -100;
    int wifi_bars_ = -1;
    int last_frame_key_ = -1;
    int status_key_ = -1;
    int battery_level_ = 0;
    int battery_percent_ = -1;
    std::string caption_shown_;
    CaptionColor caption_shown_color_ = kCaptionUser;

    static void FrameTimerCallback(lv_timer_t* timer);
    void Frame();
    FaceState CurrentState(double now);
    void UpdateTargets(FaceState state, Emotion emotion, double now);
    bool Tween();
    void DrawFace(FaceState state, Emotion emotion, double now);
    void UpdateStatus(double now);
    void UpdateCaption(double now);
    bool UpdatePreview(double now);
    void UpdateSplash(double now);

    void ClearRows(int y0, int y1);
    void FillSpan(int y, float x0, float x1, uint16_t color);
    void FillRect(int x0, int y0, int x1, int y1, uint16_t color);
    void Circle(float cx, float cy, float r, uint16_t color);
    void RoundedRect(float cx, float cy, float w, float h, uint16_t color, Emotion cut_emotion,
                     bool left_eye);
    int CutSpans(Emotion emotion, bool left, float top, float h, int y, float x0, float x1,
                 Span out[2]) const;
    void DrawWifiIcon(int x, int y, int bars, uint16_t color);
    void DrawBatteryIcon(int x, int y, int percent, uint16_t color);
    void Invalidate(int y0, int y1);
};
