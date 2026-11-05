#include "face.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <mutex>

#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <jpeg_decoder.h>
#include <libs/tiny_ttf/lv_tiny_ttf.h>

#include "audio.h"
#include "pins.h"
#include "power.h"
#include "status.h"

#define TAG "face"

extern const uint8_t kTtfStart[] asm("_binary_Montserrat_Medium_ttf_start");
extern const uint8_t kTtfEnd[] asm("_binary_Montserrat_Medium_ttf_end");

namespace {

struct Rgb {
    uint8_t r, g, b;
};

// Same palette as NeuroVision-Gemini COLORS.
constexpr Rgb kEye{40, 210, 255};
constexpr Rgb kListen{60, 255, 210};
constexpr Rgb kOfflineGray{90, 90, 100};
constexpr Rgb kHot{255, 150, 30};
constexpr Rgb kFlash{255, 255, 255};
constexpr Rgb kAccent{40, 210, 255};
constexpr Rgb kOk{40, 220, 90};
constexpr Rgb kWarn{255, 200, 0};
constexpr Rgb kBad{255, 50, 40};
constexpr Rgb kUser{150, 150, 165};
constexpr Rgb kRobot{225, 240, 255};
constexpr Rgb kIconGray{70, 70, 70};
constexpr Rgb kBatteryFrame{200, 200, 200};
// Eye color by battery level: blue, yellow (<35%), orange (<20%), red (<10%).
constexpr Rgb kBatteryEyes[] = {kEye, kWarn, kHot, kBad};

constexpr int kBatteryW = 84;
constexpr int kBatteryH = 21;
constexpr int kBatteryRight = 9;
constexpr int kWifiW = 32;
constexpr int kWifiLeft = 9;
constexpr int kStatusY = 8;

constexpr int kCaptionLines = 3;
constexpr double kCaptionHoldS = 8.0;
constexpr double kThinkingTimeoutS = 4.0;
constexpr double kSplashS = 3.0;
constexpr double kPreviewS = 3.0;
constexpr double kEmotionS = 5.0;
constexpr uint32_t kFramePeriodMs = 50;

uint16_t ToColor(const Rgb& c) { return lv_color_to_u16(lv_color_make(c.r, c.g, c.b)); }
lv_color_t ToLv(const Rgb& c) { return lv_color_make(c.r, c.g, c.b); }
double Now() { return esp_timer_get_time() / 1e6; }
float Uniform(float lo, float hi) { return lo + (hi - lo) * (esp_random() / (float)UINT32_MAX); }

bool MatchesAny(const char* value, std::initializer_list<const char*> choices) {
    for (const char* c : choices) {
        if (std::strcmp(value, c) == 0) return true;
    }
    return false;
}

// HSV -> RGB for the rainbow "Peker" letters (colorsys.hsv_to_rgb).
Rgb Hsv(float h, float s, float v) {
    h = h - std::floor(h);
    const int i = (int)(h * 6.0f);
    const float f = h * 6.0f - i;
    const float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
    float r = v, g = t, b = p;
    switch (i % 6) {
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
        default: break;
    }
    return {(uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255)};
}

struct Photo {
    uint16_t* pixels = nullptr;
    lv_image_dsc_t dsc = {};
    ~Photo() { heap_caps_free(pixels); }
};

class Face {
public:
    enum FaceState { kOffline, kLooking, kSpeaking, kThinking, kListening, kIdle };
    enum Emotion { kNeutral, kHappy, kSad, kSurprised, kAngry, kSleepy };
    enum CaptionColor { kCaptionUser, kCaptionRobot, kCaptionWarn };
    struct Span {
        float x0, x1;
    };

    void Setup();
    void SetEmotion(const char* emotion);
    void SetCaption(const char* role, const std::string& text);
    void SetPhoto(std::unique_ptr<Photo> photo);

private:
    // Shared with the callers' tasks (guarded by state_mutex_).
    std::mutex state_mutex_;
    Emotion emotion_ = kNeutral;
    double emotion_until_ = 0;
    double thinking_since_ = -1;
    double flash_until_ = 0;
    double voice_until_ = 0;
    std::string caption_text_;
    CaptionColor caption_color_ = kCaptionUser;
    double caption_at_ = 0;
    std::unique_ptr<Photo> pending_photo_;
    bool photo_pending_ = false;

    // LVGL objects and animation state (LVGL task only).
    const int width_ = LCD_WIDTH, height_ = LCD_HEIGHT;
    uint16_t* buffer_ = nullptr;
    lv_obj_t* canvas_ = nullptr;
    lv_obj_t* caption_box_ = nullptr;
    lv_obj_t* caption_label_ = nullptr;
    lv_obj_t* battery_text_ = nullptr;
    lv_obj_t* preview_box_ = nullptr;
    lv_obj_t* preview_obj_ = nullptr;
    lv_obj_t* splash_ = nullptr;
    std::unique_ptr<Photo> photo_shown_;

    int cap_top_ = 0, cap_h_ = 0, face_y0_ = 0, face_y1_ = 0;
    float cx_ = 0, cy_ = 0, eye_w_ = 0, eye_h_ = 0, gap_ = 0;
    float cur_[4] = {0, 0, 1, 1};  // dx, dy, sw, sh
    float tgt_[4] = {0, 0, 1, 1};
    double blink_at_ = 0, blink_t_ = -1, look_at_ = 0;
    double splash_until_ = 0, preview_until_ = 0, caption_drawn_at_ = 0, wifi_at_ = -100;
    int wifi_bars_ = -1, last_frame_key_ = -1, status_key_ = -1;
    int battery_level_ = 0, battery_percent_ = -1;
    std::string caption_shown_;
    CaptionColor caption_shown_color_ = kCaptionUser;

    static void FrameTimer(lv_timer_t* t) { static_cast<Face*>(lv_timer_get_user_data(t))->Frame(); }
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
    void RoundedRect(float cx, float cy, float w, float h, uint16_t color, Emotion cut, bool left);
    int CutSpans(Emotion emotion, bool left, float top, float h, int y, float x0, float x1, Span out[2]) const;
    void DrawWifiIcon(int x, int y, int bars, uint16_t color);
    void DrawBatteryIcon(int x, int y, int percent, uint16_t color);
    void Invalidate(int y0, int y1);
};

Face g_face;

void Face::Setup() {
    auto* screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    const int w = width_, h = height_;
    buffer_ = (uint16_t*)heap_caps_calloc(w * h, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buffer_ == nullptr) {
        ESP_LOGE(TAG, "no memory for the face canvas");
        return;
    }
    canvas_ = lv_canvas_create(screen);
    lv_canvas_set_buffer(canvas_, buffer_, w, h, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(canvas_, 0, 0);

    // Captions: bottom strip, last lines visible, user gray / robot white. Drawn from the
    // embedded Montserrat TTF (the built-in bitmap font has no ç ğ ı İ ö ş ü).
    const lv_font_t* font = lv_tiny_ttf_create_data(kTtfStart, kTtfEnd - kTtfStart, 14);
    if (font == nullptr) font = &lv_font_montserrat_14;
    const int line_h = lv_font_get_line_height(font) + 3;
    cap_h_ = kCaptionLines * line_h;
    cap_top_ = h - cap_h_ - 4;
    face_y0_ = 34;
    face_y1_ = cap_top_ - 1;
    cx_ = w / 2.0f;
    cy_ = h * 0.40f;
    eye_w_ = w * 0.25f;
    eye_h_ = h * 0.27f;
    gap_ = w * 0.15f;

    caption_box_ = lv_obj_create(screen);
    lv_obj_remove_style_all(caption_box_);
    lv_obj_set_pos(caption_box_, 0, cap_top_);
    lv_obj_set_size(caption_box_, w, cap_h_);
    lv_obj_set_style_bg_color(caption_box_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(caption_box_, LV_OPA_COVER, 0);
    lv_obj_clear_flag(caption_box_, LV_OBJ_FLAG_SCROLLABLE);
    caption_label_ = lv_label_create(caption_box_);
    lv_obj_set_width(caption_label_, w - 12);
    lv_label_set_long_mode(caption_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(caption_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(caption_label_, font, 0);
    lv_obj_set_style_text_line_space(caption_label_, 3, 0);
    lv_obj_set_style_text_color(caption_label_, ToLv(kUser), 0);
    lv_label_set_text(caption_label_, "");
    lv_obj_align(caption_label_, LV_ALIGN_TOP_MID, 0, 2);

    // Battery percent text, left of the battery outline (drawn on the canvas).
    battery_text_ = lv_label_create(screen);
    lv_obj_set_style_text_font(battery_text_, &lv_font_montserrat_14, 0);
    lv_obj_set_width(battery_text_, kBatteryW - 33 - 4);
    lv_obj_set_style_text_align(battery_text_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(battery_text_, w - kBatteryW - kBatteryRight,
                   kStatusY + (kBatteryH - lv_font_get_line_height(&lv_font_montserrat_14)) / 2);
    lv_label_set_text(battery_text_, "");
    lv_obj_add_flag(battery_text_, LV_OBJ_FLAG_HIDDEN);

    // Camera frame from "look": shown in place of the face for a few seconds.
    preview_box_ = lv_obj_create(screen);
    lv_obj_remove_style_all(preview_box_);
    lv_obj_set_pos(preview_box_, 0, face_y0_);
    lv_obj_set_size(preview_box_, w, h - face_y0_);  // down to the bottom edge while shown
    lv_obj_set_style_bg_color(preview_box_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(preview_box_, LV_OPA_COVER, 0);
    lv_obj_clear_flag(preview_box_, LV_OBJ_FLAG_SCROLLABLE);
    preview_obj_ = lv_image_create(preview_box_);
    lv_obj_center(preview_obj_);
    lv_obj_add_flag(preview_box_, LV_OBJ_FLAG_HIDDEN);

    // Splash: rainbow "Peker", a line, "AI Technology".
    splash_ = lv_obj_create(screen);
    lv_obj_remove_style_all(splash_);
    lv_obj_set_size(splash_, w, h);
    lv_obj_set_style_bg_color(splash_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(splash_, LV_OPA_COVER, 0);
    lv_obj_t* word = lv_obj_create(splash_);
    lv_obj_remove_style_all(word);
    lv_obj_set_size(word, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(word, LV_FLEX_FLOW_ROW);
    const char* letters[] = {"P", "e", "k", "e", "r"};
    for (int i = 0; i < 5; ++i) {
        lv_obj_t* letter = lv_label_create(word);
        lv_obj_set_style_text_font(letter, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(letter, ToLv(Hsv(0.55f + i * 0.12f, 0.85f, 1.0f)), 0);
        lv_label_set_text(letter, letters[i]);
    }
    const int word_h = lv_font_get_line_height(&lv_font_montserrat_48);
    lv_obj_align(word, LV_ALIGN_TOP_MID, 0, (int)(h * 0.47f) - word_h * 4 / 5);
    lv_obj_t* line = lv_obj_create(splash_);
    lv_obj_remove_style_all(line);
    lv_obj_set_size(line, (int)(w * 0.6f), 2);
    lv_obj_set_style_bg_color(line, ToLv(kAccent), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_pos(line, (int)(w * 0.2f), (int)(h * 0.54f));
    lv_obj_t* sub = lv_label_create(splash_);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(sub, ToLv(kRobot), 0);
    lv_label_set_text(sub, "AI Technology");
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, (int)(h * 0.63f) - lv_font_get_line_height(&lv_font_montserrat_20));

    const double now = Now();
    splash_until_ = now + kSplashS;
    blink_at_ = now + 3;
    look_at_ = now + 2;
    lv_timer_create(FrameTimer, kFramePeriodMs, this);
}

void Face::SetEmotion(const char* emotion) {
    if (emotion == nullptr) return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double now = Now();
    if (std::strcmp(emotion, "thinking") == 0) {
        thinking_since_ = now;
        return;
    }
    Emotion mapped = kNeutral;
    if (MatchesAny(emotion, {"happy", "laughing", "funny", "loving", "confident", "cool", "winking", "relaxed"})) {
        mapped = kHappy;
    } else if (MatchesAny(emotion, {"sad", "crying"})) {
        mapped = kSad;
    } else if (MatchesAny(emotion, {"surprised", "shocked"})) {
        mapped = kSurprised;
    } else if (MatchesAny(emotion, {"angry", "disgusted"})) {
        mapped = kAngry;
    } else if (MatchesAny(emotion, {"sleepy"})) {
        mapped = kSleepy;
    }
    emotion_ = mapped;
    emotion_until_ = now + kEmotionS;
}

void Face::SetCaption(const char* role, const std::string& text) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double now = Now();
    if (std::strcmp(role, "user") == 0) {
        caption_color_ = kCaptionUser;
        thinking_since_ = now;  // waiting for the answer
    } else if (std::strcmp(role, "assistant") == 0) {
        caption_color_ = kCaptionRobot;
        thinking_since_ = -1;
    } else {
        caption_color_ = kCaptionWarn;
    }
    caption_text_ = text;
    caption_at_ = now;
}

void Face::SetPhoto(std::unique_ptr<Photo> photo) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_photo_ = std::move(photo);
    photo_pending_ = true;
    flash_until_ = Now() + 0.35;
}

void Face::Frame() {
    if (buffer_ == nullptr) return;
    const double now = Now();
    UpdateSplash(now);
    UpdateStatus(now);
    UpdateCaption(now);
    if (UpdatePreview(now)) {
        last_frame_key_ = -1;  // redraw the face from scratch afterwards
        return;
    }
    Emotion emotion;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (now > emotion_until_) emotion_ = kNeutral;
        emotion = emotion_;
    }
    const FaceState state = CurrentState(now);
    UpdateTargets(state, emotion, now);
    const bool moving = Tween();
    const bool animated = state == kSpeaking || state == kThinking || state == kListening ||
                          state == kLooking || blink_t_ >= 0;
    const int frame_key = state * 100 + emotion * 10 + battery_level_;
    if (moving || animated || frame_key != last_frame_key_) {
        DrawFace(state, emotion, now);
        Invalidate(face_y0_, face_y1_);
        last_frame_key_ = frame_key;
    }
}

Face::FaceState Face::CurrentState(double now) {
    const RobotState robot = status::Get();
    if (robot == RobotState::kStarting || robot == RobotState::kOffline || !status::WifiConnected()) {
        return kOffline;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (now < flash_until_) return kLooking;
    if (robot == RobotState::kSpeaking) {
        thinking_since_ = -1;
        // Move the mouth only while audio is really playing.
        if (audio::Playing()) voice_until_ = now + 0.5;
        return now < voice_until_ ? kSpeaking : kThinking;
    }
    if (thinking_since_ >= 0 && now - thinking_since_ > kThinkingTimeoutS && robot != RobotState::kThinking) {
        thinking_since_ = -1;
    }
    if (thinking_since_ >= 0 || robot == RobotState::kThinking || robot == RobotState::kUpdating) {
        return kThinking;
    }
    if (robot == RobotState::kListening) return kListening;
    return kIdle;
}

void Face::UpdateTargets(FaceState state, Emotion emotion, double now) {
    float& dx = tgt_[0];
    float& dy = tgt_[1];
    if (state == kIdle && now > look_at_) {
        dx = Uniform(-22, 22);
        dy = Uniform(-10, 10);
        look_at_ = now + Uniform(2.0f, 5.0f);
    } else if (state == kThinking) {
        dx = 12.0f;
        dy = -14.0f;
    } else if (state == kListening || state == kSpeaking) {
        dx = 0.0f;
        dy = 0.0f;
    }
    const float big = state == kListening ? 1.08f : (state == kLooking ? 1.15f : 1.0f);
    tgt_[2] = big * (emotion == kSurprised ? 1.12f : 1.0f);
    const float sh = emotion == kSurprised ? 1.25f : (emotion == kSleepy ? 0.35f : 1.0f);
    tgt_[3] = big * sh * (state == kOffline ? 0.12f : 1.0f);
    // Blink: closes and opens in 0.18 s.
    if (blink_t_ < 0 && now > blink_at_ && (state == kIdle || state == kListening)) blink_t_ = now;
    if (blink_t_ >= 0 && now - blink_t_ > 0.18) {
        blink_t_ = -1;
        blink_at_ = now + Uniform(2.5f, 6.0f);
    }
}

bool Face::Tween() {
    bool moving = false;
    for (int i = 0; i < 4; ++i) {
        const float d = tgt_[i] - cur_[i];
        if (std::fabs(d) > 0.01f) {
            cur_[i] += d * 0.35f;
            moving = true;
        } else {
            cur_[i] = tgt_[i];
        }
    }
    return moving;
}

void Face::DrawFace(FaceState state, Emotion emotion, double now) {
    ClearRows(face_y0_, face_y1_);
    Rgb eye = kBatteryEyes[battery_level_];
    if (state == kOffline) {
        eye = kOfflineGray;
    } else if (state == kLooking) {
        eye = kFlash;
    } else if (state == kListening || state == kThinking) {
        eye = kListen;
    }
    const uint16_t color = ToColor(eye);
    float open_k = 1.0f;
    if (blink_t_ >= 0) {
        const float p = (float)((now - blink_t_) / 0.18);
        open_k = std::max(0.08f, std::fabs(1 - 2 * p));
    }
    const float w = eye_w_ * cur_[2];
    const float h = eye_h_ * cur_[3] * open_k;
    const float cy = cy_ + cur_[1];
    for (bool left : {true, false}) {
        const float cx = cx_ + cur_[0] + (left ? -1 : 1) * (gap_ / 2 + eye_w_ / 2);
        RoundedRect(cx, cy, w, h, color, h > 10 ? emotion : kNeutral, left);
    }
    const float mouth_y = cy_ + eye_h_ * 0.5f + 34;
    if (state == kSpeaking) {
        static const int kMouthHeights[] = {4, 8, 12, 16, 20};
        RoundedRect(cx_, mouth_y, 56, kMouthHeights[esp_random() % 5], ToColor(kAccent), kNeutral, true);
    } else if (state == kThinking) {
        // Green eyes, still line mouth at the usual place (it does not follow the glance).
        RoundedRect(cx_, mouth_y, 40, 6, ToColor(kListen), kNeutral, true);
    } else if (state == kListening) {
        Circle(cx_, mouth_y, 4 + 3 * std::fabs(std::sin(now * 4)), ToColor(kListen));
    }
}

void Face::UpdateStatus(double now) {
    if (now - wifi_at_ > 5) {
        wifi_at_ = now;
        if (!status::WifiConnected()) {
            wifi_bars_ = -1;
        } else {
            const int dbm = status::WifiRssi();
            wifi_bars_ = dbm >= -60 ? 3 : dbm >= -70 ? 2 : dbm >= -80 ? 1 : 0;
        }
    }
    const RobotState robot = status::Get();
    const bool online = wifi_bars_ >= 0 && robot != RobotState::kStarting && robot != RobotState::kOffline;

    const power::Reading battery = power::Get();
    const bool has_battery = battery.ok;
    const int percent = battery.ok ? battery.percent : -1;
    battery_level_ = std::clamp(battery.level, 0, 3);
    battery_percent_ = percent;

    const int key = (online ? 1 : 0) + (wifi_bars_ + 1) * 2 + (has_battery ? 1 : 0) * 10 +
                    (percent + 1) * 20 + battery_level_ * 4000;
    if (key == status_key_) return;
    status_key_ = key;
    ClearRows(kStatusY, kStatusY + kBatteryH - 1);
    // Green: online (server reachable); yellow: Wi-Fi only; red cross: no Wi-Fi.
    DrawWifiIcon(kWifiLeft, kStatusY, wifi_bars_, ToColor(wifi_bars_ < 0 ? kBad : (online ? kOk : kWarn)));
    if (has_battery) {
        const Rgb c = percent < 0 ? kOfflineGray : (battery_level_ == 0 ? kOk : kBatteryEyes[battery_level_]);
        DrawBatteryIcon(width_ - kBatteryW - kBatteryRight, kStatusY, percent, ToColor(c));
        lv_label_set_text(battery_text_, percent < 0 ? "?" : ("%" + std::to_string(percent)).c_str());
        lv_obj_set_style_text_color(battery_text_, ToLv(c), 0);
        lv_obj_remove_flag(battery_text_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(battery_text_, LV_OBJ_FLAG_HIDDEN);
    }
    Invalidate(kStatusY, kStatusY + kBatteryH - 1);
}

void Face::UpdateCaption(double now) {
    std::string text;
    CaptionColor color;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!caption_text_.empty() && now - caption_at_ > kCaptionHoldS) caption_text_.clear();
        text = caption_text_;
        color = caption_color_;
    }
    // Draw at most every 0.15 s.
    if ((text == caption_shown_ && color == caption_shown_color_) || now - caption_drawn_at_ < 0.15) return;
    caption_shown_ = text;
    caption_shown_color_ = color;
    caption_drawn_at_ = now;
    const Rgb rgb = color == kCaptionUser ? kUser : (color == kCaptionRobot ? kRobot : kWarn);
    lv_obj_set_style_text_color(caption_label_, ToLv(rgb), 0);
    lv_label_set_text(caption_label_, text.c_str());
    lv_obj_update_layout(caption_label_);
    // If it does not fit, the last lines stay visible.
    if (lv_obj_get_height(caption_label_) > lv_obj_get_height(caption_box_)) {
        lv_obj_align(caption_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
    } else {
        lv_obj_align(caption_label_, LV_ALIGN_TOP_MID, 0, 2);
    }
}

bool Face::UpdatePreview(double now) {
    std::unique_ptr<Photo> photo;
    bool pending = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (photo_pending_) {
            pending = true;
            photo = std::move(pending_photo_);
            photo_pending_ = false;
        }
    }
    if (pending && photo) {
        photo_shown_ = std::move(photo);
        lv_image_set_src(preview_obj_, &photo_shown_->dsc);
        // Full screen width, right edge to left edge, keeping the photo's aspect ratio.
        const int pw = photo_shown_->dsc.header.w, ph = photo_shown_->dsc.header.h;
        const int box_h = height_ - face_y0_;
        const int draw_h = pw > 0 ? std::min(box_h, width_ * ph / pw) : box_h;
        lv_obj_set_size(preview_obj_, width_, draw_h);
        lv_image_set_inner_align(preview_obj_, LV_IMAGE_ALIGN_STRETCH);
        lv_obj_center(preview_obj_);
        lv_obj_remove_flag(preview_box_, LV_OBJ_FLAG_HIDDEN);
        preview_until_ = now + kPreviewS;
    }
    if (now < preview_until_) return true;
    if (!lv_obj_has_flag(preview_box_, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_add_flag(preview_box_, LV_OBJ_FLAG_HIDDEN);
        lv_image_set_src(preview_obj_, nullptr);
        photo_shown_.reset();
    }
    return false;
}

void Face::UpdateSplash(double now) {
    if (splash_ == nullptr || lv_obj_has_flag(splash_, LV_OBJ_FLAG_HIDDEN)) return;
    if (now >= splash_until_ && status::Get() != RobotState::kStarting) {
        lv_obj_add_flag(splash_, LV_OBJ_FLAG_HIDDEN);
    }
}

void Face::ClearRows(int y0, int y1) {
    y0 = std::max(0, y0);
    y1 = std::min(height_ - 1, y1);
    if (y1 >= y0) std::memset(buffer_ + y0 * width_, 0, (y1 - y0 + 1) * width_ * sizeof(uint16_t));
}

void Face::FillSpan(int y, float x0, float x1, uint16_t color) {
    const int a = std::max(0, (int)x0);
    const int b = std::min(width_, (int)x1);
    if (y >= 0 && y < height_ && b > a) std::fill(buffer_ + y * width_ + a, buffer_ + y * width_ + b, color);
}

void Face::FillRect(int x0, int y0, int x1, int y1, uint16_t color) {
    for (int y = y0; y <= y1; ++y) FillSpan(y, x0, x1 + 1, color);  // inclusive, like PIL
}

void Face::Circle(float cx, float cy, float r, uint16_t color) {
    for (int y = (int)(cy - r); y <= (int)(cy + r); ++y) {
        const float d = r * r - (y - cy) * (y - cy);
        if (d > 0) {
            const float half = std::sqrt(d);
            FillSpan(y, cx - half, cx + half, color);
        }
    }
}

void Face::RoundedRect(float cx, float cy, float w, float h, uint16_t color, Emotion cut, bool left) {
    const int top = (int)(cy - h / 2);
    const int bottom = (int)(cy + h / 2);
    const float r = std::min(w, h) * 0.32f;
    for (int y = top; y < bottom; ++y) {
        const float d = std::min(y - top, bottom - 1 - y);
        const float inset = d < r ? r - std::sqrt(std::max(0.0f, r * r - (r - d) * (r - d))) : 0.0f;
        const float x0 = cx - w / 2 + inset, x1 = cx + w / 2 - inset;
        Span spans[2];
        const int n = CutSpans(cut, left, cy - h / 2, h, y, x0, x1, spans);
        for (int i = 0; i < n; ++i) FillSpan(y, spans[i].x0, spans[i].x1, color);
    }
}

int Face::CutSpans(Emotion emotion, bool left, float top, float h, int y, float x0, float x1, Span out[2]) const {
    if (emotion == kSad || emotion == kAngry) {
        // Slanted upper lid: outer corner drops when sad, inner corner when angry.
        const bool outer_side = emotion == kSad ? left : !left;
        const float k = (y - top) / (h * 0.42f);
        if (k >= 1) {
            out[0] = {x0, x1};
            return 1;
        }
        const float drop = (1 - k) * (x1 - x0);
        out[0] = outer_side ? Span{x0 + drop, x1} : Span{x0, x1 - drop};
        return 1;
    }
    if (emotion == kHappy) {
        // Lower part erased as an arc: ^ ^
        const float cy = top + h * 1.05f, ry = h * 0.62f;
        const float dy = y - cy;
        if (std::fabs(dy) >= ry) {
            out[0] = {x0, x1};
            return 1;
        }
        const float half = (x1 - x0) * 0.62f * std::sqrt(1 - (dy / ry) * (dy / ry));
        const float mid = (x0 + x1) / 2;
        out[0] = {x0, mid - half};
        out[1] = {mid + half, x1};
        return 2;
    }
    out[0] = {x0, x1};
    return 1;
}

void Face::DrawWifiIcon(int x, int y, int bars, uint16_t color) {
    // Three arcs and a dot; unlit arcs gray; a cross when there is no Wi-Fi.
    const uint16_t gray = ToColor(kIconGray);
    const int cx = x + kWifiW / 2, cy = y + kBatteryH - 4;
    const int radii[] = {6, 11, 16};
    for (int py = y; py < y + kBatteryH; ++py) {
        for (int px = x; px < x + kWifiW; ++px) {
            const int dx = px - cx, dy = py - cy;
            const float dist = std::sqrt((float)(dx * dx + dy * dy));
            if (dy >= 0 || std::abs(dx) > -dy) continue;  // the upward quarter
            for (int i = 0; i < 3; ++i) {
                if (dist <= radii[i] + 0.5f && dist > radii[i] - 2.0f) {
                    buffer_[py * width_ + px] = (bars >= 0 && i < bars) ? color : gray;
                }
            }
        }
    }
    Circle(cx + 0.5f, cy + 0.5f, 2.5f, bars >= 0 ? color : gray);
    if (bars < 0) {
        for (int t = 0; t <= 16; ++t) {
            const int py = y + 3 + t;
            for (int k = 0; k < 2; ++k) {
                FillSpan(py, cx - 8 + t + k, cx - 8 + t + k + 1, color);
                FillSpan(py, cx + 8 - t - k, cx + 8 - t - k + 1, color);
            }
        }
    }
}

void Face::DrawBatteryIcon(int x, int y, int percent, uint16_t color) {
    const uint16_t frame = ToColor(kBatteryFrame);
    const int x0 = x + kBatteryW - 33, y0 = y + 4, x1 = x + kBatteryW - 4, y1 = y + 16;
    FillRect(x0, y0, x1, y0, frame);
    FillRect(x0, y1, x1, y1, frame);
    FillRect(x0, y0, x0, y1, frame);
    FillRect(x1, y0, x1, y1, frame);
    FillRect(x1 + 1, y0 + 3, x1 + 3, y1 - 3, frame);
    if (percent > 0) FillRect(x0 + 2, y0 + 2, x0 + 2 + std::max(1, (x1 - x0 - 4) * percent / 100), y1 - 2, color);
}

void Face::Invalidate(int y0, int y1) {
    lv_area_t area = {0, std::max(0, y0), width_ - 1, std::min(height_ - 1, y1)};
    lv_obj_invalidate_area(canvas_, &area);
}

void InitPanel(esp_lcd_panel_io_handle_t& io, esp_lcd_panel_handle_t& panel) {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LCD_MOSI;
    bus.miso_io_num = GPIO_NUM_NC;
    bus.sclk_io_num = PIN_LCD_SCK;
    bus.quadwp_io_num = GPIO_NUM_NC;
    bus.quadhd_io_num = GPIO_NUM_NC;
    bus.max_transfer_sz = LCD_WIDTH * 40 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {};
    io_cfg.cs_gpio_num = GPIO_NUM_NC;
    io_cfg.dc_gpio_num = PIN_LCD_DC;
    io_cfg.spi_mode = 3;
    io_cfg.pclk_hz = LCD_SPI_HZ;
    io_cfg.trans_queue_depth = 10;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t cfg = {};
    cfg.reset_gpio_num = PIN_LCD_RST;
    cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    cfg.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    vTaskDelay(pdMS_TO_TICKS(150));  // ST7789 needs ~120 ms after reset
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    // Gamma of the common 1.69" ST7789V2 module (the driver sends none: blacks looked grey).
    static const uint8_t kGammaPos[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
    static const uint8_t kGammaNeg[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
    esp_lcd_panel_io_tx_param(io, 0xE0, kGammaPos, sizeof(kGammaPos));
    esp_lcd_panel_io_tx_param(io, 0xE1, kGammaNeg, sizeof(kGammaNeg));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
}

}  // namespace

namespace face {

void Start() {
    esp_lcd_panel_io_handle_t io = nullptr;
    esp_lcd_panel_handle_t panel = nullptr;
    InitPanel(io, panel);

    lvgl_port_cfg_t port = ESP_LVGL_PORT_INIT_CONFIG();
    port.task_priority = 3;
    port.task_stack = 8192;  // TTF glyph rendering
    port.task_stack_caps = MALLOC_CAP_SPIRAM;  // keep internal RAM for DMA
    port.task_affinity = 0;
    ESP_ERROR_CHECK(lvgl_port_init(&port));

    lvgl_port_display_cfg_t disp = {};
    disp.io_handle = io;
    disp.panel_handle = panel;
    disp.buffer_size = LCD_WIDTH * 24;
    disp.double_buffer = true;
    disp.hres = LCD_WIDTH;
    disp.vres = LCD_HEIGHT;
    disp.color_format = LV_COLOR_FORMAT_RGB565;
    disp.flags.buff_dma = 1;
    disp.flags.swap_bytes = 1;
    if (lvgl_port_add_disp(&disp) == nullptr) {
        ESP_LOGE(TAG, "lvgl display failed");
        return;
    }
    if (lvgl_port_lock(0)) {
        g_face.Setup();
        lvgl_port_unlock();
    }
}

void SetEmotion(const char* emotion) { g_face.SetEmotion(emotion); }

void SetCaption(const char* role, const std::string& text) { g_face.SetCaption(role, text); }

// Camera JPEG -> RGB565 at the smallest decoder scale that is still at least the screen width
// (1024x768 -> 256x192), stretched to the full screen width (240x180).
void ShowPhoto(const std::vector<uint8_t>& jpeg) {
    auto photo = std::make_unique<Photo>();
    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata = const_cast<uint8_t*>(jpeg.data());
    cfg.indata_size = jpeg.size();
    cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
    esp_jpeg_image_output_t info = {};
    const esp_jpeg_image_scale_t kScales[] = {JPEG_IMAGE_SCALE_1_8, JPEG_IMAGE_SCALE_1_4, JPEG_IMAGE_SCALE_1_2,
                                              JPEG_IMAGE_SCALE_0};
    for (esp_jpeg_image_scale_t sc : kScales) {
        cfg.out_scale = sc;
        if (esp_jpeg_get_image_info(&cfg, &info) != ESP_OK) return;
        if ((int)info.width >= LCD_WIDTH) break;
    }
    if (info.output_len == 0) return;
    photo->pixels = (uint16_t*)heap_caps_malloc(info.output_len, MALLOC_CAP_SPIRAM);
    if (photo->pixels == nullptr) return;
    cfg.outbuf = (uint8_t*)photo->pixels;
    cfg.outbuf_size = info.output_len;
    if (esp_jpeg_decode(&cfg, &info) != ESP_OK) return;
    photo->dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    photo->dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    photo->dsc.header.w = info.width;
    photo->dsc.header.h = info.height;
    photo->dsc.header.stride = info.width * 2;
    photo->dsc.data = (const uint8_t*)photo->pixels;
    photo->dsc.data_size = info.output_len;
    g_face.SetPhoto(std::move(photo));
}

}  // namespace face
