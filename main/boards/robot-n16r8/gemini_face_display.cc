#include "gemini_face_display.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>

#include "application.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "wifi_manager.h"

#define TAG "GeminiFace"

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

float Uniform(float lo, float hi) {
    return lo + (hi - lo) * (esp_random() / static_cast<float>(UINT32_MAX));
}

bool MatchesAny(const char* value, std::initializer_list<const char*> choices) {
    for (const char* choice : choices) {
        if (std::strcmp(value, choice) == 0) {
            return true;
        }
    }
    return false;
}

// HSV -> RGB for the rainbow "Peker" letters (colorsys.hsv_to_rgb).
Rgb Hsv(float h, float s, float v) {
    h = h - std::floor(h);
    const int i = static_cast<int>(h * 6.0f);
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
    return {static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255),
            static_cast<uint8_t>(b * 255)};
}

}  // namespace

GeminiFaceDisplay::GeminiFaceDisplay(esp_lcd_panel_io_handle_t panel_io,
                                     esp_lcd_panel_handle_t panel, int width, int height,
                                     int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                                     bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                    swap_xy) {}

GeminiFaceDisplay::~GeminiFaceDisplay() {
    if (frame_timer_ != nullptr) {
        DisplayLockGuard lock(this);
        lv_timer_delete(frame_timer_);
    }
    if (buffer_ != nullptr) {
        heap_caps_free(buffer_);
    }
}

void GeminiFaceDisplay::SetBatteryProvider(BatteryProvider provider) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    battery_provider_ = std::move(provider);
}

void GeminiFaceDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI called more than once");
        return;
    }
    SpiLcdDisplay::SetupUI();

    DisplayLockGuard lock(this);
    auto* screen = lv_screen_active();
    // The stock xiaozhi widgets stay alive for the base class but are never shown.
    for (uint32_t i = 0; i < lv_obj_get_child_count(screen); ++i) {
        lv_obj_add_flag(lv_obj_get_child(screen, i), LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    const int w = width_, h = height_;
    buffer_ = static_cast<uint16_t*>(
        heap_caps_calloc(w * h, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buffer_ == nullptr) {
        ESP_LOGE(TAG, "No memory for the face canvas");
        return;
    }
    canvas_ = lv_canvas_create(screen);
    lv_canvas_set_buffer(canvas_, buffer_, w, h, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(canvas_, 0, 0);

    // Captions: bottom strip, last lines visible, user gray / robot white.
    auto* theme = static_cast<LvglTheme*>(current_theme_);
    const lv_font_t* caption_font = (theme != nullptr && theme->text_font() != nullptr)
                                        ? theme->text_font()->font()
                                        : &lv_font_montserrat_14;
    const int line_h = lv_font_get_line_height(caption_font) + 3;
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
    lv_obj_set_style_text_font(caption_label_, caption_font, 0);
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
    // Below the status bar down to the bottom edge (over the captions for the 3 s it shows),
    // so a 4:3 photo can use the full screen width.
    lv_obj_set_size(preview_box_, w, h - face_y0_);
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
    lv_obj_align(word, LV_ALIGN_TOP_MID, 0, static_cast<int>(h * 0.47f) - word_h * 4 / 5);
    lv_obj_t* line = lv_obj_create(splash_);
    lv_obj_remove_style_all(line);
    lv_obj_set_size(line, static_cast<int>(w * 0.6f), 2);
    lv_obj_set_style_bg_color(line, ToLv(kAccent), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_pos(line, static_cast<int>(w * 0.2f), static_cast<int>(h * 0.54f));
    lv_obj_t* sub = lv_label_create(splash_);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(sub, ToLv(kRobot), 0);
    lv_label_set_text(sub, "AI Technology");
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0,
                 static_cast<int>(h * 0.63f) - lv_font_get_line_height(&lv_font_montserrat_20));

    const double now = Now();
    splash_until_ = now + kSplashS;
    blink_at_ = now + 3;
    look_at_ = now + 2;
    frame_timer_ = lv_timer_create(FrameTimerCallback, kFramePeriodMs, this);
}

void GeminiFaceDisplay::SetStatus(const char* status) {
    // The face shows the state itself; keep the base bookkeeping only.
    LvglDisplay::SetStatus(status);
}

void GeminiFaceDisplay::ShowNotification(const char* notification, int duration_ms) {
    if (notification == nullptr || notification[0] == '\0') {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    caption_text_ = notification;
    caption_color_ = kCaptionWarn;
    caption_sticky_ = false;
    caption_at_ = Now();
    caption_dirty_ = true;
}

void GeminiFaceDisplay::ShowNotification(const std::string& notification, int duration_ms) {
    ShowNotification(notification.c_str(), duration_ms);
}

void GeminiFaceDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double now = Now();
    if (std::strcmp(emotion, "thinking") == 0) {
        thinking_since_ = now;
        return;
    }
    Emotion mapped = kNeutral;
    if (MatchesAny(emotion, {"happy", "laughing", "funny", "loving", "kissy", "confident",
                             "delicious", "cool", "winking", "relaxed"})) {
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

void GeminiFaceDisplay::SetChatMessage(const char* role, const char* content) {
    if (role == nullptr || content == nullptr || content[0] == '\0') {
        return;
    }
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
    const DeviceState state = Application::GetInstance().GetDeviceState();
    const bool upgrading = state == kDeviceStateUpgrading;
    caption_sticky_ = std::strcmp(role, "system") == 0 &&
                      (state == kDeviceStateWifiConfiguring || state == kDeviceStateActivating ||
                       upgrading);
    caption_text_ = upgrading ? std::string("Updating...\n") + content : std::string(content);
    caption_at_ = now;
    caption_dirty_ = true;
}

void GeminiFaceDisplay::ClearChatMessages() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    caption_text_.clear();
    caption_sticky_ = false;
    caption_dirty_ = true;
}

void GeminiFaceDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_preview_ = std::move(image);
    preview_pending_ = true;
    flash_until_ = Now() + 0.35;
}

void GeminiFaceDisplay::SetTheme(Theme* theme) {
    LcdDisplay::SetTheme(theme);
    if (!setup_ui_called_ || caption_label_ == nullptr) {
        return;
    }
    // Assets can replace the font after SetupUI; rebind it so no dangling font is kept.
    DisplayLockGuard lock(this);
    auto* lvgl_theme = static_cast<LvglTheme*>(theme);
    if (lvgl_theme != nullptr && lvgl_theme->text_font() != nullptr) {
        lv_obj_set_style_text_font(caption_label_, lvgl_theme->text_font()->font(), 0);
    }
}

void GeminiFaceDisplay::FrameTimerCallback(lv_timer_t* timer) {
    static_cast<GeminiFaceDisplay*>(lv_timer_get_user_data(timer))->Frame();
}

void GeminiFaceDisplay::Frame() {
    if (buffer_ == nullptr) {
        return;
    }
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
        if (now > emotion_until_) {
            emotion_ = kNeutral;
        }
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

GeminiFaceDisplay::FaceState GeminiFaceDisplay::CurrentState(double now) {
    const DeviceState device = Application::GetInstance().GetDeviceState();
    if (device == kDeviceStateStarting || device == kDeviceStateWifiConfiguring ||
        device == kDeviceStateActivating || device == kDeviceStateFatalError ||
        !WifiManager::GetInstance().IsConnected()) {
        return kOffline;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (now < flash_until_) {
        return kLooking;
    }
    if (device == kDeviceStateSpeaking) {
        thinking_since_ = -1;
        // The device is "speaking" from the server's tts start, also while the model is still
        // working (tool calls, photo): move the mouth only while audio is really playing.
        if (!Application::GetInstance().GetAudioService().IsPlaybackIdle()) {
            voice_until_ = now + 0.5;
        }
        return now < voice_until_ ? kSpeaking : kThinking;
    }
    // The answer did not come in time: stop "thinking" (same as thinking_timeout_s).
    if (thinking_since_ >= 0 && now - thinking_since_ > kThinkingTimeoutS) {
        thinking_since_ = -1;
    }
    if (thinking_since_ >= 0 || device == kDeviceStateConnecting ||
        device == kDeviceStateUpgrading) {
        return kThinking;
    }
    if (device == kDeviceStateListening) {
        return kListening;
    }
    return kIdle;
}

void GeminiFaceDisplay::UpdateTargets(FaceState state, Emotion emotion, double now) {
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
    float sh = emotion == kSurprised ? 1.25f : (emotion == kSleepy ? 0.35f : 1.0f);
    tgt_[3] = big * sh * (state == kOffline ? 0.12f : 1.0f);
    // Blink: closes and opens in 0.18 s.
    if (blink_t_ < 0 && now > blink_at_ && (state == kIdle || state == kListening)) {
        blink_t_ = now;
    }
    if (blink_t_ >= 0 && now - blink_t_ > 0.18) {
        blink_t_ = -1;
        blink_at_ = now + Uniform(2.5f, 6.0f);
    }
}

bool GeminiFaceDisplay::Tween() {
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

void GeminiFaceDisplay::DrawFace(FaceState state, Emotion emotion, double now) {
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
        const float p = static_cast<float>((now - blink_t_) / 0.18);
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
    const uint16_t accent = ToColor(kAccent);
    if (state == kSpeaking) {
        static const int kMouthHeights[] = {4, 8, 12, 16, 20};
        RoundedRect(cx_, mouth_y, 56, kMouthHeights[esp_random() % 5], accent, kNeutral, true);
    } else if (state == kThinking) {
        // Green eyes, still line mouth at the usual place (it does not follow the glance).
        RoundedRect(cx_, mouth_y, 40, 6, ToColor(kListen), kNeutral, true);
    } else if (state == kListening) {
        Circle(cx_, mouth_y, 4 + 3 * std::fabs(std::sin(now * 4)), ToColor(kListen));
    }
}

void GeminiFaceDisplay::UpdateStatus(double now) {
    auto& wifi = WifiManager::GetInstance();
    if (now - wifi_at_ > 5) {
        wifi_at_ = now;
        if (!wifi.IsConnected()) {
            wifi_bars_ = -1;
        } else {
            const int dbm = wifi.GetRssi();
            wifi_bars_ = dbm >= -60 ? 3 : dbm >= -70 ? 2 : dbm >= -80 ? 1 : 0;
        }
    }
    const DeviceState device = Application::GetInstance().GetDeviceState();
    const bool online = wifi_bars_ >= 0 && device != kDeviceStateStarting &&
                        device != kDeviceStateConnecting && device != kDeviceStateActivating &&
                        device != kDeviceStateWifiConfiguring && device != kDeviceStateFatalError;

    int percent = -1, level = 0;
    bool has_battery = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (battery_provider_) {
            has_battery = true;
            if (!battery_provider_(percent, level)) {
                percent = -1;
                level = 0;
            }
        }
    }
    battery_level_ = std::clamp(level, 0, 3);
    battery_percent_ = percent;

    const int key = (online ? 1 : 0) + (wifi_bars_ + 1) * 2 + (has_battery ? 1 : 0) * 10 +
                    (percent + 1) * 20 + battery_level_ * 4000;
    if (key == status_key_) {
        return;
    }
    status_key_ = key;
    ClearRows(kStatusY, kStatusY + kBatteryH - 1);
    // Green: connected to the server; yellow: Wi-Fi but no server; red cross: no Wi-Fi.
    DrawWifiIcon(kWifiLeft, kStatusY, wifi_bars_,
                 ToColor(wifi_bars_ < 0 ? kBad : (online ? kOk : kWarn)));
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

void GeminiFaceDisplay::UpdateCaption(double now) {
    std::string text;
    CaptionColor color;
    bool sticky;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        text = caption_text_;
        color = caption_color_;
        sticky = caption_sticky_;
        if (!sticky && !text.empty() && now - caption_at_ > kCaptionHoldS) {
            caption_text_.clear();
            text.clear();
        }
        caption_dirty_ = false;
    }
    // Draw at most every 0.15 s while text streams in.
    if ((text == caption_shown_ && color == caption_shown_color_) ||
        now - caption_drawn_at_ < 0.15) {
        return;
    }
    caption_shown_ = text;
    caption_shown_color_ = color;
    caption_drawn_at_ = now;

    // Setup instructions (Wi-Fi portal, activation code) may use the whole face area.
    const int box_top = sticky ? face_y0_ : cap_top_;
    lv_obj_set_pos(caption_box_, 0, box_top);
    lv_obj_set_size(caption_box_, width_, cap_top_ + cap_h_ - box_top);
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
    if (sticky) {
        lv_obj_move_foreground(caption_box_);
    }
}

bool GeminiFaceDisplay::UpdatePreview(double now) {
    std::unique_ptr<LvglImage> image;
    bool pending = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (preview_pending_) {
            pending = true;
            image = std::move(pending_preview_);
            preview_pending_ = false;
        }
    }
    if (pending) {
        if (image == nullptr) {
            preview_until_ = 0;
        } else {
            preview_cached_ = std::move(image);
            const auto* dsc = preview_cached_->image_dsc();
            lv_image_set_src(preview_obj_, dsc);
            if (dsc->header.w > 0 && dsc->header.h > 0) {
                // Fit inside the face area keeping the aspect ratio (never enlarge).
                const int box_h = height_ - face_y0_;
                const int scale = std::min({256, 256 * width_ / static_cast<int>(dsc->header.w),
                                            256 * box_h / static_cast<int>(dsc->header.h)});
                lv_image_set_scale(preview_obj_, scale);
            }
            lv_obj_center(preview_obj_);
            lv_obj_remove_flag(preview_box_, LV_OBJ_FLAG_HIDDEN);
            preview_until_ = now + kPreviewS;
        }
    }
    if (now < preview_until_) {
        return true;
    }
    if (!lv_obj_has_flag(preview_box_, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_add_flag(preview_box_, LV_OBJ_FLAG_HIDDEN);
        lv_image_set_src(preview_obj_, nullptr);
        preview_cached_.reset();
    }
    return false;
}

void GeminiFaceDisplay::UpdateSplash(double now) {
    if (splash_ == nullptr || lv_obj_has_flag(splash_, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }
    const DeviceState device = Application::GetInstance().GetDeviceState();
    // Stay up while booting, but get out of the way of the setup instructions.
    const bool setup = device == kDeviceStateWifiConfiguring || device == kDeviceStateActivating;
    if (setup || (now >= splash_until_ && device != kDeviceStateStarting)) {
        lv_obj_add_flag(splash_, LV_OBJ_FLAG_HIDDEN);
    }
}

void GeminiFaceDisplay::ClearRows(int y0, int y1) {
    y0 = std::max(0, y0);
    y1 = std::min(height_ - 1, y1);
    if (y1 >= y0) {
        std::memset(buffer_ + y0 * width_, 0, (y1 - y0 + 1) * width_ * sizeof(uint16_t));
    }
}

void GeminiFaceDisplay::FillSpan(int y, float x0, float x1, uint16_t color) {
    const int a = std::max(0, static_cast<int>(x0));
    const int b = std::min(width_, static_cast<int>(x1));
    if (y >= 0 && y < height_ && b > a) {
        std::fill(buffer_ + y * width_ + a, buffer_ + y * width_ + b, color);
    }
}

void GeminiFaceDisplay::FillRect(int x0, int y0, int x1, int y1, uint16_t color) {
    // Inclusive corners, like PIL rectangle.
    for (int y = y0; y <= y1; ++y) {
        FillSpan(y, x0, x1 + 1, color);
    }
}

void GeminiFaceDisplay::Circle(float cx, float cy, float r, uint16_t color) {
    for (int y = static_cast<int>(cy - r); y <= static_cast<int>(cy + r); ++y) {
        const float d = r * r - (y - cy) * (y - cy);
        if (d > 0) {
            const float half = std::sqrt(d);
            FillSpan(y, cx - half, cx + half, color);
        }
    }
}

void GeminiFaceDisplay::RoundedRect(float cx, float cy, float w, float h, uint16_t color,
                                    Emotion cut_emotion, bool left_eye) {
    const int top = static_cast<int>(cy - h / 2);
    const int bottom = static_cast<int>(cy + h / 2);
    const float r = std::min(w, h) * 0.32f;
    for (int y = top; y < bottom; ++y) {
        const float d = std::min(y - top, bottom - 1 - y);
        const float inset = d < r ? r - std::sqrt(std::max(0.0f, r * r - (r - d) * (r - d))) : 0.0f;
        const float x0 = cx - w / 2 + inset, x1 = cx + w / 2 - inset;
        Span spans[2];
        const int n = CutSpans(cut_emotion, left_eye, cy - h / 2, h, y, x0, x1, spans);
        for (int i = 0; i < n; ++i) {
            FillSpan(y, spans[i].x0, spans[i].x1, color);
        }
    }
}

int GeminiFaceDisplay::CutSpans(Emotion emotion, bool left, float top, float h, int y, float x0,
                                float x1, Span out[2]) const {
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

void GeminiFaceDisplay::DrawWifiIcon(int x, int y, int bars, uint16_t color) {
    // Three arcs and a dot; unlit arcs gray; a cross when there is no Wi-Fi.
    const uint16_t gray = ToColor(kIconGray);
    const int cx = x + kWifiW / 2, cy = y + kBatteryH - 4;
    const int radii[] = {6, 11, 16};
    for (int py = y; py < y + kBatteryH; ++py) {
        for (int px = x; px < x + kWifiW; ++px) {
            const int dx = px - cx, dy = py - cy;
            const float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            // 225..315 degrees in PIL terms: the upward quarter.
            if (dy >= 0 || std::abs(dx) > -dy) {
                continue;
            }
            for (int i = 0; i < 3; ++i) {
                if (dist <= radii[i] + 0.5f && dist > radii[i] - 2.0f) {
                    const bool lit = bars >= 0 && i < bars;
                    buffer_[py * width_ + px] = lit ? color : gray;
                }
            }
        }
    }
    Circle(cx + 0.5f, cy + 0.5f, 2.5f, bars >= 0 ? color : gray);
    if (bars < 0) {
        // Cross: (cx-8, 3)-(cx+8, 19) and the mirror, 2 px wide.
        for (int t = 0; t <= 16; ++t) {
            const int py = y + 3 + t;
            for (int k = 0; k < 2; ++k) {
                FillSpan(py, cx - 8 + t + k, cx - 8 + t + k + 1, color);
                FillSpan(py, cx + 8 - t - k, cx + 8 - t - k + 1, color);
            }
        }
    }
}

void GeminiFaceDisplay::DrawBatteryIcon(int x, int y, int percent, uint16_t color) {
    const uint16_t frame = ToColor(kBatteryFrame);
    const int x0 = x + kBatteryW - 33, y0 = y + 4, x1 = x + kBatteryW - 4, y1 = y + 16;
    FillRect(x0, y0, x1, y0, frame);
    FillRect(x0, y1, x1, y1, frame);
    FillRect(x0, y0, x0, y1, frame);
    FillRect(x1, y0, x1, y1, frame);
    FillRect(x1 + 1, y0 + 3, x1 + 3, y1 - 3, frame);
    if (percent > 0) {
        FillRect(x0 + 2, y0 + 2, x0 + 2 + std::max(1, (x1 - x0 - 4) * percent / 100), y1 - 2,
                 color);
    }
}

void GeminiFaceDisplay::Invalidate(int y0, int y1) {
    lv_area_t area = {0, std::max(0, y0), width_ - 1, std::min(height_ - 1, y1)};
    lv_obj_invalidate_area(canvas_, &area);
}
