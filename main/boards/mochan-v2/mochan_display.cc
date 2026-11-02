#include "mochan_display.h"

#include <esp_log.h>
#include <esp_random.h>

#include <algorithm>
#include <cstring>
#include <initializer_list>

#include "application.h"
#include "display/lvgl_display/lvgl_theme.h"

#define TAG "MochanDisplay"

namespace {

// Upper gazes are repeated in the weighted list so the robot does not keep looking down.
constexpr const char* kWeightedGazeEyes[] = {
    "nhin tren phai",
    "nhin tren trai",
    "nhin tren phai",
    "nhin tren trai",
    "nhin tren phai",
    "nhin tren trai",
    "nhin duoi phai",
    "nhin duoi trai",
};

// Idle expressions are deliberately biased toward friendly/curious faces.
constexpr const char* kWeightedIdleExpressions[] = {
    "happy_eyes",
    "happy_eyes",
    "happy_eyes",
    "suspicious_eyes",
    "suspicious_eyes",
    "eyes_shake",
    "sad_eyes",
    "angry_eyes",
};

bool MatchesAny(const char* value, std::initializer_list<const char*> choices) {
    if (value == nullptr) {
        return false;
    }
    for (const char* choice : choices) {
        if (std::strcmp(value, choice) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

MochanDisplay::MochanDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                             int width, int height, int offset_x, int offset_y, bool mirror_x,
                             bool mirror_y, bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                    swap_xy) {
    esp_timer_create_args_t idle_timer_args = {
        .callback = IdleEyeTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mochan_idle_eye",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&idle_timer_args, &idle_eye_timer_));
}

MochanDisplay::~MochanDisplay() {
    if (idle_eye_timer_ != nullptr) {
        esp_timer_stop(idle_eye_timer_);
        esp_timer_delete(idle_eye_timer_);
        idle_eye_timer_ = nullptr;
    }

    if (typewriter_timer_ != nullptr) {
        DisplayLockGuard lock(this);
        lv_timer_delete(typewriter_timer_);
        typewriter_timer_ = nullptr;
    }
    if (cursor_timer_ != nullptr) {
        DisplayLockGuard lock(this);
        lv_timer_delete(cursor_timer_);
        cursor_timer_ = nullptr;
    }
}

void MochanDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI called more than once");
        return;
    }

    SpiLcdDisplay::SetupUI();

    {
        DisplayLockGuard lock(this);
        auto* theme = static_cast<LvglTheme*>(current_theme_);
        auto screen = lv_screen_active();
        const lv_color_t black = lv_color_hex(0x000000);
        const lv_color_t text_color = lv_color_hex(0xEAFBFF);

        lv_obj_set_style_bg_color(screen, black, 0);
        lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(container_, black, 0);
        lv_obj_set_style_bg_opa(container_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_image_src(container_, nullptr, 0);

        lv_obj_set_size(emoji_box_, width_, kEyeHeight);
        lv_obj_align(emoji_box_, LV_ALIGN_TOP_MID, 0, kStatusBarHeight);
        lv_obj_set_style_bg_color(emoji_box_, black, 0);
        lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(emoji_box_, 0, 0);
        lv_obj_set_style_border_width(emoji_box_, 0, 0);
        lv_obj_set_scrollbar_mode(emoji_box_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_center(emoji_image_);

        // Terminal starts about 20 px below the eyes and leaves a small lower margin.
        lv_obj_set_size(bottom_bar_, width_, height_ - kStatusBarHeight - kEyeHeight - 10);
        lv_obj_align(bottom_bar_, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_set_style_bg_color(bottom_bar_, lv_color_hex(0x03090C), 0);
        lv_obj_set_style_bg_opa(bottom_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bottom_bar_, 1, 0);
        lv_obj_set_style_border_color(bottom_bar_, lv_color_hex(0x16414A), 0);
        lv_obj_set_style_border_opa(bottom_bar_, LV_OPA_60, 0);
        lv_obj_set_style_radius(bottom_bar_, 4, 0);
        lv_obj_set_style_pad_top(bottom_bar_, 14, 0);
        lv_obj_set_style_pad_bottom(bottom_bar_, 7, 0);
        lv_obj_set_style_pad_left(bottom_bar_, 10, 0);
        lv_obj_set_style_pad_right(bottom_bar_, 10, 0);
        lv_obj_set_scrollbar_mode(bottom_bar_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_width(chat_message_label_, width_ - 20);
        lv_obj_set_height(chat_message_label_, LV_SIZE_CONTENT);
        lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_text_color(chat_message_label_, text_color, 0);
        if (theme != nullptr && theme->text_font() != nullptr) {
            lv_obj_set_style_text_font(chat_message_label_, theme->text_font()->font(), 0);
        }
        lv_obj_align(chat_message_label_, LV_ALIGN_TOP_LEFT, 0, 0);

        camera_box_ = lv_obj_create(screen);
        lv_obj_set_size(camera_box_, width_, height_ - kStatusBarHeight - kEyeHeight);
        lv_obj_align(camera_box_, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(camera_box_, black, 0);
        lv_obj_set_style_bg_opa(camera_box_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(camera_box_, 0, 0);
        lv_obj_set_style_radius(camera_box_, 0, 0);
        lv_obj_set_style_pad_all(camera_box_, 0, 0);
        lv_obj_set_scrollbar_mode(camera_box_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(camera_box_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(camera_box_, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_parent(preview_image_, camera_box_);
        lv_obj_set_size(preview_image_, width_, height_);
        lv_obj_align(preview_image_, LV_ALIGN_CENTER, 0, 0);

        lv_obj_set_size(top_bar_, width_, kStatusBarHeight);
        lv_obj_align(top_bar_, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_color(top_bar_, black, 0);
        lv_obj_set_style_bg_opa(top_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_top(top_bar_, 0, 0);
        lv_obj_set_style_pad_bottom(top_bar_, 0, 0);
        lv_obj_set_style_pad_left(top_bar_, 6, 0);
        lv_obj_set_style_pad_right(top_bar_, 6, 0);
        lv_obj_remove_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_style_text_color(network_label_, lv_color_hex(0xEAFBFF), 0);
        lv_obj_add_flag(mute_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);

        lv_obj_t* right_icons = lv_obj_get_parent(mute_label_);
        state_dot_ = lv_obj_create(right_icons);
        lv_obj_set_size(state_dot_, 7, 7);
        lv_obj_set_style_radius(state_dot_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(state_dot_, 0, 0);
        lv_obj_set_style_pad_all(state_dot_, 0, 0);
        lv_obj_set_style_bg_opa(state_dot_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(state_dot_, lv_color_hex(0xF5B942), 0);
        lv_obj_clear_flag(state_dot_, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);

        typewriter_timer_ =
            lv_timer_create(TypewriterTimerCallback, kTypewriterPeriodMs, this);
        lv_timer_pause(typewriter_timer_);

        cursor_timer_ = lv_timer_create(CursorTimerCallback, kCursorBlinkPeriodMs, this);
        lv_timer_pause(cursor_timer_);

        loading_overlay_ = lv_obj_create(screen);
        lv_obj_set_size(loading_overlay_, width_, height_);
        lv_obj_align(loading_overlay_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(loading_overlay_, black, 0);
        lv_obj_set_style_bg_opa(loading_overlay_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(loading_overlay_, 0, 0);
        lv_obj_set_style_radius(loading_overlay_, 0, 0);
        lv_obj_set_style_pad_all(loading_overlay_, 0, 0);
        lv_obj_clear_flag(loading_overlay_, LV_OBJ_FLAG_SCROLLABLE);

        loading_spinner_ = lv_spinner_create(loading_overlay_);
        lv_obj_set_size(loading_spinner_, 54, 54);
        lv_obj_align(loading_spinner_, LV_ALIGN_CENTER, 0, -24);
        lv_spinner_set_anim_params(loading_spinner_, 850, 270);
        lv_obj_set_style_arc_width(loading_spinner_, 5, LV_PART_MAIN);
        lv_obj_set_style_arc_color(loading_spinner_, lv_color_hex(0x12343B), LV_PART_MAIN);
        lv_obj_set_style_arc_opa(loading_spinner_, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_arc_width(loading_spinner_, 5, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(loading_spinner_, lv_color_hex(0x27D7E8),
                                   LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(loading_spinner_, true, LV_PART_INDICATOR);
        lv_obj_clear_flag(loading_spinner_, LV_OBJ_FLAG_CLICKABLE);

        loading_label_ = lv_label_create(loading_overlay_);
        lv_label_set_text(loading_label_, "Wall-E Ai By Huy Vector\nLOADING...");
        lv_obj_set_width(loading_label_, width_ - 20);
        lv_obj_set_style_text_align(loading_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(loading_label_, text_color, 0);
        if (theme != nullptr && theme->text_font() != nullptr) {
            lv_obj_set_style_text_font(loading_label_, theme->text_font()->font(), 0);
        }
        lv_obj_align(loading_label_, LV_ALIGN_CENTER, 0, 37);
        lv_obj_move_foreground(loading_overlay_);
    }

    ScheduleIdleEye(2500);
}

void MochanDisplay::SetStatus(const char* status) {
    LvglDisplay::SetStatus(status);
    if (state_dot_ == nullptr) {
        return;
    }

    lv_color_t color = lv_color_hex(0xF5B942);
    switch (Application::GetInstance().GetDeviceState()) {
        case kDeviceStateIdle:
            color = lv_color_hex(0x35D05B);
            break;
        case kDeviceStateListening:
            color = lv_color_hex(0x258DFF);
            break;
        case kDeviceStateSpeaking:
            color = lv_color_hex(0x27D7E8);
            break;
        case kDeviceStateWifiConfiguring:
            color = lv_color_hex(0xFF5757);
            break;
        default:
            break;
    }

    DisplayLockGuard lock(this);
    lv_obj_set_style_bg_color(state_dot_, color, 0);
    const DeviceState state = Application::GetInstance().GetDeviceState();
    if (boot_loading_visible_ && state != kDeviceStateStarting &&
        state != kDeviceStateActivating) {
        HideBootLoadingLocked();
    }
}

const char* MochanDisplay::MapEmotionToEye(const char* emotion) const {
    if (MatchesAny(emotion, {"happy", "laughing", "funny", "loving", "kissy", "confident",
                             "delicious", "cool", "winking"})) {
        return "happy_eyes";
    }
    if (MatchesAny(emotion, {"sad", "crying", "sleepy"})) {
        return "sad_eyes";
    }
    if (MatchesAny(emotion, {"angry", "disgusted"})) {
        return "angry_eyes";
    }
    if (MatchesAny(emotion, {"thinking", "confused", "suspicious", "embarrassed"})) {
        return "suspicious_eyes";
    }
    if (MatchesAny(emotion, {"surprised", "shocked", "nervous", "silly"})) {
        return "eyes_shake";
    }
    return "eyes_nomal";
}

void MochanDisplay::SetEmotion(const char* emotion) {
    const char* eye = MapEmotionToEye(emotion);

    if (emotion == nullptr || MatchesAny(emotion, {"neutral", "robot_2", "standby"})) {
        const int normal_loops = (emotion != nullptr && std::strcmp(emotion, "robot_2") == 0)
                                     ? 1
                                     : 1 + esp_random() % 3;
        SetEyeIfChanged(eye, normal_loops, true);
        idle_accent_next_ = true;
        ScheduleIdleEye(normal_loops * 1920 + 120);
    } else {
        SetEyeIfChanged(eye);
        idle_accent_next_ = false;
        ScheduleIdleEye(5000);
    }
}

void MochanDisplay::ScheduleIdleEye(uint32_t delay_ms) {
    if (idle_eye_timer_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(idle_timer_mutex_);
    esp_timer_stop(idle_eye_timer_);
    const esp_err_t result = esp_timer_start_once(idle_eye_timer_, delay_ms * 1000ULL);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Could not schedule idle eye timer: %s", esp_err_to_name(result));
    }
}

void MochanDisplay::IdleEyeTimerCallback(void* arg) {
    static_cast<MochanDisplay*>(arg)->ShowRandomIdleEye();
}

void MochanDisplay::ShowRandomIdleEye() {
    if (!setup_ui_called_) {
        return;
    }

    if (!idle_accent_next_) {
        const int normal_loops = 1 + esp_random() % 3;
        SetEyeIfChanged("eyes_nomal", normal_loops, true);
        idle_accent_next_ = true;
        ScheduleIdleEye(normal_loops * 1920 + 120);
        return;
    }

    const bool show_expression = (esp_random() % 100) < 18;
    const char* accent = nullptr;
    if (show_expression) {
        constexpr size_t expression_count =
            sizeof(kWeightedIdleExpressions) / sizeof(kWeightedIdleExpressions[0]);
        for (int attempt = 0; attempt < 4; ++attempt) {
            accent = kWeightedIdleExpressions[esp_random() % expression_count];
            if (last_idle_accent_ != accent) {
                break;
            }
        }
    } else {
        constexpr size_t gaze_count =
            sizeof(kWeightedGazeEyes) / sizeof(kWeightedGazeEyes[0]);
        for (int attempt = 0; attempt < 6; ++attempt) {
            accent = kWeightedGazeEyes[esp_random() % gaze_count];
            if (last_idle_accent_ != accent) {
                break;
            }
        }
    }

    SetEyeIfChanged(accent, 1);
    last_idle_accent_ = accent;
    idle_accent_next_ = false;
    ScheduleIdleEye(show_expression ? 1750 : 2100);
}

void MochanDisplay::SetEyeIfChanged(const char* eye, int loop_count, bool restart_if_same) {
    if (eye == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(eye_mutex_);
    if (current_eye_ == eye) {
        if (restart_if_same && gif_controller_) {
            DisplayLockGuard display_lock(this);
            gif_controller_->Stop();
            if (loop_count > 0) {
                gif_controller_->SetLoopCount(loop_count);
            }
            gif_controller_->Start();
        }
        return;
    }
    LcdDisplay::SetEmotion(eye);
    if (loop_count > 0 && gif_controller_) {
        DisplayLockGuard display_lock(this);
        gif_controller_->SetLoopCount(loop_count);
    }
    current_eye_ = eye;
}

void MochanDisplay::SetChatMessage(const char* role, const char* content) {
    if (!setup_ui_called_ || chat_message_label_ == nullptr) {
        ESP_LOGW(TAG, "Chat message arrived before the Mochan UI was ready");
        return;
    }

    DisplayLockGuard lock(this);
    const char* safe_content = content != nullptr ? content : "";
    if (boot_message_pending_ && role != nullptr && std::strcmp(role, "system") == 0) {
        safe_content = "Wall-E Ai By Huy Vector";
        boot_message_pending_ = false;
    }

    const DeviceState device_state = Application::GetInstance().GetDeviceState();
    const bool is_system_popup_message =
        role != nullptr && std::strcmp(role, "system") == 0 && safe_content[0] != '\0' &&
        (device_state == kDeviceStateWifiConfiguring ||
         device_state == kDeviceStateActivating || device_state == kDeviceStateUpgrading);
    if (is_system_popup_message) {
        const bool was_hidden = lv_obj_has_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
        std::string setup_content = safe_content;
        if (device_state == kDeviceStateWifiConfiguring) {
            const size_t address_separator = setup_content.find(" Open: ");
            if (address_separator != std::string::npos) {
                setup_content.replace(address_separator, 1, "\n");
            }
        }
        if (device_state == kDeviceStateActivating && boot_loading_visible_) {
            HideBootLoadingLocked();
        }
        typewriter_target_.clear();
        typewriter_visible_ = setup_content;
        typewriter_position_ = typewriter_visible_.size();
        terminal_prompt_active_ = false;
        terminal_cursor_enabled_ = false;
        terminal_cursor_visible_ = false;
        terminal_has_content_ = true;
        lv_timer_pause(typewriter_timer_);
        lv_timer_pause(cursor_timer_);
        if (was_hidden) {
            ShowTerminalPopupLocked();
        }
        ShowTextLocked(setup_content.c_str());
        lv_obj_scroll_to_y(bottom_bar_, 0, LV_ANIM_OFF);
        return;
    }

    if (role != nullptr && std::strcmp(role, "assistant") == 0 && safe_content[0] != '\0') {
        const bool was_hidden = lv_obj_has_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
        if (terminal_has_content_ && terminal_prompt_active_ && !typewriter_target_.empty()) {
            typewriter_target_ += " ";
            typewriter_target_ += safe_content;
        } else {
            typewriter_target_ = safe_content;
            typewriter_visible_.clear();
            typewriter_position_ = 0;
        }
        terminal_prompt_active_ = true;
        terminal_cursor_enabled_ = true;
        terminal_cursor_visible_ = true;
        terminal_has_content_ = true;
        if (was_hidden) {
            ShowTerminalPopupLocked();
        }
        RenderTerminalTextLocked();
        lv_obj_scroll_to_y(bottom_bar_, 0, LV_ANIM_OFF);
        lv_timer_resume(typewriter_timer_);
        lv_timer_reset(typewriter_timer_);
        lv_timer_resume(cursor_timer_);
        lv_timer_reset(cursor_timer_);
        return;
    }

    typewriter_target_.clear();
    typewriter_visible_ = safe_content;
    typewriter_position_ = typewriter_visible_.size();
    terminal_prompt_active_ = false;
    terminal_cursor_enabled_ = false;
    terminal_cursor_visible_ = false;
    lv_timer_pause(typewriter_timer_);
    lv_timer_pause(cursor_timer_);
    terminal_has_content_ = false;
    ShowTextLocked("");
    HideTerminalLocked();
}

void MochanDisplay::TypewriterTimerCallback(lv_timer_t* timer) {
    auto* display = static_cast<MochanDisplay*>(lv_timer_get_user_data(timer));
    display->AdvanceTypewriter();
}

void MochanDisplay::CursorTimerCallback(lv_timer_t* timer) {
    auto* display = static_cast<MochanDisplay*>(lv_timer_get_user_data(timer));
    display->BlinkTerminalCursor();
}

void MochanDisplay::TerminalScaleAnimCallback(void* object, int32_t value) {
    lv_obj_set_style_transform_scale(static_cast<lv_obj_t*>(object), value, 0);
}

void MochanDisplay::AdvanceTypewriter() {
    if (typewriter_position_ >= typewriter_target_.size()) {
        lv_timer_pause(typewriter_timer_);
        return;
    }

    const unsigned char lead =
        static_cast<unsigned char>(typewriter_target_[typewriter_position_]);
    size_t codepoint_bytes = 1;
    if ((lead & 0xE0) == 0xC0) {
        codepoint_bytes = 2;
    } else if ((lead & 0xF0) == 0xE0) {
        codepoint_bytes = 3;
    } else if ((lead & 0xF8) == 0xF0) {
        codepoint_bytes = 4;
    }
    codepoint_bytes =
        std::min(codepoint_bytes, typewriter_target_.size() - typewriter_position_);

    typewriter_visible_.append(typewriter_target_, typewriter_position_, codepoint_bytes);
    typewriter_position_ += codepoint_bytes;
    terminal_cursor_visible_ = true;
    RenderTerminalTextLocked();

    if (typewriter_position_ >= typewriter_target_.size()) {
        lv_timer_pause(typewriter_timer_);
    }
}

void MochanDisplay::BlinkTerminalCursor() {
    if (!terminal_cursor_enabled_) {
        return;
    }
    terminal_cursor_visible_ = !terminal_cursor_visible_;
    RenderTerminalTextLocked();
}

void MochanDisplay::RenderTerminalTextLocked() {
    std::string rendered;
    if (terminal_prompt_active_) {
        rendered = "> ";
    }
    rendered += typewriter_visible_;
    if (terminal_cursor_enabled_ && terminal_cursor_visible_) {
        rendered += "|";
    }
    ShowTextLocked(rendered.c_str());
}

void MochanDisplay::ShowTerminalPopupLocked() {
    if (bottom_bar_ == nullptr || preview_visible_) {
        return;
    }

    lv_anim_delete(bottom_bar_, nullptr);
    lv_obj_set_style_transform_scale(bottom_bar_, 218, 0);
    lv_obj_set_style_opa(bottom_bar_, LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(bottom_bar_);

    lv_anim_t scale_animation;
    lv_anim_init(&scale_animation);
    lv_anim_set_var(&scale_animation, bottom_bar_);
    lv_anim_set_values(&scale_animation, 218, 256);
    lv_anim_set_exec_cb(&scale_animation, TerminalScaleAnimCallback);
    lv_anim_set_duration(&scale_animation, kTerminalPopupPeriodMs);
    lv_anim_set_path_cb(&scale_animation, lv_anim_path_ease_out);
    lv_anim_start(&scale_animation);

    lv_obj_fade_in(bottom_bar_, kTerminalPopupPeriodMs - 40, 0);
}

void MochanDisplay::HideTerminalLocked() {
    if (bottom_bar_ == nullptr) {
        return;
    }
    lv_anim_delete(bottom_bar_, nullptr);
    lv_obj_set_style_transform_scale(bottom_bar_, 256, 0);
    lv_obj_set_style_opa(bottom_bar_, LV_OPA_COVER, 0);
    lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
}

void MochanDisplay::HideBootLoadingLocked() {
    if (loading_overlay_ != nullptr) {
        lv_obj_add_flag(loading_overlay_, LV_OBJ_FLAG_HIDDEN);
    }
    boot_loading_visible_ = false;
}

void MochanDisplay::ShowTextLocked(const char* content) {
    lv_label_set_text(chat_message_label_, content != nullptr ? content : "");
    ScrollTextToBottomLocked();
}

void MochanDisplay::ScrollTextToBottomLocked() {
    lv_obj_update_layout(chat_message_label_);
    lv_obj_scroll_to_y(bottom_bar_, LV_COORD_MAX, LV_ANIM_OFF);
}

void MochanDisplay::ClearChatMessages() {
    if (!setup_ui_called_ || chat_message_label_ == nullptr) {
        return;
    }
    DisplayLockGuard lock(this);
    typewriter_target_.clear();
    typewriter_visible_.clear();
    typewriter_position_ = 0;
    terminal_prompt_active_ = false;
    terminal_cursor_enabled_ = false;
    terminal_cursor_visible_ = false;
    terminal_has_content_ = false;
    lv_timer_pause(typewriter_timer_);
    lv_timer_pause(cursor_timer_);
    lv_label_set_text(chat_message_label_, "");
    lv_obj_scroll_to_y(bottom_bar_, 0, LV_ANIM_OFF);
    HideTerminalLocked();
}

void MochanDisplay::SetTheme(Theme* theme) {
    LcdDisplay::SetTheme(theme);
    if (!setup_ui_called_) {
        return;
    }

    DisplayLockGuard lock(this);
    auto* lvgl_theme = static_cast<LvglTheme*>(theme);
    const lv_color_t black = lv_color_hex(0x000000);
    lv_obj_set_style_bg_color(lv_screen_active(), black, 0);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(container_, black, 0);
    lv_obj_set_style_bg_opa(container_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_image_src(container_, nullptr, 0);
    lv_obj_set_style_bg_color(emoji_box_, black, 0);
    lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bottom_bar_, lv_color_hex(0x03090C), 0);
    lv_obj_set_style_bg_opa(bottom_bar_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bottom_bar_, 1, 0);
    lv_obj_set_style_border_color(bottom_bar_, lv_color_hex(0x16414A), 0);
    lv_obj_set_style_border_opa(bottom_bar_, LV_OPA_60, 0);
    if (camera_box_ != nullptr) {
        lv_obj_set_style_bg_color(camera_box_, black, 0);
        lv_obj_set_style_bg_opa(camera_box_, LV_OPA_COVER, 0);
    }
    lv_obj_set_style_bg_color(top_bar_, black, 0);
    lv_obj_set_style_bg_opa(top_bar_, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(network_label_, lv_color_hex(0xEAFBFF), 0);
    if (lvgl_theme != nullptr && lvgl_theme->text_font() != nullptr) {
        // Assets can replace the font object after SetupUI. Rebind the label before the
        // previous font owner is released, otherwise LVGL keeps a dangling lv_font_t pointer.
        lv_obj_set_style_text_font(chat_message_label_, lvgl_theme->text_font()->font(), 0);
        if (loading_label_ != nullptr) {
            lv_obj_set_style_text_font(loading_label_, lvgl_theme->text_font()->font(), 0);
        }
    }
    lv_obj_set_style_text_color(chat_message_label_, lv_color_hex(0xEAFBFF), 0);
    if (loading_overlay_ != nullptr) {
        lv_obj_set_style_bg_color(loading_overlay_, black, 0);
        lv_obj_set_style_bg_opa(loading_overlay_, LV_OPA_COVER, 0);
    }
}

void MochanDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
    DisplayLockGuard lock(this);
    if (preview_image_ == nullptr) {
        ESP_LOGE(TAG, "Preview image is not initialized");
        return;
    }

    if (image == nullptr) {
        esp_timer_stop(preview_timer_);
        preview_visible_ = false;
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(camera_box_, LV_OBJ_FLAG_HIDDEN);
        if (terminal_has_content_) {
            ShowTerminalPopupLocked();
        } else {
            HideTerminalLocked();
        }
        preview_image_cached_.reset();
        return;
    }

    preview_visible_ = true;
    preview_image_cached_ = std::move(image);
    const auto* image_descriptor = preview_image_cached_->image_dsc();
    lv_image_set_src(preview_image_, image_descriptor);

    if (image_descriptor->header.w > 0 && image_descriptor->header.h > 0) {
        const int scale_x = 256 * width_ / image_descriptor->header.w;
        const int camera_height = height_ - kStatusBarHeight - kEyeHeight;
        const int scale_y = 256 * camera_height / image_descriptor->header.h;
        // Fill the text area completely; the camera box clips the excess vertically.
        lv_image_set_scale(preview_image_, std::max(scale_x, scale_y));
    }

    lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(camera_box_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(camera_box_);

    esp_timer_stop(preview_timer_);
    ESP_ERROR_CHECK(esp_timer_start_once(preview_timer_, PREVIEW_IMAGE_DURATION_MS * 1000ULL));
}
