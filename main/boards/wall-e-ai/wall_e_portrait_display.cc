#include "wall_e_portrait_display.h"

#include <cctype>
#include <cstdint>
#include <cstring>
#include <vector>

#include <esp_err.h>
#include <esp_log.h>

namespace {
constexpr char kTag[] = "WallEDisplay";
constexpr uint8_t kOledAddress = 0x3c;

const uint8_t* Glyph(char c) {
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t letters[][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
        {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
        {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
        {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
        {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
        {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
        {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
        {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
    };
    static const uint8_t digits[][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    if (c >= 'A' && c <= 'Z')
        return letters[c - 'A'];
    if (c >= '0' && c <= '9')
        return digits[c - '0'];
    if (c == '-')
        return dash;
    if (c == '.')
        return dot;
    if (c == ':')
        return colon;
    return blank;
}

enum TinyForm : uint8_t { kFormNone, kFormBreve, kFormCircumflex, kFormHorn, kFormStroke };
enum TinyTone : uint8_t { kToneNone, kToneAcute, kToneGrave, kToneHook, kToneTilde, kToneDot };

struct TinyCharacter {
    char base = ' ';
    TinyForm form = kFormNone;
    TinyTone tone = kToneNone;
};

const uint8_t* TinyGlyph(char c) {
    static const uint8_t blank[3] = {0, 0, 0};
    static const uint8_t letters[][3] = {
        {0x1e, 0x05, 0x1e}, {0x1f, 0x15, 0x0a}, {0x0e, 0x11, 0x11}, {0x1f, 0x11, 0x0e},
        {0x1f, 0x15, 0x15}, {0x1f, 0x05, 0x05}, {0x0e, 0x11, 0x1d}, {0x1f, 0x04, 0x1f},
        {0x11, 0x1f, 0x11}, {0x10, 0x10, 0x0f}, {0x1f, 0x04, 0x1b}, {0x1f, 0x10, 0x10},
        {0x1f, 0x06, 0x1f}, {0x1f, 0x0e, 0x1f}, {0x0e, 0x11, 0x0e}, {0x1f, 0x05, 0x02},
        {0x0e, 0x11, 0x1e}, {0x1f, 0x05, 0x1a}, {0x12, 0x15, 0x09}, {0x01, 0x1f, 0x01},
        {0x0f, 0x10, 0x0f}, {0x07, 0x18, 0x07}, {0x1f, 0x0c, 0x1f}, {0x1b, 0x04, 0x1b},
        {0x03, 0x1c, 0x03}, {0x19, 0x15, 0x13},
    };
    static const uint8_t digits[][3] = {
        {0x0e, 0x11, 0x0e}, {0x12, 0x1f, 0x10}, {0x19, 0x15, 0x12}, {0x11, 0x15, 0x0a},
        {0x07, 0x04, 0x1f}, {0x17, 0x15, 0x09}, {0x0e, 0x15, 0x08}, {0x01, 0x1d, 0x03},
        {0x0a, 0x15, 0x0a}, {0x02, 0x15, 0x0e},
    };
    static const uint8_t dash[3] = {0x04, 0x04, 0x04};
    static const uint8_t dot[3] = {0x00, 0x10, 0x00};
    static const uint8_t comma[3] = {0x00, 0x18, 0x00};
    static const uint8_t colon[3] = {0x00, 0x0a, 0x00};
    static const uint8_t question[3] = {0x01, 0x15, 0x02};
    static const uint8_t exclamation[3] = {0x00, 0x17, 0x00};
    if (c >= 'A' && c <= 'Z')
        return letters[c - 'A'];
    if (c >= '0' && c <= '9')
        return digits[c - '0'];
    if (c == '-')
        return dash;
    if (c == '.')
        return dot;
    if (c == ',')
        return comma;
    if (c == ':')
        return colon;
    if (c == '?')
        return question;
    if (c == '!')
        return exclamation;
    return blank;
}

bool MapToneSeries(uint32_t codepoint, uint32_t first, char base, TinyForm form,
                   TinyCharacter& result) {
    if (codepoint < first || codepoint > first + 9)
        return false;
    static constexpr TinyTone tones[] = {kToneAcute, kToneGrave, kToneHook, kToneTilde, kToneDot};
    result = {base, form, tones[(codepoint - first) / 2]};
    return true;
}

TinyCharacter MapVietnamese(uint32_t codepoint) {
    TinyCharacter result;
    if (codepoint < 0x80) {
        result.base = static_cast<char>(std::toupper(static_cast<unsigned char>(codepoint)));
        return result;
    }

    if (MapToneSeries(codepoint, 0x1ea4, 'A', kFormCircumflex, result) ||
        MapToneSeries(codepoint, 0x1eae, 'A', kFormBreve, result) ||
        MapToneSeries(codepoint, 0x1ebe, 'E', kFormCircumflex, result) ||
        MapToneSeries(codepoint, 0x1ed0, 'O', kFormCircumflex, result) ||
        MapToneSeries(codepoint, 0x1eda, 'O', kFormHorn, result) ||
        MapToneSeries(codepoint, 0x1ee8, 'U', kFormHorn, result)) {
        return result;
    }

    switch (codepoint) {
        case 0x00c0:
        case 0x00e0:
            return {'A', kFormNone, kToneGrave};
        case 0x00c1:
        case 0x00e1:
            return {'A', kFormNone, kToneAcute};
        case 0x00c2:
        case 0x00e2:
            return {'A', kFormCircumflex, kToneNone};
        case 0x00c3:
        case 0x00e3:
            return {'A', kFormNone, kToneTilde};
        case 0x0102:
        case 0x0103:
            return {'A', kFormBreve, kToneNone};
        case 0x1ea0:
        case 0x1ea1:
            return {'A', kFormNone, kToneDot};
        case 0x1ea2:
        case 0x1ea3:
            return {'A', kFormNone, kToneHook};

        case 0x00c8:
        case 0x00e8:
            return {'E', kFormNone, kToneGrave};
        case 0x00c9:
        case 0x00e9:
            return {'E', kFormNone, kToneAcute};
        case 0x00ca:
        case 0x00ea:
            return {'E', kFormCircumflex, kToneNone};
        case 0x1eb8:
        case 0x1eb9:
            return {'E', kFormNone, kToneDot};
        case 0x1eba:
        case 0x1ebb:
            return {'E', kFormNone, kToneHook};
        case 0x1ebc:
        case 0x1ebd:
            return {'E', kFormNone, kToneTilde};

        case 0x00cc:
        case 0x00ec:
            return {'I', kFormNone, kToneGrave};
        case 0x00cd:
        case 0x00ed:
            return {'I', kFormNone, kToneAcute};
        case 0x0128:
        case 0x0129:
            return {'I', kFormNone, kToneTilde};
        case 0x1ec8:
        case 0x1ec9:
            return {'I', kFormNone, kToneHook};
        case 0x1eca:
        case 0x1ecb:
            return {'I', kFormNone, kToneDot};

        case 0x00d2:
        case 0x00f2:
            return {'O', kFormNone, kToneGrave};
        case 0x00d3:
        case 0x00f3:
            return {'O', kFormNone, kToneAcute};
        case 0x00d4:
        case 0x00f4:
            return {'O', kFormCircumflex, kToneNone};
        case 0x00d5:
        case 0x00f5:
            return {'O', kFormNone, kToneTilde};
        case 0x01a0:
        case 0x01a1:
            return {'O', kFormHorn, kToneNone};
        case 0x1ecc:
        case 0x1ecd:
            return {'O', kFormNone, kToneDot};
        case 0x1ece:
        case 0x1ecf:
            return {'O', kFormNone, kToneHook};

        case 0x00d9:
        case 0x00f9:
            return {'U', kFormNone, kToneGrave};
        case 0x00da:
        case 0x00fa:
            return {'U', kFormNone, kToneAcute};
        case 0x0168:
        case 0x0169:
            return {'U', kFormNone, kToneTilde};
        case 0x01af:
        case 0x01b0:
            return {'U', kFormHorn, kToneNone};
        case 0x1ee4:
        case 0x1ee5:
            return {'U', kFormNone, kToneDot};
        case 0x1ee6:
        case 0x1ee7:
            return {'U', kFormNone, kToneHook};

        case 0x00dd:
        case 0x00fd:
            return {'Y', kFormNone, kToneAcute};
        case 0x1ef2:
        case 0x1ef3:
            return {'Y', kFormNone, kToneGrave};
        case 0x1ef4:
        case 0x1ef5:
            return {'Y', kFormNone, kToneDot};
        case 0x1ef6:
        case 0x1ef7:
            return {'Y', kFormNone, kToneHook};
        case 0x1ef8:
        case 0x1ef9:
            return {'Y', kFormNone, kToneTilde};
        case 0x0110:
        case 0x0111:
            return {'D', kFormStroke, kToneNone};
        default:
            return {' ', kFormNone, kToneNone};
    }
}

std::vector<TinyCharacter> DecodeUtf8(const char* text) {
    std::vector<TinyCharacter> result;
    if (text == nullptr)
        return result;
    const auto* cursor = reinterpret_cast<const uint8_t*>(text);
    while (*cursor != 0) {
        uint32_t codepoint = 0;
        if (*cursor < 0x80) {
            codepoint = *cursor++;
        } else if ((*cursor & 0xe0) == 0xc0 && cursor[1] != 0) {
            codepoint = ((*cursor & 0x1f) << 6) | (cursor[1] & 0x3f);
            cursor += 2;
        } else if ((*cursor & 0xf0) == 0xe0 && cursor[1] != 0 && cursor[2] != 0) {
            codepoint = ((*cursor & 0x0f) << 12) | ((cursor[1] & 0x3f) << 6) | (cursor[2] & 0x3f);
            cursor += 3;
        } else {
            ++cursor;
            continue;
        }
        if (codepoint == '\n' || codepoint == '\r' || codepoint == '\t')
            codepoint = ' ';
        if (!result.empty()) {
            TinyCharacter& previous = result.back();
            switch (codepoint) {
                case 0x0300:
                    previous.tone = kToneGrave;
                    continue;
                case 0x0301:
                    previous.tone = kToneAcute;
                    continue;
                case 0x0303:
                    previous.tone = kToneTilde;
                    continue;
                case 0x0309:
                    previous.tone = kToneHook;
                    continue;
                case 0x0323:
                    previous.tone = kToneDot;
                    continue;
                case 0x0302:
                    previous.form = kFormCircumflex;
                    continue;
                case 0x0306:
                    previous.form = kFormBreve;
                    continue;
                case 0x031b:
                    previous.form = kFormHorn;
                    continue;
                default:
                    break;
            }
        }
        result.push_back(MapVietnamese(codepoint));
    }
    return result;
}
}  // namespace

WallEPortraitDisplay::WallEPortraitDisplay(i2c_master_bus_handle_t bus) {
    width_ = 64;
    height_ = 128;
    i2c_device_config_t config = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                  .device_address = kOledAddress,
                                  .scl_speed_hz = 400000,
                                  .scl_wait_us = 0,
                                  .flags = {}};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &config, &device_));
    const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B, 0xA1, 0xC8,
                            0xDA, 0x12, 0x81, 0x80, 0xD9, 0x22, 0xDB, 0x35, 0xA4, 0xA6, 0xAF};
    for (uint8_t command : init)
        Command(command);
}

WallEPortraitDisplay::~WallEPortraitDisplay() {
    if (device_ != nullptr)
        i2c_master_bus_rm_device(device_);
}
bool WallEPortraitDisplay::Lock(int) {
    mutex_.lock();
    return true;
}
void WallEPortraitDisplay::Unlock() { mutex_.unlock(); }
void WallEPortraitDisplay::Command(uint8_t command) {
    const uint8_t packet[] = {0x00, command};
    const esp_err_t err = i2c_master_transmit(device_, packet, sizeof(packet), 100);
    if (err != ESP_OK)
        ESP_LOGW(kTag, "OLED command 0x%02x failed: %s", command, esp_err_to_name(err));
}

std::string WallEPortraitDisplay::Normalize(const char* text) {
    std::string out;
    if (text == nullptr)
        return out;
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p; ++p)
        if (*p < 0x80)
            out.push_back(static_cast<char>(std::toupper(*p)));
    return out;
}

void WallEPortraitDisplay::Pixel(int x, int y) {
    if (x < 0 || x >= 64 || y < 0 || y >= 128)
        return;
    const int physical_x = y;
    const int physical_y = 63 - x;
    frame_[physical_x + (physical_y / 8) * 128] |= 1U << (physical_y & 7);
}

void WallEPortraitDisplay::Text(int x, int y, const std::string& text) {
    for (char c : text) {
        if (x > 58)
            break;
        const uint8_t* glyph = Glyph(c);
        for (int col = 0; col < 5; ++col)
            for (int row = 0; row < 7; ++row)
                if (glyph[col] & (1U << row))
                    Pixel(x + col, y + row);
        x += 6;
    }
}

void WallEPortraitDisplay::Render() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    frame_.fill(0);

    // Two-line header: icons have their own row, so neither can overlap the
    // full WALL-E AI title on this narrow 64-pixel portrait canvas.
    for (int x = 1; x < 63; ++x) {
        Pixel(x, 1);
        Pixel(x, 23);
    }
    for (int bar = 0; bar < 3; ++bar)
        for (int y = 0; y < 2 + bar * 2; ++y)
            for (int x = 0; x < 2; ++x)
                Pixel(3 + bar * 3 + x, 10 - y);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
            Pixel(56 + x, 5 + y);
    Text(5, 14, "WALL-E AI");

    Text(2, 28, "WIFI WALL-E");
    Text(2, 39, Normalize(status_.c_str()).substr(0, 10));
    // Terminal panel for Xiaozhi replies.
    for (int x = 1; x < 63; ++x) {
        Pixel(x, 52);
        Pixel(x, 126);
    }
    for (int y = 52; y < 127; ++y) {
        Pixel(1, y);
        Pixel(62, y);
    }
    Text(4, 56, "AI REPLY");
    const auto reply_characters = DecodeUtf8(message_.c_str());
    constexpr size_t kCharactersPerLine = 14;
    const auto draw_character = [this](int x, int y, const TinyCharacter& character) {
        const uint8_t* glyph = TinyGlyph(character.base);
        for (int column = 0; column < 3; ++column) {
            for (int row = 0; row < 5; ++row) {
                if (glyph[column] & (1U << row))
                    Pixel(x + column, y + 3 + row);
            }
        }
        if (character.form == kFormCircumflex) {
            Pixel(x, y + 2);
            Pixel(x + 1, y + 1);
            Pixel(x + 2, y + 2);
        } else if (character.form == kFormBreve) {
            Pixel(x, y + 1);
            Pixel(x + 1, y + 2);
            Pixel(x + 2, y + 1);
        } else if (character.form == kFormHorn) {
            Pixel(x + 2, y + 1);
            Pixel(x + 2, y + 2);
        } else if (character.form == kFormStroke) {
            Pixel(x, y + 5);
            Pixel(x + 1, y + 5);
            Pixel(x + 2, y + 5);
        }
        switch (character.tone) {
            case kToneAcute:
                Pixel(x + 2, y);
                break;
            case kToneGrave:
                Pixel(x, y);
                break;
            case kToneHook:
                Pixel(x + 1, y);
                Pixel(x + 2, y);
                break;
            case kToneTilde:
                Pixel(x, y + 1);
                Pixel(x + 1, y);
                Pixel(x + 2, y + 1);
                break;
            case kToneDot:
                Pixel(x + 1, y + 8);
                break;
            default:
                break;
        }
    };
    for (int row = 0; row < 5; ++row) {
        const size_t offset = static_cast<size_t>(row) * kCharactersPerLine;
        if (offset >= reply_characters.size())
            break;
        const size_t end = std::min(reply_characters.size(), offset + kCharactersPerLine);
        int x = 4;
        for (size_t i = offset; i < end && x <= 58; ++i, x += 4) {
            draw_character(x, 68 + row * 11, reply_characters[i]);
        }
    }
    // A small terminal cursor is shown when Wall-E has no text to display.
    if (reply_characters.empty())
        for (int y = 113; y < 120; ++y)
            Pixel(5, y);
    for (uint8_t page = 0; page < 8; ++page) {
        Command(0xB0 | page);
        Command(0x02);
        Command(0x10);
        uint8_t packet[129] = {0x40};
        std::memcpy(packet + 1, frame_.data() + page * 128, 128);
        const esp_err_t err = i2c_master_transmit(device_, packet, sizeof(packet), 100);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "OLED page %u failed: %s", page, esp_err_to_name(err));
        }
    }
}

void WallEPortraitDisplay::SetupUI() {
    Display::SetupUI();
    Render();
}
void WallEPortraitDisplay::SetStatus(const char* status) {
    const std::string next = status ? status : "";
    if (status_ == next)
        return;
    status_ = next;
    Render();
}
void WallEPortraitDisplay::ShowNotification(const char* notification, int) {
    SetStatus(notification);
}
void WallEPortraitDisplay::SetChatMessage(const char*, const char* content) {
    const std::string next = content ? content : "";
    if (message_ == next)
        return;
    message_ = next;
    Render();
}
void WallEPortraitDisplay::ClearChatMessages() {
    if (message_.empty())
        return;
    message_.clear();
    Render();
}
void WallEPortraitDisplay::SetEmotion(const char*) {
    // This display intentionally has no emoji layer; keep the status text intact.
}
