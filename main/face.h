// Jarvis face on the ST7789 (from NeuroVision-Gemini nv/display.py): eyes, expressions,
// blink, idle glances, mouth,
// Wi-Fi and battery icons, captions, the "Peker / AI Technology" splash and the photo
// preview. Drawn row span by row span into an RGB565 canvas (LVGL 9), only changed rows
// are invalidated.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace face {
void Start();
void SetEmotion(const char* emotion);  // happy, sad, surprised, angry, sleepy, neutral, thinking
void SetCaption(const char* role, const std::string& text);  // "user", "assistant", "system"
void ShowPhoto(const std::vector<uint8_t>& jpeg);
}  // namespace face
