// "Jarvis" wake word (Espressif esp-sr WakeNet, wn9_jarvis_tts). The model is packed from the
// esp-sr component at build time and embedded in the app image (main/CMakeLists.txt), so the
// firmware needs nothing else in flash.
#pragma once
#include <cstdint>

namespace wake_word {
bool Start();           // false: no model found (wake by the BOOT button only)
int ChunkSamples();     // samples per Detect call (16 kHz mono)
bool Detect(int16_t* samples);
}  // namespace wake_word
