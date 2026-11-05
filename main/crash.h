// Crash recorder: the panic reason, task and backtrace are kept in RTC memory (survives
// the reset after a panic) because the robot has no USB log attached. Linked with
// -Wl,--wrap=esp_panic_handler (main/CMakeLists.txt). Console: 'crash', 'crash clear'.
#pragma once
#include <string>

namespace crash {
std::string Describe();  // last crash, or "no crash recorded"
void Clear();
const char* ResetReason();
}  // namespace crash
