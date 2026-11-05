// Small NVS wrapper. Writes touch flash: call them only from tasks whose stack is in
// internal RAM (every task in this project is; never from a PSRAM-stack task).
#pragma once
#include <string>

namespace settings {
int GetInt(const char* ns, const char* key, int fallback);
void SetInt(const char* ns, const char* key, int value);
std::string GetStr(const char* ns, const char* key, const std::string& fallback = "");
void SetStr(const char* ns, const char* key, const std::string& value);
}  // namespace settings
