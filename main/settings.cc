#include "settings.h"

#include <nvs.h>

namespace settings {

int GetInt(const char* ns, const char* key, int fallback) {
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READONLY, &h) != ESP_OK) return fallback;
    int32_t v = fallback;
    if (nvs_get_i32(h, key, &v) != ESP_OK) v = fallback;
    nvs_close(h);
    return v;
}

void SetInt(const char* ns, const char* key, int value) {
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, key, value);
    nvs_commit(h);
    nvs_close(h);
}

std::string GetStr(const char* ns, const char* key, const std::string& fallback) {
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READONLY, &h) != ESP_OK) return fallback;
    size_t len = 0;
    std::string out = fallback;
    if (nvs_get_str(h, key, nullptr, &len) == ESP_OK && len > 0) {
        out.resize(len);
        nvs_get_str(h, key, out.data(), &len);
        out.resize(len - 1);
    }
    nvs_close(h);
    return out;
}

void SetStr(const char* ns, const char* key, const std::string& value) {
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, key, value.c_str());
    nvs_commit(h);
    nvs_close(h);
}

}  // namespace settings
