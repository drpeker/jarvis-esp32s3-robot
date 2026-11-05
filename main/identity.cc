#include "identity.h"

#include <cstdio>

#include <esp_mac.h>
#include <esp_random.h>

#include "settings.h"

namespace identity {

std::string Mac() {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char s[18];
    snprintf(s, sizeof(s), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return s;
}

std::string Uuid() {
    static std::string uuid;
    if (!uuid.empty()) return uuid;
    uuid = settings::GetStr("board", "uuid");
    if (uuid.empty()) {
        uint8_t b[16];
        esp_fill_random(b, sizeof(b));
        b[6] = (b[6] & 0x0F) | 0x40;  // version 4
        b[8] = (b[8] & 0x3F) | 0x80;  // variant
        char s[37];
        snprintf(s, sizeof(s), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", b[0], b[1],
                 b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
        uuid = s;
        settings::SetStr("board", "uuid", uuid);
    }
    return uuid;
}

}  // namespace identity
