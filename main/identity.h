// The identity the voice server knows this robot by: the Wi-Fi MAC address and the UUID
// kept in NVS ("board"/"uuid", created once if missing). Keeping both keeps the robot bound
// to its account on the server.
#pragma once
#include <string>

namespace identity {
std::string Mac();   // "aa:bb:cc:dd:ee:ff"
std::string Uuid();
}  // namespace identity
