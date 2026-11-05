// Wi-Fi setup portal: the robot's own access point "Jarvis04112026" (WPA2, password shown on
// the face), a DNS server that answers every name with the robot (so the phone opens the
// page by itself), and the setup page at http://192.168.4.1 to choose a network.
// Started and stopped by wifi.cc only.
#pragma once

namespace wifi_portal {
constexpr const char* kSsid = "Jarvis04112026";
constexpr const char* kPassword = "jarvis2026";
void Start();          // Wi-Fi must already run in AP+STA mode
void Stop();
bool Running();
bool InUse();          // a phone is connected to the setup network
}  // namespace wifi_portal
