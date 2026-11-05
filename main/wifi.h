// Wi-Fi. Networks stored on the robot (NVS namespace "wifi": ssid/password, ssid1..9/
// password1..9); the console 'wifi <ssid> <password>' and the setup portal store them.
//   - Every attempt: fresh scan, then join the strongest AP of a known network (by BSSID).
//   - No known network, or none joined within 30 s: the robot opens its own network
//     "Jarvis04112026" with a setup page (wifi_portal) where a network is chosen. While it
//     is open, the known networks are still tried every 60 s when no phone is on it.
//   - Once connected, the setup network closes.
// One task ("wifi") does all of it in order; the Wi-Fi/IP event handlers only post to it.
#pragma once
#include <string>
#include <vector>

namespace wifi {
void Start();
bool WaitConnected(int timeout_ms);

// For the setup portal (any task).
struct Network {
    std::string ssid;
    int rssi;
    bool secure;
};
std::vector<Network> LastScan();  // networks seen by the last scan, strongest first
void RequestScan();               // refresh LastScan in the background
void Join(const std::string& ssid, const std::string& password);  // store it and try it
std::string LastError();          // "" or why the last portal attempt failed
}  // namespace wifi
