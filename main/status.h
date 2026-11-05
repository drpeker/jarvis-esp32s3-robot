// Shared robot state, read by the face and the console, written by the owners.
#pragma once

enum class RobotState { kStarting, kOffline, kIdle, kListening, kThinking, kSpeaking, kUpdating };

namespace status {
RobotState Get();
void Set(RobotState s);
const char* Name(RobotState s);
bool WifiConnected();
int WifiRssi();
void SetWifi(bool connected, int rssi);
}  // namespace status
