#include "status.h"

#include <atomic>

namespace {
std::atomic<RobotState> g_state{RobotState::kStarting};
std::atomic<bool> g_wifi{false};
std::atomic<int> g_rssi{-100};
}  // namespace

namespace status {
RobotState Get() { return g_state.load(); }
void Set(RobotState s) { g_state.store(s); }
const char* Name(RobotState s) {
    switch (s) {
        case RobotState::kStarting: return "starting";
        case RobotState::kOffline: return "offline";
        case RobotState::kIdle: return "idle";
        case RobotState::kListening: return "listening";
        case RobotState::kThinking: return "thinking";
        case RobotState::kSpeaking: return "speaking";
        case RobotState::kUpdating: return "updating";
    }
    return "?";
}
bool WifiConnected() { return g_wifi.load(); }
int WifiRssi() { return g_rssi.load(); }
void SetWifi(bool connected, int rssi) {
    g_wifi.store(connected);
    g_rssi.store(rssi);
}
}  // namespace status
