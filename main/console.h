// Telnet console on port 23 (nc <robot-ip> 23): one client at a time, live log on request,
// tests and tuning, OTA. The task stack is in internal RAM, so commands may write NVS.
#pragma once

namespace console {
void Start();
}  // namespace console
