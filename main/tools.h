// Jarvis's tools for the voice server, over MCP (JSON-RPC 2.0: initialize, tools/list,
// tools/call). Independent of the server: requests come in as text, replies go out through
// the `send` function given to Start(). Calls run one at a time in their own task, so a
// long move never blocks the network or the audio.
#pragma once
#include <functional>
#include <string>
#include <vector>

namespace tools {
void Start(std::function<void(const std::string& payload)> send);
void HandleMcp(const std::string& payload);  // from the server; returns at once
// Console measurement: sends the photo to the vision server like take_photo does.
// upload_ms: until the photo is sent; total_ms: until the answer arrived.
bool MeasureUpload(const std::vector<uint8_t>& jpeg, int* upload_ms, int* total_ms);
}  // namespace tools
