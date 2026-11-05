// Jarvis's tools for the voice server, over MCP (JSON-RPC 2.0: initialize, tools/list,
// tools/call). Independent of the server: requests come in as text, replies go out through
// the `send` function given to Start(). Calls run one at a time in their own task, so a
// long move never blocks the network or the audio.
#pragma once
#include <functional>
#include <string>

namespace tools {
void Start(std::function<void(const std::string& payload)> send);
void HandleMcp(const std::string& payload);  // from the server; returns at once
}  // namespace tools
