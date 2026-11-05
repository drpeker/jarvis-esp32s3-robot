// The robot's connection to a voice server. The conversation talks only to this interface;
// to use another service, write another implementation (xiaozhi_link is the current one).
//
// A session: OpenSession -> SendWakeWord / StartListening -> microphone Opus packets up
// (SendAudio) -> the server answers with on_speak_start, Opus packets (on_audio),
// on_robot_text per sentence, on_speak_stop -> StartListening again ... -> CloseSession or
// the server ends it (on_session_closed). Tools arrive as MCP JSON-RPC (on_mcp / SendMcp).
// Events are called from the link's own task: they must return quickly.
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

class ServerLink {
public:
    struct Events {
        std::function<void(const uint8_t* packet, size_t len)> on_audio;
        std::function<void(const std::string& text)> on_user_text;   // what the server heard
        std::function<void(const std::string& text)> on_robot_text;  // sentence being spoken
        std::function<void(const std::string& emotion)> on_emotion;
        std::function<void()> on_speak_start;
        std::function<void()> on_speak_stop;
        std::function<void(const std::string& payload)> on_mcp;      // JSON-RPC message
        std::function<void()> on_session_closed;
    };

    virtual ~ServerLink() = default;
    virtual bool Start(const Events& events) = 0;  // configuration + control channel
    virtual bool Connected() const = 0;
    virtual bool OpenSession() = 0;                // audio channel; blocks until ready
    virtual void CloseSession() = 0;
    virtual bool SessionOpen() const = 0;
    virtual int SpeakerSampleRate() const = 0;     // rate of the incoming Opus packets
    virtual void SendWakeWord(const std::string& word) = 0;
    virtual void StartListening() = 0;
    virtual void StopListening() = 0;
    virtual void AbortSpeaking() = 0;
    virtual bool SendAudio(const uint8_t* packet, size_t len) = 0;
    virtual void SendMcp(const std::string& payload) = 0;
    virtual std::string Describe() const = 0;      // one line for the console
};
