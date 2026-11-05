// The conversation: one state machine on top of a ServerLink.
//   idle --wake--> connecting --session open--> listening (microphone -> Opus -> server)
//   listening --server speaks--> speaking (server Opus -> speaker; microphone not sent)
//   speaking --server done and audio played--> listening ... server closes -> idle
// Three tasks with one job each: control (events, in order), microphone, speaker. Network
// events only queue work and never wait.
#pragma once
#include <string>

class ServerLink;

namespace conversation {
void Start(ServerLink* link);
void Wake(bool by_wake_word);  // wake word, BOOT button or console
std::string Describe();        // console
}  // namespace conversation
