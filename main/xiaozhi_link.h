// ServerLink for the xiaozhi.me server (written from its wire protocol, no xiaozhi code):
//  - configuration: POST to the OTA URL with the robot's identity -> MQTT credentials
//    (and an activation code while the robot is not bound to an account);
//  - control: MQTT over TLS, JSON messages (hello, listen, abort, goodbye, tts, stt, llm, mcp);
//  - audio: Opus over UDP, AES-128-CTR, 16-byte header used as the counter block:
//    |type 1|flags 1|payload_len 2|ssrc 4|timestamp 4|sequence 4| payload.
#pragma once
#include "server_link.h"

ServerLink* CreateXiaozhiLink();
