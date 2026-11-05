// Opus for the voice link (Espressif esp_audio_codec): 16 kHz mono 60 ms frames to the
// server, server frames decoded at the rate the server announces. One encoder and one
// decoder, buffers allocated once.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace opus {
constexpr int kFrameMs = 60;
constexpr int kMicRate = 16000;
constexpr int kMicFrameSamples = kMicRate * kFrameMs / 1000;  // 960

bool OpenEncoder();
// pcm: kMicFrameSamples samples. Returns the packet size (0 on error).
size_t Encode(const int16_t* pcm, uint8_t* out, size_t out_max);

bool OpenDecoder(int sample_rate);
int DecoderRate();
// Returns decoded samples (mono) written to pcm, 0 on error.
size_t Decode(const uint8_t* packet, size_t len, int16_t* pcm, size_t pcm_max);
}  // namespace opus
