// INMP441 microphone (I2S0, 16 kHz) and MAX98357A speaker (I2S1). Half duplex like
// NeuroVision: the microphone is not sent while Jarvis speaks, so there is no echo canceller.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace audio {
void Start();

// Blocks until one utterance was heard (energy gate against the room noise floor:
// >= 250 ms of speech, ends after 800 ms of silence, at most max_ms) or until timeout_ms
// passes without speech. Returns false on timeout. Samples are 16 kHz mono int16 (PSRAM).
bool Listen(std::vector<int16_t>& out, int timeout_ms, int max_ms = 12000);
float NoiseFloorDb();

// Plays a WAV file (PCM16, any rate, mono or stereo). Blocks until done or StopPlayback().
bool PlayWav(const uint8_t* data, size_t size);
void StopPlayback();
bool Playing();
void Beep(int hz, int ms);

// Streaming (voice link): one microphone frame of `samples` (multiple of 320) at 16 kHz,
// after dropping what queued up; and PCM to the speaker at `rate`, blocking until written.
void FlushMic();
bool ReadMic(int16_t* pcm, int samples);
void PlayPcm(const int16_t* pcm, size_t samples, int rate);
void SetStreaming(bool on);  // marks speech output for the face (mouth) while a stream plays

int Volume();             // 0..100 (NVS audio/volume)
void SetVolume(int percent);
}  // namespace audio
