#include "opus_codec.h"

#include <esp_log.h>
#include <decoder/impl/esp_opus_dec.h>
#include <encoder/impl/esp_opus_enc.h>

#define TAG "opus"

namespace {
void* g_enc = nullptr;
void* g_dec = nullptr;
int g_dec_rate = 0;
}  // namespace

namespace opus {

bool OpenEncoder() {
    if (g_enc) return true;
    esp_opus_enc_config_t cfg = ESP_OPUS_ENC_CONFIG_DEFAULT();
    cfg.sample_rate = kMicRate;
    cfg.channel = 1;
    cfg.bitrate = 24000;
    cfg.frame_duration = ESP_OPUS_ENC_FRAME_DURATION_60_MS;
    cfg.application_mode = ESP_OPUS_ENC_APPLICATION_VOIP;
    cfg.complexity = 0;  // lowest CPU and stack use (libopus keeps large arrays on the stack)
    if (esp_opus_enc_open(&cfg, sizeof(cfg), &g_enc) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "encoder open failed");
        g_enc = nullptr;
        return false;
    }
    return true;
}

size_t Encode(const int16_t* pcm, uint8_t* out, size_t out_max) {
    if (!g_enc) return 0;
    esp_audio_enc_in_frame_t in = {};
    in.buffer = (uint8_t*)pcm;
    in.len = kMicFrameSamples * sizeof(int16_t);
    esp_audio_enc_out_frame_t o = {};
    o.buffer = out;
    o.len = out_max;
    if (esp_opus_enc_process(g_enc, &in, &o) != ESP_AUDIO_ERR_OK) return 0;
    return o.encoded_bytes;
}

bool OpenDecoder(int sample_rate) {
    if (g_dec && g_dec_rate == sample_rate) return true;
    if (g_dec) {
        esp_opus_dec_close(g_dec);
        g_dec = nullptr;
    }
    esp_opus_dec_cfg_t cfg = ESP_OPUS_DEC_CONFIG_DEFAULT();
    cfg.sample_rate = sample_rate;
    cfg.channel = 1;
    cfg.frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS;
    if (esp_opus_dec_open(&cfg, sizeof(cfg), &g_dec) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "decoder open failed (%d Hz)", sample_rate);
        g_dec = nullptr;
        return false;
    }
    g_dec_rate = sample_rate;
    return true;
}

int DecoderRate() { return g_dec_rate; }

size_t Decode(const uint8_t* packet, size_t len, int16_t* pcm, size_t pcm_max) {
    if (!g_dec) return 0;
    esp_audio_dec_in_raw_t raw = {};
    raw.buffer = (uint8_t*)packet;
    raw.len = len;
    esp_audio_dec_out_frame_t out = {};
    out.buffer = (uint8_t*)pcm;
    out.len = pcm_max * sizeof(int16_t);
    esp_audio_dec_info_t info = {};
    if (esp_opus_dec_decode(g_dec, &raw, &out, &info) != ESP_AUDIO_ERR_OK) return 0;
    return out.decoded_size / sizeof(int16_t);
}

}  // namespace opus
