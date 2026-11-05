#include "wake_word.h"

#include <cstring>
#include <string>

#include <esp_log.h>
#include <esp_wn_iface.h>
#include <esp_wn_models.h>
#include <model_path.h>

#define TAG "wake"

// srmodels.bin, packed from the esp-sr component at build time (main/CMakeLists.txt).
extern const uint8_t kModelsStart[] asm("_binary_srmodels_bin_start");
extern const uint8_t kModelsEnd[] asm("_binary_srmodels_bin_end");

namespace {
const esp_wn_iface_t* g_iface = nullptr;
model_iface_data_t* g_model = nullptr;
int g_chunk = 0;
}  // namespace

namespace wake_word {

bool Start() {
    // The models stay in the app image (flash, read through the cache): no RAM copy.
    ESP_LOGI(TAG, "models in the app: %u bytes", (unsigned)(kModelsEnd - kModelsStart));
    srmodel_list_t* list = srmodel_load(kModelsStart);
    char* name = list ? esp_srmodel_filter(list, ESP_WN_PREFIX, "jarvis") : nullptr;
    if (!name) {
        ESP_LOGW(TAG, "no Jarvis WakeNet model in srmodels.bin");
        return false;
    }
    g_iface = esp_wn_handle_from_name(name);
    g_model = g_iface ? g_iface->create(name, DET_MODE_95) : nullptr;
    if (!g_model) return false;
    g_chunk = g_iface->get_samp_chunksize(g_model);
    ESP_LOGI(TAG, "wake word ready: %s, %d samples per chunk", name, g_chunk);
    return true;
}

int ChunkSamples() { return g_chunk; }

bool Detect(int16_t* samples) {
    return g_model && g_iface->detect(g_model, samples) == WAKENET_DETECTED;
}

}  // namespace wake_word
