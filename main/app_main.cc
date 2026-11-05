// Jarvis v1.7. Start order: storage, shared I2C bus, face (splash while the rest starts),
// motors, IMU, battery, audio, Wi-Fi, console, then the conversation with the voice server.
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <nvs_flash.h>

#include "audio.h"
#include "console.h"
#include "conversation.h"
#include "crash.h"
#include "face.h"
#include "i2c_bus.h"
#include "imu.h"
#include "motors.h"
#include "power.h"
#include "status.h"
#include "wifi.h"
#include "xiaozhi_link.h"

extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_LOGI("main", "Jarvis v1.7 starting, last reset: %s", crash::ResetReason());
    status::Set(RobotState::kStarting);
    i2c_bus::Init();
    face::Start();
    motors::Start();
    imu::Start();
    power::Start();
    audio::Start();
    wifi::Start();
    console::Start();
    wifi::WaitConnected(15000);

    // The bootloader has app rollback on: an OTA image must confirm itself once the
    // hardware, Wi-Fi and the console (the way to update again) are up.
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI("main", "new firmware marked valid");
    }
    status::Set(RobotState::kIdle);
    // The voice server: change this one line to use another ServerLink implementation.
    conversation::Start(CreateXiaozhiLink());
}
