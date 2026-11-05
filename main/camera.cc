#include "camera.h"

#include <mutex>

#include <esp_camera.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "pins.h"

#define TAG "camera"

namespace {
std::mutex g_mutex;

camera_config_t Config() {
    camera_config_t c = {};
    c.pin_pwdn = -1;
    c.pin_reset = -1;  // not wired: the driver resets the sensor over SCCB (XCLK running)
    c.pin_xclk = PIN_CAM_XCLK;
    c.pin_sccb_sda = -1;  // use the shared I2C bus (port 1, created by i2c_bus::Init)
    c.pin_sccb_scl = -1;
    c.sccb_i2c_port = I2C_PORT;
    c.pin_d7 = PIN_CAM_D7;
    c.pin_d6 = PIN_CAM_D6;
    c.pin_d5 = PIN_CAM_D5;
    c.pin_d4 = PIN_CAM_D4;
    c.pin_d3 = PIN_CAM_D3;
    c.pin_d2 = PIN_CAM_D2;
    c.pin_d1 = PIN_CAM_D1;
    c.pin_d0 = PIN_CAM_D0;
    c.pin_vsync = PIN_CAM_VSYNC;
    c.pin_href = PIN_CAM_HREF;
    c.pin_pclk = PIN_CAM_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0;     // motors use timer 2, channels 1-4
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = FRAMESIZE_HVGA;   // 480x320: enough for the vision model, half the image tokens of VGA
    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    return c;
}
}  // namespace

namespace camera {

bool TakePhoto(std::vector<uint8_t>& jpeg, int* width, int* height) {
    std::lock_guard<std::mutex> lock(g_mutex);
    const int64_t t0 = esp_timer_get_time();
    camera_config_t cfg = Config();
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return false;
    }
    if (sensor_t* s = esp_camera_sensor_get()) {
        s->set_vflip(s, 1);  // mounted upside down
        s->set_hmirror(s, 1);
    }
    bool ok = false;
    for (int i = 0; i < 3; ++i) {  // first frames: exposure still settling
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == nullptr) continue;
        if (i == 2) {
            jpeg.assign(fb->buf, fb->buf + fb->len);
            if (width) *width = fb->width;
            if (height) *height = fb->height;
            ok = true;
        }
        esp_camera_fb_return(fb);
    }
    esp_camera_deinit();
    ESP_LOGI(TAG, "photo %s: %u bytes in %d ms", ok ? "ok" : "FAILED", (unsigned)jpeg.size(),
             (int)((esp_timer_get_time() - t0) / 1000));
    return ok;
}

}  // namespace camera
