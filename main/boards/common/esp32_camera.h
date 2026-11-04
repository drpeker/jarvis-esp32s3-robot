#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <memory>
#include <mutex>
#include <vector>

#include "camera.h"
#include "esp_camera.h"
#include "jpg/image_to_jpeg.h"

class Esp32Camera : public Camera {
private:
    bool streaming_on_ = false;
    bool swap_bytes_enabled_ = true;  // Swap pixel byte order for RGB565, enabled by default
    std::string explain_url_;
    std::string explain_token_;
    camera_fb_t* current_fb_ = nullptr;
    uint8_t* encode_buf_ = nullptr;  // Buffer for JPEG encoding (with optional byte swap)
    size_t encode_buf_size_ = 0;
    std::mutex mutex_;

    // Photo on demand (opt-in): the driver is started for each photo and stopped right
    // after, so the sensor gets a clean reset with XCLK running every time and no camera
    // DMA runs between photos. The JPEG made by the sensor is kept here for Explain().
    bool on_demand_ = false;
    camera_config_t config_ = {};
    bool hmirror_ = false;
    bool vflip_ = false;
    int drop_frames_ = 3;
    std::vector<uint8_t> jpeg_;
    uint16_t jpeg_width_ = 0;
    uint16_t jpeg_height_ = 0;
    bool StartDriver();
    void ShowPreview();

public:
    Esp32Camera(const camera_config_t& config);
    // on_demand: see above. The camera is probed once here and then stopped.
    Esp32Camera(const camera_config_t& config, bool on_demand);
    bool IsReady() const { return streaming_on_; }
    void SetDropFrames(int frames) { drop_frames_ = frames; }
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual bool SetSwapBytes(bool enabled) override;
    virtual std::string Explain(const std::string& question) override;
    virtual std::mutex* GetMutex() override { return &mutex_; }
};
