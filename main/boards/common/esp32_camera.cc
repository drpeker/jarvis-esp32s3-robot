#include "sdkconfig.h"

#include <esp_heap_caps.h>
#include <cstdio>
#include <cstring>
#include <esp_log.h>
#include <img_converters.h>

#include "esp32_camera.h"
#include "board.h"
#include "display.h"
#include "lvgl_display.h"
#include "mcp_server.h"
#include "system_info.h"
#include "jpg/image_to_jpeg.h"
#include "esp_timer.h"

#define TAG "Esp32Camera"

Esp32Camera::Esp32Camera(const camera_config_t &config) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 0); // Control camera mirror: 1 for mirror, 0 for normal
        }
        ESP_LOGI(TAG, "Camera initialized: format=%d", config.pixel_format);
    }

    streaming_on_ = true;
}

Esp32Camera::~Esp32Camera() {
    if (streaming_on_) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
            current_fb_ = nullptr;
        }
        if (encode_buf_) {
            heap_caps_free(encode_buf_);
            encode_buf_ = nullptr;
            encode_buf_size_ = 0;
        }
        esp_camera_deinit();
        streaming_on_ = false;
    }
}

void Esp32Camera::SetExplainUrl(const std::string &url, const std::string &token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool Esp32Camera::Capture() {
    if (!streaming_on_) {
        return false;
    }

    // A single-buffer JPEG camera must only be acquired once per request.
    // A second immediate esp_camera_fb_get() is unreliable on ESP32-S3 after
    // Wi-Fi is associated and was the trigger for Wall-E's vision resets.
    if (current_fb_) {
        esp_camera_fb_return(current_fb_);
    }
    current_fb_ = esp_camera_fb_get();
    if (!current_fb_) {
        ESP_LOGE(TAG, "Camera capture failed");
        return false;
    }

    // Prepare encode buffer for RGB565 format (with optional byte swapping)
    if (current_fb_->format == PIXFORMAT_RGB565) {
        size_t pixel_count = current_fb_->width * current_fb_->height;
        size_t data_size = pixel_count * 2;

        // Allocate or reallocate encode buffer if needed
        if (encode_buf_size_ < data_size) {
            if (encode_buf_) {
                heap_caps_free(encode_buf_);
            }
            encode_buf_ = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (encode_buf_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate memory for encode buffer");
                encode_buf_size_ = 0;
                return false;
            }
            encode_buf_size_ = data_size;
        }

        // Copy data to encode buffer with optional byte swapping
        uint16_t *src = (uint16_t *)current_fb_->buf;
        uint16_t *dst = (uint16_t *)encode_buf_;
        if (swap_bytes_enabled_) {
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
        } else {
            memcpy(encode_buf_, current_fb_->buf, data_size);
        }

        // Allocate separate buffer for preview display
        uint8_t *preview_data = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (preview_data != nullptr) {
            memcpy(preview_data, encode_buf_, data_size);
            auto display = dynamic_cast<LvglDisplay *>(Board::GetInstance().GetDisplay());
            if (display != nullptr) {
                display->SetPreviewImage(std::make_unique<LvglAllocatedImage>(preview_data, data_size, current_fb_->width, current_fb_->height, current_fb_->width * 2, LV_COLOR_FORMAT_RGB565));
            } else {
                heap_caps_free(preview_data);
            }
        }
    } else if (current_fb_->format == PIXFORMAT_JPEG) {
        // JPEG format preview usually requires decoding, skip preview display for now, just log
        ESP_LOGW(TAG, "JPEG capture success, len=%zu, but not supported for preview", current_fb_->len);
    }

    ESP_LOGI(TAG, "Captured frame: %dx%d, len=%zu, format=%d",
             current_fb_->width, current_fb_->height, current_fb_->len, current_fb_->format);

    return true;
}

bool Esp32Camera::SetHMirror(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_hmirror(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetVFlip(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_vflip(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetSwapBytes(bool enabled) {
    swap_bytes_enabled_ = enabled;
    return true;
}

std::string Esp32Camera::Explain(const std::string &question) {
    if (explain_url_.empty()) {
        throw std::runtime_error("Image explain URL or token is not set");
    }

    if (current_fb_ == nullptr) {
        throw std::runtime_error("No camera frame captured");
    }

    // Always return the driver's single framebuffer, including on network or
    // encoder exceptions. Keeping it checked out can starve the next capture.
    struct FrameGuard {
        camera_fb_t*& frame;
        ~FrameGuard() {
            if (frame != nullptr) {
                esp_camera_fb_return(frame);
                frame = nullptr;
            }
        }
    } frame_guard{current_fb_};

    const uint16_t frame_width = current_fb_->width;
    const uint16_t frame_height = current_fb_->height;
    const pixformat_t frame_format = current_fb_->format;

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    http->SetTimeout(20000);
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";

    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        throw std::runtime_error("Failed to connect to explain URL");
    }

    bool upload_ok = true;
    {
        std::string question_field;
        question_field += "--" + boundary + "\r\n";
        question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
        question_field += "\r\n";
        question_field += question + "\r\n";
        upload_ok = http->Write(question_field.c_str(), question_field.size()) >= 0;
    }
    {
        std::string file_header;
        file_header += "--" + boundary + "\r\n";
        file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
        file_header += "Content-Type: image/jpeg\r\n";
        file_header += "\r\n";
        if (upload_ok && http->Write(file_header.c_str(), file_header.size()) < 0) {
            upload_ok = false;
        }
    }

    size_t total_sent = 0;
    if (frame_format == PIXFORMAT_JPEG) {
        // OV2640 already produced a complete JPEG. Sending that framebuffer
        // directly avoids a second encoder task, queue and full-image copy.
        if (upload_ok && http->Write(reinterpret_cast<const char*>(current_fb_->buf),
                                     current_fb_->len) >= 0) {
            total_sent = current_fb_->len;
        } else {
            upload_ok = false;
        }
    } else {
        v4l2_pix_fmt_t encoder_format;
        switch (frame_format) {
            case PIXFORMAT_RGB565: encoder_format = V4L2_PIX_FMT_RGB565; break;
            case PIXFORMAT_YUV422: encoder_format = V4L2_PIX_FMT_YUYV; break;
            case PIXFORMAT_YUV420: encoder_format = V4L2_PIX_FMT_YUV420; break;
            case PIXFORMAT_GRAYSCALE: encoder_format = V4L2_PIX_FMT_GREY; break;
            case PIXFORMAT_RGB888: encoder_format = V4L2_PIX_FMT_RGB24; break;
            default:
                http->Close();
                throw std::runtime_error("Unsupported camera pixel format");
        }

        uint8_t* source = current_fb_->buf;
        size_t source_length = current_fb_->len;
        if (frame_format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
            source = encode_buf_;
            source_length = encode_buf_size_;
        }

        struct UploadContext {
            Http* http;
            bool ok;
            size_t sent;
        } context{http.get(), upload_ok, 0};
        const bool encoded = image_to_jpeg_cb(
            source, source_length, frame_width, frame_height, encoder_format, 80,
            [](void* arg, size_t, const void* data, size_t len) -> size_t {
                auto* context = static_cast<UploadContext*>(arg);
                if (data != nullptr && len > 0 && context->ok) {
                    if (context->http->Write(static_cast<const char*>(data), len) < 0) {
                        context->ok = false;
                    } else {
                        context->sent += len;
                    }
                }
                return len;
            }, &context);
        upload_ok = encoded && context.ok;
        total_sent = context.sent;
    }

    if (!upload_ok || total_sent == 0) {
        http->Close();
        ESP_LOGE(TAG, "Camera JPEG upload failed, sent=%zu", total_sent);
        throw std::runtime_error("Failed to upload camera image");
    }

    {
        std::string multipart_footer;
        multipart_footer += "\r\n--" + boundary + "--\r\n";
        if (http->Write(multipart_footer.c_str(), multipart_footer.size()) < 0) {
            http->Close();
            throw std::runtime_error("Failed to finish camera upload");
        }
    }
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        throw std::runtime_error("Failed to upload photo");
    }

    std::string result = http->ReadAll();
    http->Close();

    size_t remain_stack_size = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "Explain image size=%dx%d, compressed size=%d, remain stack size=%d, question=%s\n%s",
             frame_width, frame_height, (int)total_sent, (int)remain_stack_size, question.c_str(), result.c_str());
    return result;
}
