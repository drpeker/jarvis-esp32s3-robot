// OV2640 photo on demand: the driver is started for the photo and stopped right after, so
// no camera DMA runs while Jarvis talks or drives. The sensor makes the JPEG itself.
#pragma once
#include <cstdint>
#include <vector>

#include <sensor.h>  // framesize_t (esp32-camera)

namespace camera {
constexpr framesize_t kPhotoSize = FRAMESIZE_XGA;  // 1024x768, chosen by the user
// JPEG bytes (PSRAM). false if the camera did not answer.
bool TakePhoto(std::vector<uint8_t>& jpeg, int* width = nullptr, int* height = nullptr,
               framesize_t size = kPhotoSize);
}  // namespace camera
