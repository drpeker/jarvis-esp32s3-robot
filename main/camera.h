// OV2640 photo on demand: the driver is started for the photo and stopped right after, so
// no camera DMA runs while Jarvis talks or drives. The sensor makes the JPEG itself.
#pragma once
#include <cstdint>
#include <vector>

namespace camera {
// JPEG bytes (PSRAM). false if the camera did not answer.
bool TakePhoto(std::vector<uint8_t>& jpeg, int* width = nullptr, int* height = nullptr);
}  // namespace camera
