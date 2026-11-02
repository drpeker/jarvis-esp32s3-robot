#include "ina219.h"

#include <esp_log.h>

#define TAG "Ina219"

namespace {
constexpr uint8_t kRegConfig = 0x00;
constexpr uint8_t kRegShuntVoltage = 0x01;
constexpr uint8_t kRegBusVoltage = 0x02;
// 32V bus range, PGA /8 (+-320mV), 128-sample averaging (~68ms, smooths motor PWM),
// continuous -- same as NeuroVision-Gemini.
constexpr uint16_t kConfigValue = 0x3FFF;
constexpr int kTimeoutMs = 50;
}  // namespace

Ina219::Ina219(i2c_master_bus_handle_t bus, uint8_t address, uint32_t shunt_milliohms)
    : shunt_milliohms_(shunt_milliohms) {
    if (i2c_master_probe(bus, address, kTimeoutMs) != ESP_OK) {
        ESP_LOGW(TAG, "INA219 not found at 0x%02X; power readings disabled", address);
        return;
    }

    i2c_device_config_t config = {};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &config, &device_) != ESP_OK) {
        device_ = nullptr;
        ESP_LOGE(TAG, "Failed to add INA219 device");
        return;
    }
    if (!WriteRegister(kRegConfig, kConfigValue)) {
        ESP_LOGW(TAG, "INA219 config write failed; using power-on defaults");
    }
    ESP_LOGI(TAG, "INA219 ready at 0x%02X (shunt %lu mOhm)", address,
             static_cast<unsigned long>(shunt_milliohms_));
}

Ina219::~Ina219() {
    if (device_ != nullptr) {
        i2c_master_bus_rm_device(device_);
    }
}

bool Ina219::Read(int& bus_mv, int& current_ma, int& power_mw) {
    if (device_ == nullptr) {
        return false;
    }
    uint16_t raw_bus = 0;
    uint16_t raw_shunt = 0;
    if (!ReadRegister(kRegBusVoltage, raw_bus) || !ReadRegister(kRegShuntVoltage, raw_shunt)) {
        return false;
    }
    // Bus voltage: bits 15..3, LSB 4mV. Shunt voltage: signed, LSB 10uV.
    bus_mv = static_cast<int>(raw_bus >> 3) * 4;
    const int shunt_uv = static_cast<int16_t>(raw_shunt) * 10;
    current_ma = shunt_milliohms_ == 0 ? 0 : shunt_uv / static_cast<int>(shunt_milliohms_);
    power_mw = bus_mv * current_ma / 1000;
    return true;
}

bool Ina219::ReadRegister(uint8_t reg, uint16_t& value) {
    uint8_t buffer[2] = {};
    if (i2c_master_transmit_receive(device_, &reg, 1, buffer, 2, kTimeoutMs) != ESP_OK) {
        return false;
    }
    value = static_cast<uint16_t>(buffer[0] << 8 | buffer[1]);
    return true;
}

bool Ina219::WriteRegister(uint8_t reg, uint16_t value) {
    const uint8_t buffer[3] = {reg, static_cast<uint8_t>(value >> 8),
                               static_cast<uint8_t>(value & 0xFF)};
    return i2c_master_transmit(device_, buffer, 3, kTimeoutMs) == ESP_OK;
}
