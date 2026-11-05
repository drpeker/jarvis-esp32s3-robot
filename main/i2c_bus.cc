#include "i2c_bus.h"

#include <esp_log.h>

#include <initializer_list>

#include "pins.h"

namespace {
i2c_master_bus_handle_t g_bus = nullptr;
}

namespace i2c_bus {

void Init() {
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = static_cast<i2c_port_t>(I2C_PORT);
    cfg.sda_io_num = PIN_I2C_SDA;
    cfg.scl_io_num = PIN_I2C_SCL;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = 1;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &g_bus));
    for (uint8_t a : {OV2640_ADDR, INA219_ADDR, MPU6050_ADDR}) {
        ESP_LOGI("i2c", "0x%02x %s", a, i2c_master_probe(g_bus, a, 50) == ESP_OK ? "found" : "missing");
    }
}

i2c_master_bus_handle_t Handle() { return g_bus; }

i2c_master_dev_handle_t AddDevice(uint8_t address) {
    if (i2c_master_probe(g_bus, address, 50) != ESP_OK) return nullptr;
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = address;
    cfg.scl_speed_hz = I2C_HZ;
    i2c_master_dev_handle_t dev = nullptr;
    i2c_master_bus_add_device(g_bus, &cfg, &dev);
    return dev;
}

}  // namespace i2c_bus
