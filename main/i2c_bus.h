// The one shared I2C bus (GPIO4/5): OV2640 SCCB, INA219, MPU-6050. Created once here;
// the i2c_master driver serialises access. Devices are added by their owners.
#pragma once
#include <driver/i2c_master.h>

namespace i2c_bus {
void Init();
i2c_master_bus_handle_t Handle();
i2c_master_dev_handle_t AddDevice(uint8_t address);  // nullptr if it does not answer
}  // namespace i2c_bus
