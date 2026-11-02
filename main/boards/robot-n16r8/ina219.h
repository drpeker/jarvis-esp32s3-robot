#ifndef _ROBOT_N16R8_INA219_H_
#define _ROBOT_N16R8_INA219_H_

#include <cstdint>

#include <driver/i2c_master.h>

// Minimal INA219 reader. Current is computed from the raw shunt voltage, so the
// calibration register is not needed. Every call tolerates a missing sensor.
class Ina219 {
public:
    Ina219(i2c_master_bus_handle_t bus, uint8_t address, uint32_t shunt_milliohms);
    ~Ina219();

    bool IsPresent() const { return device_ != nullptr; }
    // bus_mv: voltage on VIN- (the load side). current_ma: positive into the load.
    bool Read(int& bus_mv, int& current_ma, int& power_mw);

private:
    i2c_master_dev_handle_t device_ = nullptr;
    uint32_t shunt_milliohms_;

    bool ReadRegister(uint8_t reg, uint16_t& value);
    bool WriteRegister(uint8_t reg, uint16_t value);
};

#endif  // _ROBOT_N16R8_INA219_H_
