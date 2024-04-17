#ifndef VL53L0X_DRV_H
#define VL53L0X_DRV_H

#include <stdint.h>

typedef void (*db1_hal_i2c_read_t)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
typedef void (*db1_hal_i2c_write_t)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

typedef struct
{

} vl53l0x_hal_t;

typedef struct
{
    uint8_t addr;
    uint32_t en_pin;
    uint32_t int_pin;
} vl53l0x_device_t;

bool vl553l0x_init(vl53l0x_device_t *dev);

#endif