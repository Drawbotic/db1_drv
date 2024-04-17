#ifndef VL53L0X_DRV_H
#define VL53L0X_DRV_H

#include "drawbotic_types.h"
#include "drawbotic_hal.h"

db1_error_t vl53l0x_init(db1_hal_t *hal);
db1_error_t vl53l0x_configure(db1_tof_location_t location, vl53l0x_config_t config);
int vl53l0x_range(db1_tof_location_t location);

#endif