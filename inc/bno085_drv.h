#ifndef BNO085_DRV_H
#define BNO085_DRV_H

#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"

#include "drawbotic_hal.h"
#include "drawbotic_types.h"

#define BNO085_DEFAULT_ADDR 0x4A

typedef enum 
{
    BNO085_ROTATION_EVT,
    BNO085_ACCEL_EVT,
    BNO085_BUMP_EVT,
}
bno085_evt_t;

typedef void (*bno085_evt_handler_t)(bno085_evt_t evt, void *p_data);

bool bno085_init(uint8_t addr, db1_hal_t *hal, bno085_evt_handler_t callback, db1_imu_settings_t settings);
bool bno085_begin();
void bno085_hardware_reset();

void bno085_service();

#endif