#ifndef VEML6040_DRV_H
#define VEML6040_DRV_H

#include "drawbotic_types.h"
#include "drawbotic_hal.h"

#define VEML6040_ADDR         0x10
#define VEML6040_REG_CONF     0x00
#define VEML6040_REG_R_DATA   0x08
#define VEML6040_REG_G_DATA   0x09
#define VEML6040_REG_B_DATA   0x0A
#define VEML6040_REG_W_DATA   0x0B

#define VEML6040_TRIG_DISABLE 0x00
#define VEML6040_TRIG_ENABLE  0x04

#define VEML6040_AF_AUTO      0x00
#define VEML6040_AF_FORCE     0x02

#define VEML6040_SD_ENABLE    0x00
#define VEML6040_SD_DISABLE   0x01 

// G SENSITIVITY
#define VEML6040_GSENS_40MS   0.25168
#define VEML6040_GSENS_80MS   0.12584
#define VEML6040_GSENS_160MS  0.06292
#define VEML6040_GSENS_320MS  0.03146
#define VEML6040_GSENS_640MS  0.01573
#define VEML6040_GSENS_1280MS 0.007865

bool veml6040_init(db1_hal_t *hal);
void veml6040_set_config(veml6040_int_time_t int_time, bool force, bool trig, bool disabled);
uint16_t veml6040_current_integration_time();
db1_colour_reading_t veml6040_read_colour();
float veml6040_calc_cct(float offset);
float veml6040_ambient_lux();
uint16_t veml6040_integration_time_to_ms(veml6040_int_time_t int_time);

#endif