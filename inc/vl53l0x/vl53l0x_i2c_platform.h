/*
 * COPYRIGHT (C) STMicroelectronics 2014. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/**
 * @file   VL53L0X_platform.h
 * @brief  Function prototype definitions for Ewok Platform layer.
 *
 */


#ifndef _VL53L0X_I2C_PLATFORM_H_
#define _VL53L0X_I2C_PLATFORM_H_

#include "vl53l0x_def.h"
#include "../drawbotic_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Include uint8_t, unit16_t  etc definitions

#include <stdint.h>
#include <stdarg.h>

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count, db1_hal_t *hal);

int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count, db1_hal_t *hal);

int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t   data, db1_hal_t *hal);

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data, db1_hal_t *hal);

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data, db1_hal_t *hal);

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata, db1_hal_t *hal);

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata, db1_hal_t *hal);

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata, db1_hal_t *hal);


#ifdef __cplusplus
}
#endif

#endif //_VL53L0X_I2C_PLATFORM_H_

