#include "db1/inc/vl53l0x/vl53l0x_i2c_platform.h"

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count, db1_hal_t *hal)
{
    hal->i2c_reg_write(address, index, pdata, count);
    
    return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count, db1_hal_t *hal)
{
    hal->i2c_reg_read(address, index, pdata, count);

    return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t   data, db1_hal_t *hal)
{
    uint8_t b[1] = {data};
    return VL53L0X_write_multi(address, index, b, 1, hal);
}

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data, db1_hal_t *hal)
{
    uint8_t buff[2];
    buff[1] = data & 0xFF;
    buff[0] = data >> 8;

    return VL53L0X_write_multi(address, index, buff, 2, hal);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data, db1_hal_t *hal)
{
    uint8_t buff[4];
    buff[3] = data & 0xFF;
    buff[2] = data >> 8;
    buff[1] = data >> 16;
    buff[0] = data >> 24;

    return VL53L0X_write_multi(address, index, buff, 4, hal);
}

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata, db1_hal_t *hal)
{
    uint8_t buff[1];

    int r = VL53L0X_read_multi(address, index, buff, 1, hal);

    *pdata = buff[0];

    return r;
}

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata, db1_hal_t *hal)
{
    uint8_t buff[2];
    int r = VL53L0X_read_multi(address, index, buff, 2, hal);

    uint16_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];

    *pdata = tmp;

    return r;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata, db1_hal_t *hal)
{
    uint8_t buff[4];
    int r = VL53L0X_read_multi(address, index, buff, 4, hal);

    uint32_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    tmp <<= 8;
    tmp |= buff[2];
    tmp <<= 8;
    tmp |= buff[3];

    *pdata = tmp;

    return r;
}