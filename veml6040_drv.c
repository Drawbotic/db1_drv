#include "inc/veml6040_drv.h"

#include <math.h>

static db1_hal_t *m_db1_hal = 0;
static veml6040_int_time_t m_current_int_time = VEML6040_INT_40MS;

bool veml6040_init(db1_hal_t *hal)
{
    m_db1_hal = hal;
    return m_db1_hal->i2c_write(VEML6040_ADDR, 0, 0);
}

void veml6040_set_config(veml6040_int_time_t int_time, bool force, bool trig, bool disabled)
{
    uint8_t sensor_enabled = disabled ? VEML6040_SD_DISABLE : VEML6040_SD_ENABLE;
    uint8_t mode = force ? VEML6040_AF_FORCE : VEML6040_AF_AUTO;
    uint8_t trigger = trig ? VEML6040_TRIG_ENABLE : VEML6040_TRIG_DISABLE;

    uint16_t conf = (uint16_t)int_time + sensor_enabled + mode + trigger;

    uint8_t write_buffer[2];
    write_buffer[0] = 0x00;
    write_buffer[1] = 0x00;
    m_db1_hal->i2c_reg_write(VEML6040_ADDR, VEML6040_REG_CONF, write_buffer, 2);

    m_current_int_time = int_time;
}

uint16_t veml6040_current_integration_time()
{
    return veml6040_integration_time_to_ms(m_current_int_time);
}

db1_colour_reading_t veml6040_read_colour()
{
    uint16_t r, g, b, w;
    uint16_t read_buffer[4];

    r = m_db1_hal->i2c_reg_read_16(VEML6040_ADDR, VEML6040_REG_R_DATA);
    g = m_db1_hal->i2c_reg_read_16(VEML6040_ADDR, VEML6040_REG_G_DATA);
    b = m_db1_hal->i2c_reg_read_16(VEML6040_ADDR, VEML6040_REG_B_DATA);
    w = m_db1_hal->i2c_reg_read_16(VEML6040_ADDR, VEML6040_REG_W_DATA);

    db1_colour_reading_t result = { r, g, b, w };

    return result;
}

float veml6040_calc_cct(float offset)
{
    db1_colour_reading_t reading = veml6040_read_colour();

    float ccti;

    ccti = (float)reading.r - (float)reading.b / (float)reading.g;
    ccti += offset;

    return 4278.6 * pow(ccti, -1.2455);
}

float veml6040_ambient_lux()
{
    uint16_t green;
    float lux;

    green = veml6040_read_colour().g;

    switch(m_current_int_time) 
    {
    case VEML6040_INT_40MS:
        lux = green * VEML6040_GSENS_40MS;
        break;
    case VEML6040_INT_80MS:
        lux = green * VEML6040_GSENS_80MS;
        break;
    case VEML6040_INT_160MS:
        lux = green * VEML6040_GSENS_160MS;
        break;
    case VEML6040_INT_320MS:
        lux = green * VEML6040_GSENS_320MS;
        break;
    case VEML6040_INT_640MS:
        lux = green * VEML6040_GSENS_640MS;
        break; 
    case VEML6040_INT_1280MS:
        lux = green * VEML6040_GSENS_1280MS; 
        break;
    default:
        lux = -1;
        break;
    }

    return lux;
}

uint16_t veml6040_integration_time_to_ms(veml6040_int_time_t int_time) {
    switch(int_time) 
    {
    case VEML6040_INT_40MS:
        return 40;
    case VEML6040_INT_80MS:
        return 80;
    case VEML6040_INT_160MS:
        return 160;
    case VEML6040_INT_320MS:
        return 320;
    case VEML6040_INT_640MS:
        return 640;
    case VEML6040_INT_1280MS:
        return 1280;
    default:
        return 1;
    }
}