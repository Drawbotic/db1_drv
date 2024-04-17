#include "inc/bno085_drv.h"
#include "inc/drawbotic_utils.h"

#include <stdlib.h>
#include <memory.h>
#include <stdio.h>

//Private state
static bno085_evt_handler_t m_callback = 0;
static db1_hal_t *m_db1_hal = 0;
static uint8_t m_dev_addr = BNO085_DEFAULT_ADDR;
static db1_imu_settings_t m_current_settings;

//Forward Declarations
static int      db1_bno_open(sh2_Hal_t *hal);
static void     db1_bno_close(sh2_Hal_t *hal);
static int      db1_bno_read(sh2_Hal_t *hal, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int      db1_bno_write(sh2_Hal_t *hal, uint8_t *pBuffer, unsigned len);
static uint32_t db1_bno_get_time_us(sh2_Hal_t *hal);
static void     bno085_hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void     bno085_sensor_callback(void *cookie, sh2_SensorEvent_t *pEvent);

//SH2 HAL definition
static sh2_Hal_t m_bno_hal = {
    .open = db1_bno_open,
    .close = db1_bno_close,
    .read = db1_bno_read,
    .write = db1_bno_write,
    .getTimeUs = db1_bno_get_time_us,
};

//Private functions
bool bno085_enable_report(sh2_SensorId_t sensor, uint32_t interval_us)
{
    static sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.reportInterval_us = interval_us;

    int status = sh2_setSensorConfig(sensor, &config);
    if(status != SH2_OK)
        return false;

    return true;
}

bool bno085_configure_sig_motion_detection(float acc_threshold, uint32_t step_threshold)
{
    uint32_t frs_data[2];
    frs_data[0] = db1_float32_to_fixed32(acc_threshold, 24); //acc_threshold must be converted to fixed point, Q-point: 24.
    frs_data[1] = step_threshold;

    sh2_setFrs(SIG_MOTION_DETECT_CONFIG, frs_data, 2);

    return true;
}

bool bno085_set_reports() 
{
    if(!bno085_enable_report(SH2_ROTATION_VECTOR, m_current_settings.rotation_interval_us))
        return false;
    
    if(!bno085_enable_report(SH2_LINEAR_ACCELERATION, m_current_settings.accel_interval_us))
        return false;

    if(!bno085_configure_sig_motion_detection(m_current_settings.bump_acc_threshold, m_current_settings.bump_step_threshold))
       return false;

    if(!bno085_enable_report(SH2_SIGNIFICANT_MOTION, m_current_settings.accel_interval_us))
        return false;

    return true;
}

//Module definitions
bool bno085_init(uint8_t addr, db1_hal_t* hal, bno085_evt_handler_t callback, db1_imu_settings_t settings)
{
    if(callback == 0)
        return false;
    
    if(hal == 0)
        return false;
        
    m_callback = callback;
    m_db1_hal = hal;
    m_dev_addr = addr;
    m_current_settings = settings;

    m_db1_hal->set_pinmode(m_db1_hal->pins.imu_reset, DB1_HAL_PIN_OUT);

    return true;
}

bool bno085_begin()
{
    int status = sh2_open(&m_bno_hal, bno085_hal_callback, 0);
    if(status != SH2_OK)
        return false;

    sh2_ProductIds_t ids;
    memset(&ids, 0, sizeof(ids));
    status = sh2_getProdIds(&ids);
    if(status != SH2_OK)
        return false;

    sh2_setSensorCallback(bno085_sensor_callback, 0);

    return bno085_set_reports();
}

void bno085_hardware_reset()
{
    m_db1_hal->digital_write(m_db1_hal->pins.imu_reset, 1);
    m_db1_hal->delay_ms(10);
    m_db1_hal->digital_write(m_db1_hal->pins.imu_reset, 0);
    m_db1_hal->delay_ms(10);
    m_db1_hal->digital_write(m_db1_hal->pins.imu_reset, 1);
    m_db1_hal->delay_ms(10);
}

void bno085_service()
{
    sh2_service();
}

//<------- BNO DB1 Hal translation -------->
static int db1_bno_open(sh2_Hal_t *hal)
{
    return 0;
}

static void db1_bno_close(sh2_Hal_t *hal)
{
    //Nothing
}

static int db1_bno_read(sh2_Hal_t *hal, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    if(m_db1_hal == 0)
        return 0;

    uint8_t header[4];
    if(!m_db1_hal->i2c_read(m_dev_addr, header, 4))
        return 0;

    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    packet_size &= ~0x8000;

    if (packet_size > len)
        return 0;

    size_t i2c_buffer_max = m_db1_hal->i2c_buffer_size();
    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[i2c_buffer_max];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;

    while (cargo_remaining > 0) 
    {
        if (first_read) 
            read_size = db1_min(i2c_buffer_max, (size_t)cargo_remaining);
        else 
            read_size = db1_min(i2c_buffer_max, (size_t)cargo_remaining + 4);

        if(!m_db1_hal->i2c_read(m_dev_addr, i2c_buffer, read_size))
            return 0;

        if (first_read) 
        {
            cargo_read_amount = read_size;
            memcpy(pBuffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        } 
        else 
        {
            cargo_read_amount = read_size - 4;
            memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
        }
        pBuffer += cargo_read_amount;
        cargo_remaining -= cargo_read_amount;
    }

    return packet_size;
}

static int db1_bno_write(sh2_Hal_t *hal, uint8_t *pBuffer, unsigned len)
{
    if(m_db1_hal == 0)
        return 0;

    size_t i2c_buffer_max = m_db1_hal->i2c_buffer_size();
    uint16_t write_size = db1_min(i2c_buffer_max, len);

    if (!m_db1_hal->i2c_write(m_dev_addr, pBuffer, write_size))
        return 0;

    return write_size;
}

static uint32_t db1_bno_get_time_us(sh2_Hal_t *hal)
{
    return m_db1_hal->micros();
}

//<------- SH2 Callbacks -------->
static void bno085_hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    if(pEvent->eventId == SH2_RESET)
        bno085_set_reports();
}

static void bno085_sensor_callback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    sh2_SensorValue_t val;
    db1_vector_t v;
    db1_quaternion_t q;

    int status = sh2_decodeSensorEvent(&val, pEvent);
    if(status != SH2_OK)
        return;
    
    if(m_callback == 0)
        return;

    switch(val.sensorId)
    {
    case SH2_LINEAR_ACCELERATION:
        v.x = val.un.linearAcceleration.x,
        v.y = val.un.linearAcceleration.y,
        v.z = val.un.linearAcceleration.z,
        m_callback(BNO085_ACCEL_EVT, (void*)&v);
        break;
    
    case SH2_ROTATION_VECTOR:
        q.r = val.un.rotationVector.real,
        q.i = val.un.rotationVector.i,
        q.j = val.un.rotationVector.j,
        q.k = val.un.rotationVector.k,
        m_callback(BNO085_ROTATION_EVT, (void*)&q);
        break;

    case SH2_SIGNIFICANT_MOTION:
        bno085_enable_report(SH2_SIGNIFICANT_MOTION, m_current_settings.accel_interval_us); //Reenable the sig mot report
        m_callback(BNO085_BUMP_EVT, 0);
        break;
    }
}