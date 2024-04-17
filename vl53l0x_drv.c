#include "inc/vl53l0x_drv.h"
#include "inc/vl53l0x/vl53l0x_api.h"

#define VL53L0X_I2C_ADDR 0x29

//private constants
//! I2C Addresses of Time of Flight sensors, set during initialisation
const int DB1_TOF_ADDRESSES[DB1_TOF_COUNT] = { 0x1C, 0x1B, 0x1A };
//! Enable pins for Time of Flight sensors, defined in DB1 board package
int DB1_TOF_EN_PINS[DB1_TOF_COUNT] = { -1, -1, -1 };

static db1_hal_t *m_db1_hal = 0;
static VL53L0X_Dev_t m_tofs[DB1_TOF_COUNT];
static VL53L0X_DeviceInfo_t m_infos[DB1_TOF_COUNT];

db1_error_t vl53l0x_set_address(VL53L0X_Dev_t *dev, uint8_t addr) 
{
    addr &= 0x7F;

    VL53L0X_Error err = VL53L0X_SetDeviceAddress(dev, addr * 2); // 7->8 bit

    m_db1_hal->delay_ms(10);

    if (err == VL53L0X_ERROR_NONE) {
        dev->I2cDevAddr = addr; // 7 bit addr
        return DB1_SUCCESS;
    }
    return DB1_ERR_TOF_ADDRESS;
}

db1_error_t vl53l0x_init(db1_hal_t *hal)
{
    VL53L0X_Error err;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    m_db1_hal = hal;
    DB1_TOF_EN_PINS[0] = m_db1_hal->pins.tof3_en;
    DB1_TOF_EN_PINS[1] = m_db1_hal->pins.tof2_en;
    DB1_TOF_EN_PINS[2] = m_db1_hal->pins.tof1_en;

    for(int i = 0; i < DB1_TOF_COUNT; i++)
    {
        m_tofs[i].I2cDevAddr = VL53L0X_I2C_ADDR;
        m_tofs[i].db1_hal = m_db1_hal;
        //Drive all en pins low, turn off all sensors
        m_db1_hal->set_pinmode(DB1_TOF_EN_PINS[i], DB1_HAL_PIN_OUT);
        m_db1_hal->digital_write(DB1_TOF_EN_PINS[i], 0);
    }
    m_db1_hal->delay_ms(50);

    for(int i = 0; i < DB1_TOF_COUNT; i++)
    {
        m_db1_hal->digital_write(DB1_TOF_EN_PINS[i], 1);
        m_db1_hal->delay_ms(50);

        if(VL53L0X_DataInit(&m_tofs[i]) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_DATA_INIT;

        if(vl53l0x_set_address(&m_tofs[i], DB1_TOF_ADDRESSES[i]) != DB1_SUCCESS)
            return DB1_ERR_TOF_ADDRESS;

        if(VL53L0X_GetDeviceInfo(&m_tofs[i], &m_infos[i]) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_INFO;

        if(VL53L0X_StaticInit(&m_tofs[i]) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_INIT;

        if(VL53L0X_PerformRefCalibration(&m_tofs[i], &VhvSettings, &PhaseCal) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_CALI;

        if(VL53L0X_SetDeviceMode(&m_tofs[i], VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SET_MODE;
    }
    return DB1_SUCCESS;
}

db1_error_t vl53l0x_configure(db1_tof_location_t location, vl53l0x_config_t config)
{
    VL53L0X_Error err;

    if(location < 0 || location >= DB1_TOF_COUNT)
        return DB1_ERR_TOF;
    
    if(VL53L0X_SetLimitCheckEnable(&m_tofs[location], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1) != VL53L0X_ERROR_NONE)
        return DB1_ERR_TOF_SETTINGS;
    
    if(VL53L0X_SetLimitCheckEnable(&m_tofs[location], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1) != VL53L0X_ERROR_NONE)
        return DB1_ERR_TOF_SETTINGS;

    switch(config)
    {
    case VL53L0X_SENSE_DEFAULT:
        if(VL53L0X_SetLimitCheckEnable(&m_tofs[location], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;
        
        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5 * 0.023 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;
        break;

    case VL53L0X_SENSE_LONG_RANGE:
        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_tofs[location], 33000) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetVcselPulsePeriod(&m_tofs[location], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetVcselPulsePeriod(&m_tofs[location], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;
        break;

    case VL53L0X_SENSE_HIGH_SPEED:
        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_tofs[location], 30000) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;
        break;
    case VL53L0X_SENSE_HIGH_ACCURACY:
        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetLimitCheckValue(&m_tofs[location], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536)) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_tofs[location], 200000) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS;

        if(VL53L0X_SetLimitCheckEnable(&m_tofs[location], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0) != VL53L0X_ERROR_NONE)
            return DB1_ERR_TOF_SETTINGS; 
        break;
    }
    
    if(VL53L0X_SetDeviceMode(&m_tofs[location], VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING) != VL53L0X_ERROR_NONE)
        return DB1_ERR_TOF_SETTINGS;

    if(VL53L0X_SetInterMeasurementPeriodMilliSeconds(&m_tofs[location], 20) != VL53L0X_ERROR_NONE)
        return DB1_ERR_TOF_SETTINGS;

    if(VL53L0X_StartMeasurement(&m_tofs[location]) != VL53L0X_ERROR_NONE)
        return DB1_ERR_TOF_SETTINGS;

    return DB1_SUCCESS;
}

int vl53l0x_range(db1_tof_location_t location)
{
    VL53L0X_RangingMeasurementData_t data;

    if(location < 0 || location >= DB1_TOF_COUNT)
        return 0;

    if(VL53L0X_GetRangingMeasurementData(&m_tofs[location], &data) != VL53L0X_ERROR_NONE)
        return 0;

    return data.RangeMilliMeter;
}