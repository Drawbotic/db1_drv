#ifndef DRAWBOTIC_TYPES_H
#define DRAWBOTIC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define DB1_MOTOR_COUNT         2
//! Number of RGB LEDs on board
#define DB1_LIGHT_COUNT         8
//! Number of Time of Flight sensor on board
#define DB1_TOF_COUNT           3
//! Number of IR line detectors on board
#define DB1_IR_COUNT            5

//! Delay time for Time of Flight sensors to boot
#define DB1_TOF_BOOT_DELAY_MS   20

//! Number of IR sensor samples during calibration
#define DB1_IR_CALIBRATION_COUNT    1000
//! Delay between IR calibration samples
#define DB1_IR_CALIBRATION_DELAY_MS 10

//! Number of Colour sensor samples during calibration 
#define DB1_COLOUR_CALIBRATION_COUNT    5
//! Delay between Colour calibration samples
#define DB1_COLOUR_CALIBRATION_DELAY_MS 500

//! Default Servo Up position
#define DB1_SERVO_UP_DEFAULT    25
//! Default Servo Down position
#define DB1_SERVO_DOWN_DEFAULT  90

//! Default Timeout of Time of Flight Sensors
#define DB1_TOF_RATE_DEFAULT            500
//! Default Signal Limit of Time of Flight Sensors
#define DB1_TOF_SIGLIM_DEFAULT          0.25f
//! Default Timing Budget of Time of Flight Sensors
#define DB1_TOF_TIMING_BUDGET_DEFAULT   33000
//! Default Pre Pulse Period of Time of Flight Sensors
#define DB1_TOF_PRE_PCLKS_DEFAULT       14
//! Default Final Pulse Period of Time of Flight Sensors
#define DB1_TOF_FIN_PCLKS_DEFAULT       10

//! Default IR Average Window Size
#define DB1_IR_READ_COUNT_DEFAULT       4

//! Voltage for Low Battery level
#define DB1_BATT_VOLT_LOW               6.0f
//! Voltage for High Battery level
#define DB1_BATT_VOLT_HIGH              8.4f

//! Macro for Default settings
#define DB1_DEFAULT_SETTINGS                        \
{                                                   \
    .pen = {                                        \
        .pen_up_pos = DB1_SERVO_UP_DEFAULT,         \
        .pen_down_pos = DB1_SERVO_DOWN_DEFAULT,     \
    },                                              \
    .tofs = {                                       \
        VL53L0X_SENSE_LONG_RANGE,                   \
        VL53L0X_SENSE_LONG_RANGE,                   \
        VL53L0X_SENSE_LONG_RANGE,                   \
    },                                              \
    .imu = {                                        \
        .rotation_interval_us = 10000,              \
        .accel_interval_us = 10000,                 \
        .bump_acc_threshold = 8.0f,                 \
        .bump_step_threshold = 2,                   \
    },                                              \
    .colour_int_time = VEML6040_INT_40MS,           \
    .use_encoders = true,                           \
    .ir_window_size = DB1_COLOUR_CALIBRATION_COUNT, \
}

typedef enum
{
    VEML6040_INT_40MS   = 0x00,
    VEML6040_INT_80MS   = 0x10,
    VEML6040_INT_160MS  = 0x20,
    VEML6040_INT_320MS  = 0x30,
    VEML6040_INT_640MS  = 0x40,
    VEML6040_INT_1280MS = 0x50,
}
veml6040_int_time_t;

typedef enum {
    VL53L0X_SENSE_DEFAULT = 0,
    VL53L0X_SENSE_LONG_RANGE,
    VL53L0X_SENSE_HIGH_SPEED,
    VL53L0X_SENSE_HIGH_ACCURACY
} vl53l0x_config_t;

typedef enum
{
    DB1_SUCCESS           =  0,
    DB1_ERR_HAL           = -1,
    DB1_ERR_IMU           = -2,
    DB1_ERR_TOF           = -3,
    DB1_ERR_TOF_DATA_INIT = -4,
    DB1_ERR_TOF_INIT      = -5,
    DB1_ERR_TOF_ADDRESS   = -6,
    DB1_ERR_TOF_CALI      = -7,
    DB1_ERR_TOF_SPAD      = -8,
    DB1_ERR_TOF_SET_MODE  = -9,
    DB1_ERR_TOF_INFO      = -10,
    DB1_ERR_TOF_SETTINGS  = -11,
    DB1_ERR_COLOUR        = -12,
}
db1_error_t;

typedef enum
{
    DB1_TOF_LEFT   = 0,
    DB1_TOF_CENTRE = 1,
    DB1_TOF_RIGHT  = 2,
}
db1_tof_location_t;

typedef enum
{
    DB1_M_1 = 0,
    DB1_M_2 = 1,
}
db1_motor_t;

typedef struct
{
    double pen_up_pos;
    double pen_down_pos;
}
db1_servo_settings_t;

typedef struct
{
    vl53l0x_config_t ranging_type;
    uint32_t continuous_rate;
    uint32_t measurement_timing_budget;
    uint8_t pre_pclks;
    uint8_t final_pclks;
}
db1_tof_settings_t;

typedef struct
{
    uint32_t rotation_interval_us;
    uint32_t accel_interval_us;
    float bump_acc_threshold;
    uint32_t bump_step_threshold;
}
db1_imu_settings_t;

typedef struct
{
    db1_servo_settings_t pen;
    vl53l0x_config_t tofs[DB1_TOF_COUNT];
    db1_imu_settings_t imu;
    veml6040_int_time_t colour_int_time;
    bool use_encoders;
    uint8_t ir_window_size;
}
db1_settings_t;

typedef struct
{
    uint32_t far_left;
    uint32_t left;
    uint32_t centre;
    uint32_t right;
    uint32_t far_right;
}
db1_ir_array_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}
db1_colour_t;

typedef struct
{
    uint32_t r;
    uint32_t g;
    uint32_t b;
    uint32_t w;
}
db1_colour_reading_t;

typedef struct
{
    db1_colour_t led[DB1_LIGHT_COUNT];
}
db1_lights_t;

typedef struct
{
    float r;
    float i;
    float j;
    float k;
}
db1_quaternion_t;

typedef struct
{
    float x;
    float y;
    float z;
}
db1_vector_t;

typedef struct
{
    float heading;
    float pitch;
    float roll;
}
db1_orientation_t;

#endif