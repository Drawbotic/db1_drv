#ifndef DRAWBOTIC_TYPES_H
#define DRAWBOTIC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

//! Number of Motors on board
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

/*!
 * \brief Enum describing the Colour sensor integration times
 */
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

/*!
 * \brief Enum describing the Time of Flight operation modes
 */
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

/*!
 * \brief Enum describing the Time of Flight sensor locations
 */
typedef enum
{
    DB1_TOF_LEFT   = 0,
    DB1_TOF_CENTRE = 1,
    DB1_TOF_RIGHT  = 2,
}
db1_tof_location_t;

/*!
 * \brief Enum describing the motor positions
 */
typedef enum
{
    DB1_M1 = 0,
    DB1_M2 = 1,
}
db1_motor_t;

/*!
 * \brief A struct containing the settings for the Pen Lift Servo
 */
typedef struct
{
    double pen_up_pos; //!< The servo position for when the pen is up
    double pen_down_pos; //!< The servo position for when the pen is down
}
db1_servo_settings_t;

/*!
 * \brief A struct containing the settings for the VL53L0X Time of Flight sensors
 */
typedef struct
{
    vl53l0x_config_t ranging_type; //!< The ranging operation mode for the sensor
    uint32_t continuous_rate; //!< The sample rate of the sensor in continuous mode
    uint32_t measurement_timing_budget; //!< The total measurement timing budget in microseconds, this is overall time each measurement takes.
    uint8_t pre_pclks; //!< The Pre VCSEL Pulse Period
    uint8_t final_pclks; //!< The Final VCSEL Pulse Period
}
db1_tof_settings_t;

/*!
 * \brief A struct containing the settings for the BNO085 IMU
 */
typedef struct
{
    uint32_t rotation_interval_us; //!< The sample rate of the orientation in microseconds
    uint32_t accel_interval_us; //!< The sample rate of the acceleration in microseconds
    float bump_acc_threshold; //!< The acceleration threshold that must be past to trigger a bump in m/s
    uint32_t bump_step_threshold; //!< The number of steps (measurements) that must be over the threshold to trigger a bump
}
db1_imu_settings_t;

/*!
 * \brief A struct containing all of the settings need to initialise DB1
 */
typedef struct
{
    db1_servo_settings_t pen; //!< The Pen Servo settings, see db1_servo_settings_t
    vl53l0x_config_t tofs[DB1_TOF_COUNT]; //!< The ranging type for each Time of FLight sensor, see vl53l0x_config_t
    db1_imu_settings_t imu; //!< The IMU settings, see db1_imu_settings_t
    veml6040_int_time_t colour_int_time; //!< The integration time for the colour sensor, see veml6040_int_time_t
    bool use_encoders;  //!< Sets if the encoders should be enabled
    uint8_t ir_window_size; //!< Sets the sliding average window size for reading the IR line detectors, higher values increase accuracy but changes take longer to appear
}
db1_settings_t;

/*!
 * \brief A struct used to represent the state of the IR line detectors
 */
typedef struct
{
    uint32_t far_left; //!< The value of the far left IR line detector
    uint32_t left; //!< The value of the left IR line detector
    uint32_t centre; //!< The value of the centre IR line detector
    uint32_t right; //!< The value of the right IR line detector
    uint32_t far_right; //!< The value of the far right IR line detector
}
db1_ir_array_t;

/*!
 * \brief A struct to represent a colour for the RGB LEDs
 */
typedef struct
{
    uint8_t r; //!< The red component
    uint8_t g; //!< The green component
    uint8_t b; //!< The blue component
}
db1_colour_t;

/*!
 * \brief A struct to represent a colour read from the colour sensor
 */
typedef struct
{
    uint32_t r; //!< The red component
    uint32_t g; //!< The green component
    uint32_t b; //!< The blue component
    uint32_t w; //!< The white component
}
db1_colour_reading_t;

/*!
 * \brief A struct containing the RGB colours for all of the RGB LEDs on board
 */
typedef struct
{
    db1_colour_t led[DB1_LIGHT_COUNT]; //!< An array containing the colour value for each light
}
db1_lights_t;

/*!
 * \brief A 4D quaternion stuct used to represent DB1 orientation
 */
typedef struct
{
    float r; //!< The real component
    float i; //!< The i component
    float j; //!< The j component
    float k; //!< The k component
}
db1_quaternion_t;

/*!
 * \brief A 3D vector struct used to represent DB1 acceleration
 */
typedef struct
{
    float x; //!< The x component
    float y; //!< The y component
    float z; //!< The z component
}
db1_vector_t;

/*!
 * \brief A struct used to represent the Euler Orientation of the DB1
 */
typedef struct
{
    float heading; //!< The heading (Z) component
    float pitch; //!< The pitch (X) component
    float roll; //!< The roll (Y) component
}
db1_orientation_t;

#endif