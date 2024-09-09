#ifndef DRAWBOTIC_DB1_DRV_H
#define DRAWBOTIC_DB1_DRV_H

#ifdef __cplusplus 
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "drawbotic_hal.h"
#include "drawbotic_types.h"

db1_error_t db1_init(db1_hal_t *hal, db1_settings_t settings);
db1_error_t db1_init_motors(bool use_encoders);
db1_error_t db1_init_servo(db1_servo_settings_t settings);
db1_error_t db1_init_tof(db1_tof_location_t index, vl53l0x_config_t config);
db1_error_t db1_init_colour_sensor(veml6040_int_time_t int_time);
db1_error_t db1_init_imu(db1_imu_settings_t settings);

db1_settings_t db1_get_current_settings();

//IR array calibration functions
void           db1_calibrate_ir_array(uint8_t analogResolution);
void           db1_set_ir_calibration(db1_ir_array_t low, db1_ir_array_t high);
db1_ir_array_t db1_get_ir_high();
db1_ir_array_t db1_get_ir_low();

//Colour calibration functions
void                 db1_calibrate_colour();
void                 db1_set_colour_calibration(db1_colour_reading_t low, db1_colour_reading_t high);
db1_colour_reading_t db1_get_colour_high();
db1_colour_reading_t db1_get_colour_low();

//Lights
void         db1_set_white_led(bool on);
void         db1_set_lights(db1_lights_t lights);
void         db1_set_top_light(db1_colour_t colour);
db1_lights_t db1_get_lights();
db1_colour_t db1_get_top_light();

//Battery monitor
float db1_update_battery(bool lights);

//Pen control
void db1_set_pen(bool down);
void db1_set_pen_pos(float pos);

//Motor control
void db1_set_motor_speed(db1_motor_t motor, float speed, uint8_t analogResolution);
long db1_encoder_val(db1_motor_t motor);
long db1_encoder_delta(db1_motor_t motor);
void db1_reset_encoder_deltas();

//Sensor access
db1_colour_reading_t db1_read_colour(bool calibrated);
db1_ir_array_t       db1_read_ir(bool calibrated);
int                  db1_read_tof(db1_tof_location_t tof);
db1_quaternion_t     db1_read_rotation();
db1_orientation_t    db1_read_orientation();
db1_vector_t         db1_read_acceleration();
bool                 db1_check_bump();

void db1_delay_ms(uint32_t ms);
long db1_millis();

#ifdef __cplusplus
}
#endif

#endif