#ifndef DRAWBOTIC_HAL_H
#define DRAWBOTIC_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    DB1_HAL_PIN_IN,
    DB1_HAL_PIN_IN_PULLUP,
    DB1_HAL_PIN_IN_ANALOG,
    DB1_HAL_PIN_OUT,
    DB1_HAL_PIN_OUT_PULLUP,
    DB1_HAL_PIN_OUT_PWM,
    DB1_HAL_PIN_OUT_SERVO,
} db1_hal_pinmode_t;

typedef enum
{
    DB1_HAL_INT_RISING,
    DB1_HAL_INT_FALLING,
    DB1_HAL_INT_CHANGE,
} db1_hal_interrupt_type_t;

typedef void (*db1_hal_int_callback_t)();

typedef void     (*db1_hal_set_pinmode_t)(uint8_t pin, db1_hal_pinmode_t mode);
typedef uint8_t  (*db1_hal_read_pin_t)(uint8_t pin);
typedef void     (*db1_hal_set_pin_t)(uint8_t pin, uint8_t val);
typedef uint16_t (*db1_hal_analog_read_t)(uint8_t pin);
typedef void     (*db1_hal_analog_res_t)(uint8_t res);
typedef void     (*db1_hal_set_pin_pwm_t)(uint8_t pin, uint16_t duty);
typedef void     (*db1_hal_attach_int_t)(uint8_t pin, db1_hal_interrupt_type_t type, db1_hal_int_callback_t handler);
typedef void     (*db1_hal_detach_int_t)(uint8_t pin);
typedef bool     (*db1_hal_i2c_t)(uint8_t addr, uint8_t *data, uint8_t len);
typedef bool     (*db1_hal_i2c_reg_t)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
typedef uint16_t (*db1_i2c16_t)(uint8_t addr, uint8_t reg);
typedef uint32_t (*db1_hal_i2c_size_t)();
typedef void     (*db1_hal_servo_write_t)(uint8_t pin, uint8_t value);
typedef void     (*db1_hal_rgb_init_t)(uint8_t pin, uint8_t count);
typedef void     (*db1_hal_rgb_set_t)(uint8_t pin, uint16_t index, uint8_t r, uint8_t g, uint8_t b);
typedef void     (*db1_hal_rgb_fill_t)(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
typedef void     (*db1_hal_delay_ms_t)(uint32_t ms);
typedef uint32_t (*db1_hal_counter_t)();

typedef struct
{
    uint8_t m1_dir_a;
    uint8_t m1_dir_b;
    uint8_t m2_dir_a;
    uint8_t m2_dir_b;
    uint8_t stat_dout;
    uint8_t rgb_dout;
    uint8_t m1_e_a;
    uint8_t m1_e_b;
    uint8_t m2_e_a;
    uint8_t m2_e_b;
    uint8_t led_en;
    uint8_t tof1_int;
    uint8_t tof2_int;
    uint8_t tof3_int;
    uint8_t tof1_en;
    uint8_t tof2_en;
    uint8_t tof3_en;
    uint8_t batt_lvl1;
    uint8_t batt_lvl2;
    uint8_t batt_lvl3;
    uint8_t batt_lvl4;
    uint8_t host_int_1;
    uint8_t host_int_2;
    uint8_t imu_int;
    uint8_t imu_reset;
    uint8_t m1_pwm;
    uint8_t m2_pwm;
    uint8_t buzzer;
    uint8_t servo_pwm;
    uint8_t ir_far_left;
    uint8_t ir_left;
    uint8_t ir_centre;
    uint8_t ir_right;
    uint8_t ir_far_right;
    uint8_t v_div_batt;
    uint8_t mic;
}
db1_hal_pins_t;

typedef struct
{
    db1_hal_pins_t        pins;
    db1_hal_set_pinmode_t set_pinmode;
    db1_hal_read_pin_t    digital_read;
    db1_hal_set_pin_t     digital_write;
    db1_hal_analog_read_t analog_read;
    db1_hal_set_pin_pwm_t analog_write;
    db1_hal_analog_res_t  analog_resolution;
    db1_hal_attach_int_t  attach_interrupt;
    db1_hal_detach_int_t  detach_interrupt;
    db1_hal_i2c_t         i2c_read;
    db1_hal_i2c_t         i2c_write;
    db1_hal_i2c_reg_t     i2c_reg_read;
    db1_hal_i2c_reg_t     i2c_reg_write;
    db1_i2c16_t           i2c_reg_read_16;
    db1_hal_i2c_size_t    i2c_buffer_size;
    db1_hal_servo_write_t servo_write;
    db1_hal_rgb_init_t    rgb_init;
    db1_hal_rgb_set_t     rgb_set;
    db1_hal_rgb_fill_t    rgb_fill;
    db1_hal_delay_ms_t    delay_ms;
    db1_hal_counter_t     millis;
    db1_hal_counter_t     micros;

} db1_hal_t;

#endif