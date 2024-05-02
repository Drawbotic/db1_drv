#include "inc/drawbotic_db1_drv.h"
#include "inc/drawbotic_utils.h"
#include "inc/veml6040_drv.h"
#include "inc/bno085_drv.h"
#include "inc/vl53l0x_drv.h"

#include <math.h>
#include <memory.h>

//private module variables
static db1_hal_t *m_hal;
static db1_settings_t m_current_settings;

static db1_ir_array_t m_ir_high;
static db1_ir_array_t m_ir_low;
static db1_colour_reading_t m_colour_high;
static db1_colour_reading_t m_colour_low;

static volatile long m_encoders[DB1_MOTOR_COUNT];
static volatile uint8_t m_encoder_last_a_state[DB1_MOTOR_COUNT];
static long m_last_encoders[DB1_MOTOR_COUNT] = { 0, 0 };

static db1_ir_array_t *m_ir_window = 0;
static db1_quaternion_t m_current_rotation;
static db1_vector_t m_current_acceleration;
static bool m_was_bumped;

static db1_lights_t m_current_lights;
static db1_colour_t m_current_top_light;

static uint8_t m_white_led_state = 0;

//callback functions
void db1_m1_encoder_callback()
{
    uint8_t m1_a_state = m_hal->digital_read(m_hal->pins.m1_e_a);

    if(m1_a_state != m_encoder_last_a_state[DB1_M1] && m1_a_state == 1)
    {
        if(m_hal->digital_read(m_hal->pins.m1_e_b) != m1_a_state)
            m_encoders[DB1_M1]--;
        else
            m_encoders[DB1_M1]++;
    }
    m_encoder_last_a_state[DB1_M1] = m1_a_state;
}

void db1_m2_encoder_callback()
{
    uint8_t m2_a_state = m_hal->digital_read(m_hal->pins.m2_e_a);

    if(m2_a_state != m_encoder_last_a_state[DB1_M2] && m2_a_state == 1)
    {
        if(m_hal->digital_read(m_hal->pins.m2_e_b) != m2_a_state)
            m_encoders[DB1_M2]++;
        else
            m_encoders[DB1_M2]--;
    }
    m_encoder_last_a_state[DB1_M2] = m2_a_state;
}

void db1_imu_int_callback(bno085_evt_t evt, void *p_data)
{
    db1_vector_t *accel;
    db1_quaternion_t *rot;

    switch(evt)
    {
    case BNO085_ACCEL_EVT:
        accel = (db1_vector_t*)p_data;
        m_current_acceleration.x = accel->x;
        m_current_acceleration.y = accel->y;
        m_current_acceleration.z = accel->z;
        break;
    case BNO085_ROTATION_EVT:
        rot = (db1_quaternion_t*)p_data;
        m_current_rotation.r = rot->r;
        m_current_rotation.i = rot->i;
        m_current_rotation.j = rot->j;
        m_current_rotation.k = rot->k;
        break;
    case BNO085_BUMP_EVT:
        m_was_bumped = true;
    }
}

db1_error_t db1_validate_hal()
{
    if ((m_hal->set_pinmode      == 0) ||
        (m_hal->digital_read     == 0) ||
        (m_hal->digital_write    == 0) ||
        (m_hal->analog_read      == 0) ||
        (m_hal->analog_write     == 0) ||
        (m_hal->attach_interrupt == 0) ||
        (m_hal->detach_interrupt == 0) ||
        (m_hal->i2c_read         == 0) ||
        (m_hal->i2c_write        == 0) ||
        (m_hal->i2c_reg_read     == 0) ||
        (m_hal->i2c_reg_write    == 0) ||
        (m_hal->i2c_buffer_size  == 0) ||
        (m_hal->servo_write      == 0) ||
        (m_hal->rgb_init         == 0) ||
        (m_hal->rgb_set          == 0) ||
        (m_hal->rgb_fill         == 0) ||
        (m_hal->delay_ms         == 0) ||
        (m_hal->millis           == 0) ||
        (m_hal->micros           == 0))
    {
        return DB1_ERR_HAL;
    }

    return DB1_SUCCESS;
}

db1_error_t db1_init_motors(bool use_encoders) 
{
    m_current_settings.use_encoders = use_encoders;

    m_hal->analog_resolution(16);

    m_hal->set_pinmode(m_hal->pins.m1_dir_a, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.m1_dir_b, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.m2_dir_a, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.m2_dir_b, DB1_HAL_PIN_OUT);

    m_hal->set_pinmode(m_hal->pins.m1_e_a, DB1_HAL_PIN_IN);
    m_hal->set_pinmode(m_hal->pins.m1_e_b, DB1_HAL_PIN_IN);
    m_hal->set_pinmode(m_hal->pins.m2_e_a, DB1_HAL_PIN_IN);
    m_hal->set_pinmode(m_hal->pins.m2_e_b, DB1_HAL_PIN_IN);

    if(m_current_settings.use_encoders)
    {
        m_hal->attach_interrupt(m_hal->pins.m1_e_a, DB1_HAL_INT_CHANGE, db1_m1_encoder_callback);
        m_hal->attach_interrupt(m_hal->pins.m1_e_b, DB1_HAL_INT_CHANGE, db1_m1_encoder_callback);
        m_hal->attach_interrupt(m_hal->pins.m2_e_a, DB1_HAL_INT_CHANGE, db1_m2_encoder_callback);
        m_hal->attach_interrupt(m_hal->pins.m2_e_b, DB1_HAL_INT_CHANGE, db1_m2_encoder_callback);
    }
    else 
    {
        m_hal->detach_interrupt(m_hal->pins.m1_e_a);
        m_hal->detach_interrupt(m_hal->pins.m1_e_b);
        m_hal->detach_interrupt(m_hal->pins.m2_e_a);
        m_hal->detach_interrupt(m_hal->pins.m2_e_b);
    }

    return DB1_SUCCESS;
}

db1_error_t db1_init_ir_array(uint8_t ir_window_size)
{
    m_current_settings.ir_window_size = ir_window_size;

    m_hal->set_pinmode(m_hal->pins.ir_far_left,  DB1_HAL_PIN_IN_ANALOG);
    m_hal->set_pinmode(m_hal->pins.ir_left,      DB1_HAL_PIN_IN_ANALOG);
    m_hal->set_pinmode(m_hal->pins.ir_centre,    DB1_HAL_PIN_IN_ANALOG);
    m_hal->set_pinmode(m_hal->pins.ir_right,     DB1_HAL_PIN_IN_ANALOG);
    m_hal->set_pinmode(m_hal->pins.ir_far_right, DB1_HAL_PIN_IN_ANALOG);

    return DB1_SUCCESS;
}

db1_error_t db1_init_battery_monitor()
{
    m_hal->set_pinmode(m_hal->pins.batt_lvl1, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.batt_lvl2, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.batt_lvl3, DB1_HAL_PIN_OUT);
    m_hal->set_pinmode(m_hal->pins.batt_lvl4, DB1_HAL_PIN_OUT);

    m_hal->digital_write(m_hal->pins.batt_lvl1, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl2, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl3, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl4, 0);

    m_hal->set_pinmode(m_hal->pins.v_div_batt, DB1_HAL_PIN_IN_ANALOG);

    return DB1_SUCCESS;
}

db1_error_t db1_init_white_led()
{
    m_hal->set_pinmode(m_hal->pins.led_en, DB1_HAL_PIN_OUT);
    m_hal->digital_write(m_hal->pins.led_en, 0);

    return DB1_SUCCESS;
}

db1_error_t db1_init_servo(db1_servo_settings_t settings)
{
    m_current_settings.pen = settings;
    m_hal->set_pinmode(m_hal->pins.servo_pwm, DB1_HAL_PIN_OUT_SERVO);

    return DB1_SUCCESS;
}

db1_error_t db1_init_tof(db1_tof_location_t location, vl53l0x_config_t config)
{
    m_current_settings.tofs[location] = config;

    return vl53l0x_configure(location, config);
}

db1_error_t db1_init_colour_sensor(veml6040_int_time_t int_time)
{
    m_current_settings.colour_int_time = int_time;
    if(!veml6040_init(m_hal))
        return DB1_ERR_COLOUR;

    veml6040_set_config(m_current_settings.colour_int_time, false, false, false);

    return DB1_SUCCESS;
}

db1_error_t db1_init_rgb_lights()
{
    m_hal->rgb_init(m_hal->pins.rgb_dout, DB1_LIGHT_COUNT);
    m_hal->rgb_fill(m_hal->pins.rgb_dout, 0, 0, 0);

    m_hal->rgb_init(m_hal->pins.stat_dout, 1);
    m_hal->rgb_fill(m_hal->pins.stat_dout, 0, 0, 0);

    return DB1_SUCCESS;
}

db1_error_t db1_init_imu(db1_imu_settings_t settings)
{
    m_current_settings.imu = settings;
    
    if(!bno085_init(BNO085_DEFAULT_ADDR, m_hal, db1_imu_int_callback, m_current_settings.imu))
        return DB1_ERR_IMU;

    bno085_hardware_reset();

    if(!bno085_begin())
        return DB1_ERR_IMU;

    return DB1_SUCCESS;
}

db1_error_t db1_init(db1_hal_t *hal, db1_settings_t settings)
{
    db1_error_t err;
    
    m_hal = hal;
    m_current_settings = settings;

    err = db1_validate_hal();
    DB1_ERR_CHECK(err);

    err = db1_init_motors(m_current_settings.use_encoders);
    DB1_ERR_CHECK(err);

    err = db1_init_ir_array(m_current_settings.ir_window_size);
    DB1_ERR_CHECK(err);

    err = db1_init_battery_monitor();
    DB1_ERR_CHECK(err);

    err = db1_init_white_led();
    DB1_ERR_CHECK(err);

    err = db1_init_servo(m_current_settings.pen);
    DB1_ERR_CHECK(err);

    err = db1_init_rgb_lights();
    DB1_ERR_CHECK(err);

    err = db1_init_colour_sensor(m_current_settings.colour_int_time);
    DB1_ERR_CHECK(err);

    err = vl53l0x_init(m_hal);
    DB1_ERR_CHECK(err);

    for(int i = 0; i < DB1_TOF_COUNT; i++) 
    {
        err = db1_init_tof(i, m_current_settings.tofs[i]);
        DB1_ERR_CHECK(err);
    }

    err = db1_init_imu(m_current_settings.imu);
    DB1_ERR_CHECK(err);
    
    return DB1_SUCCESS;
}

db1_settings_t db1_get_current_settings()
{
    return m_current_settings;
}

void db1_calibrate_ir_array()
{
    m_ir_low.far_left  = 1024;
    m_ir_low.left      = 1024;
    m_ir_low.centre    = 1024;
    m_ir_low.right     = 1024;
    m_ir_low.far_right = 1024;

    m_ir_high.far_left  = 0;
    m_ir_high.left      = 0;
    m_ir_high.centre    = 0;
    m_ir_high.right     = 0;
    m_ir_high.far_right = 0;

    db1_set_motor_speed(DB1_M1, 0.1);
    db1_set_motor_speed(DB1_M2, -0.1);

    for(int i = 0; i < DB1_IR_CALIBRATION_COUNT; i++)
    {
        db1_ir_array_t ir = db1_read_ir(false);
        if(ir.far_left < m_ir_low.far_left)
            m_ir_low.far_left = ir.far_left;
        else if(ir.far_left > m_ir_high.far_left)
            m_ir_high.far_left = ir.far_left;
        
        if(ir.left < m_ir_low.left)
            m_ir_low.left = ir.left;
        else if(ir.left > m_ir_high.left)
            m_ir_high.left = ir.left;

        if(ir.centre < m_ir_low.centre)
            m_ir_low.centre = ir.centre;
        else if(ir.centre > m_ir_high.centre)
            m_ir_high.centre = ir.centre;

        if(ir.right < m_ir_low.right)
            m_ir_low.right = ir.right;
        else if(ir.right > m_ir_high.right)
            m_ir_high.right = ir.right;

        if(ir.far_right < m_ir_low.far_right)
            m_ir_low.far_right = ir.far_right;
        else if(ir.far_right > m_ir_high.far_right)
            m_ir_high.far_right = ir.far_right;
        
        m_hal->delay_ms(DB1_IR_CALIBRATION_DELAY_MS);
    }
    db1_set_motor_speed(DB1_M1, 0);
    db1_set_motor_speed(DB1_M2, 0);
}

void db1_set_ir_calibration(db1_ir_array_t low, db1_ir_array_t high)
{
    m_ir_low = low;
    m_ir_high = high;
}

db1_ir_array_t db1_get_ir_high()
{
    return m_ir_high;
}

db1_ir_array_t db1_get_ir_low()
{
    return m_ir_low;
}

void db1_calibrate_colour()
{
    uint8_t led_state = m_white_led_state;
    db1_colour_reading_t high_sum;
    db1_colour_reading_t low_sum;

    memset(&high_sum, 0, sizeof(high_sum));
    memset(&low_sum, 0, sizeof(low_sum));

    db1_set_white_led(true);

    for(int i = 0; i < DB1_COLOUR_CALIBRATION_COUNT; i++)
    {
        db1_colour_reading_t r = veml6040_read_colour();
        high_sum.r += r.r;
        high_sum.g += r.g;
        high_sum.b += r.b;
        high_sum.w += r.w;
        m_hal->delay_ms(DB1_COLOUR_CALIBRATION_DELAY_MS);
    }
    m_colour_high.r = high_sum.r / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_high.g = high_sum.g / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_high.b = high_sum.b / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_high.w = high_sum.w / DB1_COLOUR_CALIBRATION_COUNT;

    db1_set_white_led(false);
    m_hal->delay_ms(DB1_COLOUR_CALIBRATION_DELAY_MS);
    
    for(int i = 0; i < DB1_COLOUR_CALIBRATION_COUNT; i++)
    {
        db1_colour_reading_t r = veml6040_read_colour();
        low_sum.r += r.r;
        low_sum.g += r.g;
        low_sum.b += r.b;
        low_sum.w += r.w;
        m_hal->delay_ms(DB1_COLOUR_CALIBRATION_DELAY_MS);
    }
    m_colour_low.r = low_sum.r / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_low.g = low_sum.g / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_low.b = low_sum.b / DB1_COLOUR_CALIBRATION_COUNT;
    m_colour_low.w = low_sum.w / DB1_COLOUR_CALIBRATION_COUNT;

    db1_set_white_led(led_state);
}

void db1_set_colour_calibration(db1_colour_reading_t low, db1_colour_reading_t high)
{
    m_colour_low = low;
    m_colour_high = high;
}

db1_colour_reading_t db1_get_colour_high()
{
    return m_colour_high;
}

db1_colour_reading_t db1_get_colour_low()
{
    return m_colour_low;
}

void db1_set_white_led(bool on)
{
    m_white_led_state = on;
    m_hal->digital_write(m_hal->pins.led_en, on);
}

void db1_set_lights(db1_lights_t lights)
{
    m_current_lights = lights;
    for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    {
        m_hal->rgb_set(m_hal->pins.rgb_dout, i, lights.led[i].r, lights.led[i].g, lights.led[i].b);
    }
}

void db1_set_top_light(db1_colour_t colour)
{
    m_current_top_light = colour;

    m_hal->rgb_set(m_hal->pins.stat_dout, 0, colour.r, colour.g, colour.b);
}

db1_lights_t db1_get_light(int index)
{
    return m_current_lights;
}

db1_colour_t db1_get_top_light()
{
    return m_current_top_light;
}

float db1_update_battery(bool lights)
{
    int batt_reading = m_hal->analog_read(m_hal->pins.v_div_batt);
    float voltage = (batt_reading / 1024.0f) * DB1_BATT_VOLT_HIGH;

    float percentage = db1_constrainf(db1_mapf(voltage, DB1_BATT_VOLT_LOW, DB1_BATT_VOLT_HIGH, 0.0f, 100.0f), 0.0f, 100.0f);

    m_hal->digital_write(m_hal->pins.batt_lvl1, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl2, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl3, 0);
    m_hal->digital_write(m_hal->pins.batt_lvl4, 0);

    if(lights)
    {
        if(percentage > 10)
            m_hal->digital_write(m_hal->pins.batt_lvl1, 1);
        if(percentage > 25)
            m_hal->digital_write(m_hal->pins.batt_lvl2, 1);
        if(percentage > 50)
            m_hal->digital_write(m_hal->pins.batt_lvl3, 1);
        if(percentage > 75)
            m_hal->digital_write(m_hal->pins.batt_lvl4, 1);
    }

    return percentage;
}

void db1_set_pen(bool down)
{
    if(down)
        m_hal->servo_write(m_hal->pins.servo_pwm, m_current_settings.pen.pen_down_pos);
    else
        m_hal->servo_write(m_hal->pins.servo_pwm, m_current_settings.pen.pen_up_pos);
}

void db1_set_pen_pos(float pos)
{
    uint8_t val = (uint8_t)(db1_constrainf(db1_mapf(pos, 0.0f, 1.0f, m_current_settings.pen.pen_up_pos, m_current_settings.pen.pen_down_pos), m_current_settings.pen.pen_up_pos, m_current_settings.pen.pen_down_pos));
    m_hal->servo_write(m_hal->pins.servo_pwm, val);
}

void db1_set_motor_speed(db1_motor_t motor, float speed)
{
    bool direction = speed > 0;
    uint16_t pwm_val = (uint16_t)(fabs(speed) * UINT16_MAX);

    switch(motor)
    {
    case DB1_M1:
        if(direction)
        {
            m_hal->digital_write(m_hal->pins.m1_dir_a, 1);
            m_hal->digital_write(m_hal->pins.m1_dir_b, 0);
        }
        else
        {
            m_hal->digital_write(m_hal->pins.m1_dir_a, 0);
            m_hal->digital_write(m_hal->pins.m1_dir_b, 1);
        }
        m_hal->analog_write(m_hal->pins.m1_pwm, pwm_val);
        break;
    case DB1_M2:
        if(direction)
        {
            m_hal->digital_write(m_hal->pins.m2_dir_a, 0);
            m_hal->digital_write(m_hal->pins.m2_dir_b, 1);
        }
        else
        {
            m_hal->digital_write(m_hal->pins.m2_dir_a, 1);
            m_hal->digital_write(m_hal->pins.m2_dir_b, 0);
        }
        m_hal->analog_write(m_hal->pins.m2_pwm, pwm_val);
        break;
    }

    
}

long db1_encoder_val(db1_motor_t motor)
{
    return m_encoders[motor];
}

long db1_encoder_delta(db1_motor_t motor)
{
    long delta = m_encoders[motor] - m_last_encoders[motor];
    m_last_encoders[motor] = m_encoders[motor];
    return delta;
}

void db1_reset_encoder_deltas()
{
    m_last_encoders[DB1_M1] = m_encoders[DB1_M1];
    m_last_encoders[DB1_M2] = m_encoders[DB1_M2];
}

db1_colour_reading_t db1_read_colour(bool calibrated)
{
    db1_colour_reading_t r = veml6040_read_colour();

    if(calibrated)
    {
        r.r = db1_constrain(db1_map(r.r, m_colour_low.r, m_colour_high.r, 0, 255), 0, 255);
        r.g = db1_constrain(db1_map(r.g, m_colour_low.g, m_colour_high.g, 0, 255), 0, 255);
        r.b = db1_constrain(db1_map(r.b, m_colour_low.b, m_colour_high.b, 0, 255), 0, 255);
        r.w = db1_constrain(db1_map(r.w, m_colour_low.w, m_colour_high.w, 0, 255), 0, 255);
    }

    return r;
}

db1_ir_array_t db1_read_ir(bool calibrated)
{
    db1_ir_array_t ir = { 0 };

    for(int i = 0; i < m_current_settings.ir_window_size; i++)
    {
        ir.far_left  += m_hal->analog_read(m_hal->pins.ir_far_left);
        ir.left      += m_hal->analog_read(m_hal->pins.ir_left);
        ir.centre    += m_hal->analog_read(m_hal->pins.ir_centre);
        ir.right     += m_hal->analog_read(m_hal->pins.ir_right);
        ir.far_right += m_hal->analog_read(m_hal->pins.ir_far_right);
    }

    ir.far_left  = ir.far_left  / m_current_settings.ir_window_size;
    ir.left      = ir.left      / m_current_settings.ir_window_size;
    ir.centre    = ir.centre    / m_current_settings.ir_window_size;
    ir.right     = ir.right     / m_current_settings.ir_window_size;
    ir.far_right = ir.far_right / m_current_settings.ir_window_size;

    if(calibrated)
    {
        ir.far_left  = db1_constrain(db1_map(ir.far_left,  m_ir_low.far_left,  m_ir_high.far_left,  0, 255), 0, 255);
        ir.left      = db1_constrain(db1_map(ir.left,      m_ir_low.left,      m_ir_high.left,      0, 255), 0, 255);
        ir.centre    = db1_constrain(db1_map(ir.centre,    m_ir_low.centre,    m_ir_high.centre,    0, 255), 0, 255);
        ir.right     = db1_constrain(db1_map(ir.right,     m_ir_low.right,     m_ir_high.right,     0, 255), 0, 255);
        ir.far_right = db1_constrain(db1_map(ir.far_right, m_ir_low.far_right, m_ir_high.far_right, 0, 255), 0, 255);
    }

    return ir;
}

int db1_read_tof(db1_tof_location_t tof)
{
    return vl53l0x_range(tof);
}

db1_quaternion_t db1_read_rotation()
{
    bno085_service();
    return m_current_rotation;
}

db1_orientation_t db1_read_orientation()
{   
    bno085_service();
    return db1_quaternion_to_orientation(m_current_rotation);
}

db1_vector_t db1_read_acceleration()
{
    bno085_service();
    return m_current_acceleration;
}

bool db1_check_bump()
{
    bno085_service();

    if(m_was_bumped)
    {
        m_was_bumped = false;
        return true;
    }
    return false;
}

void db1_delay_ms(uint32_t ms)
{
    m_hal->delay_ms(ms);
}

long db1_millis()
{
    return m_hal->millis();
}