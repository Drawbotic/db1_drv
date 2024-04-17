#include "inc/drawbotic_nav.h"
#include "inc/drawbotic_db1_drv.h"
#include "inc/drawbotic_utils.h"

#define L_TURN_ERROR    1.01789f   // scalar multiplier, obtained from testing, made left turns more accurate
#define R_TURN_ERROR    1.0045f    // scalar multiplier, obtained from testing, made right turns more accurate
#define L_ROTATE_ERROR  1.008f     // scalar multiplier, obtained from testing, made left rotate more accurate
#define R_ROTATE_ERROR  1.0045f    // scalar multiplier, obtained from testing, made right rotations more accurate
#define FORWARD_ERROR   1.0000f    // scalar multiplier, obtained from testing, made driving forward more accurate
#define ENC_BITS_P_MM   2.3433f    // encoder signals per rotation (295.6793)/wheel circum (40pi)
#define BOT_RADIUS      60
#define IMU_TOLERANCE   0.2f
#define SPEED_MIN       0.045f
#define ROTATE_KP       0.03f
#define FORWARD_KP      0.01f

static float m_speed = 0.2f;
static float m_cp = 0.015f;

typedef bool (*db1_nav_function_t)(db1_nav_action_t*);

bool db1_nav_forward(db1_nav_action_t *action)
{
    float enc_limit = action->desired_distance *  ENC_BITS_P_MM * FORWARD_ERROR;
    int enc_error = enc_limit - action->progress;

    if(enc_error > 0)
    {
        float power = db1_constrainf(m_speed * (enc_error * FORWARD_KP), SPEED_MIN, m_speed);
        float m1_delta = db1_encoder_delta(DB1_M_1);
        float m2_delta = db1_encoder_delta(DB1_M_2);
        float error = m1_delta - m2_delta;
        action->follow_speed += error + m_cp;

        db1_set_motor_speed(DB1_M_1, power);
        db1_set_motor_speed(DB1_M_2, action->follow_speed);
        action->progress += m1_delta;
        return false;
    }
    return true;
}

bool db1_nav_turn(db1_nav_action_t *action)
{
    if(action->progress < fabsf(action->desired_angle))
    {
        float d1 = db1_encoder_delta(DB1_M_1);
        float d2 = db1_encoder_delta(DB1_M_2);

        if(action->desired_angle >= 0)
        {
            float r1 = action->desired_radius + BOT_RADIUS;
            float r2 = action->desired_radius - BOT_RADIUS;

            float multiplier = r2 / r1;

            
            float error = d1 - (d2 / multiplier);

            action->follow_speed += error * m_cp;

            db1_set_motor_speed(DB1_M_1, m_speed);
            db1_set_motor_speed(DB1_M_2, action->follow_speed * multiplier);

            action->progress += (180 * d1) / (M_PI * ENC_BITS_P_MM * r1 * L_TURN_ERROR);
            return false;
        }
        else
        {
            float r1 = action->desired_radius - BOT_RADIUS;
            float r2 = action->desired_radius + BOT_RADIUS;

            float multiplier = r1 / r2;

            
            float error = d2 - (d1 / multiplier);

            action->follow_speed += error * m_cp;

            db1_set_motor_speed(DB1_M_1, action->follow_speed * multiplier);
            db1_set_motor_speed(DB1_M_2, m_speed);

            action->progress += (180 * d2) / (M_PI * ENC_BITS_P_MM * r2 * L_TURN_ERROR);
            return false;
        }
    }
    return true;
}

bool db1_nav_rotate(db1_nav_action_t *action)
{
    float current_angle = db1_read_orientation().heading;

    //error calculation performed in millidegrees to allow for modulus operations
    float error = (int)((current_angle - action->final_angle + 540) * 1000) % 360000 - 180000;
    //convert to floating point degrees
    error /= 1000;

    if(error > -IMU_TOLERANCE && error < IMU_TOLERANCE) 
        return true;

    float power = db1_constrainf(m_speed * (ROTATE_KP * fabsf(error)), SPEED_MIN, m_speed);
    if(error > 0)
    {
        db1_set_motor_speed(DB1_M_1, -power);
        db1_set_motor_speed(DB1_M_2, power);
    }
    else
    {
        db1_set_motor_speed(DB1_M_1, power);
        db1_set_motor_speed(DB1_M_2, -power);
    }

    return false;
}

bool db1_nav_stop(db1_nav_action_t *action)
{
    if(action->final_time < 0)
        action->final_time = db1_millis() + action->desired_time;
    
    db1_set_motor_speed(DB1_M_1, 0);
    db1_set_motor_speed(DB1_M_2, 0);

    action->progress = db1_millis();

    return (action->progress >= action->final_time);
}

bool db1_nav_pen_down(db1_nav_action_t *action)
{
    db1_set_pen(true);
    return true;
}

bool db1_nav_pen_up(db1_nav_action_t *action)
{
    db1_set_pen(false);
    return true;
}

void db1_nav_set_cp(float cp)
{
    m_cp = cp;
}

void db1_nav_set_speed(float speed)
{
    if(speed > 0 && speed <= 1.0)
        m_speed = speed;
}

void db1_nav_build_forward_action(float distance_mm, db1_nav_action_t *action)
{
    action->type = NAV_FORWARD;
    action->desired_distance = distance_mm;
    action->follow_speed = m_speed;
    action->progress = 0;
}

void db1_nav_build_turn_action(float radius_mm, float angle_deg, db1_nav_action_t *action)
{
    action->type = NAV_TURN;
    action->desired_radius = radius_mm;
    action->desired_angle = angle_deg;
    action->follow_speed = m_speed;
    action->progress = 0;
}

void db1_nav_build_rotate_action(float angle_deg, db1_nav_action_t *action)
{
    action->type = NAV_ROTATE;
    action->desired_angle = angle_deg;
    action->follow_speed = m_speed;
    action->progress = 0;

    float current_angle = db1_read_orientation().heading;
    action->final_angle = current_angle + action->desired_angle;

    if(action->final_angle >= 360.0f)
        action->final_angle -= 360.0f;
    else if(action->final_angle < 0)
        action->final_angle += 360.0f;
}

void db1_nav_build_stop_action(float time_ms, db1_nav_action_t *action)
{
    action->type = NAV_STOP;
    action->desired_time = time_ms;
    action->follow_speed = m_speed;
    action->final_time = -1;
    action->progress = 0;
}

void db1_nav_build_pen_action(bool down, db1_nav_action_t *action)
{
    action->type = down ? NAV_PEN_DOWN : NAV_PEN_UP;
    action->follow_speed = m_speed;
    action->progress = 0;
}

bool db1_nav_perform_action(db1_nav_action_t* action, bool blocking)
{
    db1_nav_function_t func = 0;

    //Pick which function to run
    switch(action->type)
    {
    case NAV_FORWARD:
        func = db1_nav_forward;
        break;
    case NAV_TURN:
        func = db1_nav_turn;
        break;
    case NAV_ROTATE:
        func = db1_nav_rotate;
        break;
    case NAV_STOP:
        func = db1_nav_stop;
        break;
    case NAV_PEN_DOWN:
        func = db1_nav_pen_down;
        break;
    case NAV_PEN_UP:
        func = db1_nav_pen_up;
        break;
    }

    //guard against no function
    if(func == 0)
        return false;

    //if blocking run function is loop until complete
    if(blocking)
    {
        while(func(action) != false)
            db1_delay_ms(1);
        
        db1_reset_encoder_deltas();
        return true;
    }

    //otherwise run the function
    if(func(action))
    {
        //if it's finished reset deltas
        db1_reset_encoder_deltas();
        return true;
    }

    return false;
}