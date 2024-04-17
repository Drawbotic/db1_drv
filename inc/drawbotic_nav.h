#ifndef DRAWBOTIC_NAV_H
#define DRAWBOTIC_NAV_H

#ifdef __cplusplus 
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum 
{
    NAV_FORWARD,
    NAV_TURN,
    NAV_ROTATE,
    NAV_STOP,
    NAV_PEN_UP,
    NAV_PEN_DOWN,
}
db1_nav_type_t;

typedef struct
{
    db1_nav_type_t type;
    float follow_speed;
    float progress;
    float desired_angle;
    float final_angle;
    float desired_distance;
    float desired_radius;
    float desired_time;
    float final_time;
}
db1_nav_action_t;

void db1_nav_set_cp(float cp);
void db1_nav_set_speed(float speed);

void db1_nav_build_forward_action(float distance_mm, db1_nav_action_t *action);
void db1_nav_build_turn_action(float radius_mm, float angle_deg, db1_nav_action_t *action);
void db1_nav_build_rotate_action(float angle_deg, db1_nav_action_t *action);
void db1_nav_build_stop_action(float time_ms, db1_nav_action_t *action);
void db1_nav_build_pen_action(bool down, db1_nav_action_t *action);

bool db1_nav_perform_action(db1_nav_action_t* action, bool blocking);

#ifdef __cplusplus
}
#endif


#endif