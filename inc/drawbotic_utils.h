#ifndef DRAWBOTIC_UTILS_H
#define DRAWBOTIC_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "drawbotic_types.h"

#define DB1_RAD_TO_DEG 57.295779513082320876798154814105

#define DB1_ERR_CHECK(err) if(err != 0) return err;

inline int db1_min(int a, int b)
{
    return a < b ? a : b;
}

inline int db1_constrain(int val, int min, int max)
{
    return val < min ? min : val > max ? max : val;
}

inline float db1_constrainf(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

inline int db1_map(int value, int in_min, int in_max, int out_min, int out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float db1_mapf(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float db1_lerpf(float amount, float start, float end)
{
    return db1_mapf(amount, 0.0f, 1.0f, start, end);
}

inline db1_colour_t db1_lerp_colour(float amount, db1_colour_t start, db1_colour_t end)
{
    db1_colour_t c = {
        (uint8_t)db1_mapf(amount, 0.0f, 1.0f, start.r, end.r), 
        (uint8_t)db1_mapf(amount, 0.0f, 1.0f, start.g, end.g), 
        (uint8_t)db1_mapf(amount, 0.0f, 1.0f, start.b, end.b)
    };

    return c;
}

inline db1_orientation_t db1_quaternion_to_orientation(db1_quaternion_t q)
{
    db1_orientation_t result;
    float sqr = q.r * q.r;
    float sqi = q.i * q.i;
    float sqj = q.j * q.j;
    float sqk = q.k * q.k;

    result.heading = (atan2(2.0 * (q.i * q.j + q.k * q.r), (sqi - sqj - sqk + sqr)) * DB1_RAD_TO_DEG) + 180.0f;
    result.roll = (asin(-2.0 * (q.i * q.k - q.j * q.r) / (sqi + sqj + sqk + sqr)) * DB1_RAD_TO_DEG) + 180.0f;
    result.pitch = (atan2(2.0 * (q.j * q.k + q.i * q.r), (-sqi - sqj + sqk + sqr)) * DB1_RAD_TO_DEG) + 180.0f;

    return result;
}

inline int32_t db1_float32_to_fixed32(float val, uint8_t q_point)
{
    return (int32_t)(val * (1 << q_point));
}

#ifdef __cplusplus
}
#endif

#endif