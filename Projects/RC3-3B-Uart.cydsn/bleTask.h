#pragma once
#include <stdint.h>

typedef enum {
    M1,
    M2
} motors_t;

typedef enum {
    ABSOLUTE,
    RELATIVE
} motor_change_t;

void writeMotorPosition(motors_t motor, motor_change_t type, int8_t percent);

void bleTask(void *arg);
