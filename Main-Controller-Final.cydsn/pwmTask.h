#ifndef PWMTASK_H
#define PWMTASK_H
    
typedef enum {
    M1,
    M2
} motors_t;

typedef struct PWM_Message {
    motors_t motor;
    int percent;
    int percentChange;
} PWM_Message_t;

int getMotorPercent(motors_t motor);

#endif