/***************************************************************************//**
* \file global.h
* \version 
*
* \brief
* This gives global access to the RTOS handles used to communicate between
* the tasks
*
*******************************************************************************/
#ifndef GLOBAL_H
#define GLOBAL_H

#include "FreeRTOS.h"
#include "event_groups.h"    
#include "semphr.h"  
    

// This semaphore is used to signal the I2C Task that new values are ready to be updated
extern SemaphoreHandle_t updateEZI2CSemaphore;


//extern SemaphoreHandle_t uartSemaphore;
    
// You send messages to the PWM (motor contorller task) in the pwmQueue
extern QueueHandle_t pwmQueue;
    
// The PWM event group is used to signal a change in the motor percent values
extern EventGroupHandle_t pwmEventGroup;    
#define PWM_EVENT_I2C (1<<0)
#define PWM_EVENT_BLE (1<<1)
#define PWM_EVENT_ALL (PWM_EVENT_I2C | PWM_EVENT_BLE)    

extern SemaphoreHandle_t uartSemaphore;
extern  TaskHandle_t bleTaskHandle;
#endif