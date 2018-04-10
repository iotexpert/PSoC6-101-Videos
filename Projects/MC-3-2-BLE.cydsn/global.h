#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

extern QueueHandle_t pwmQueue;
    
extern EventGroupHandle_t pwmEventGroup;
#define PWM_EVENT_I2C (1<<0)
#define PWM_EVENT_BLE (1<<1)
#define PWM_EVENT_ALL (PWM_EVENT_I2C | PWM_EVENT_BLE)

extern TaskHandle_t bleTaskHandle;
 