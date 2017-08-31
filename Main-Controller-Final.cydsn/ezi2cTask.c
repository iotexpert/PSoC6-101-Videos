
#include "project.h"
#include "FreeRTOS.h"
#include "global.h"
#include "pwmTask.h"


void ezi2cTask(void *arg)
{
    (void)arg;
    uint8_t motorPercent[2];
    
    EZI2C_Start();
    EZI2C_SetBuffer1(motorPercent,sizeof(motorPercent),0);
    
    while(1)
    {
        motorPercent[0] = getMotorPercent(1);
        motorPercent[1] = getMotorPercent(2);
        
        xEventGroupWaitBits(
                       pwmEventGroup,
                       PWM_EVENT_I2C,
                       pdTRUE,
                       pdFALSE,
                       portMAX_DELAY );
        
    }
}
