#include "project.h"
#include "FreeRTOS.h"
#include "pwmTask.h"

#include "global.h"

void ezi2cTask(void *arg)
{
    (void)arg;
    uint8_t motorPercent[2];
    
    EZI2C_Start();
    EZI2C_SetBuffer1(motorPercent,sizeof(motorPercent),0);
    
    while(1)
    {
        motorPercent[0] = getMotorPercent(M1);
        motorPercent[1] = getMotorPercent(M2);
        
        xEventGroupWaitBits(
                       pwmEventGroup,      // which event group
                       PWM_EVENT_I2C,      // which bits to wait for
                       pdTRUE,             // clear the bit after it is set
                       pdFALSE,            // only wait for the i2c bit
                       portMAX_DELAY );    // wait forever
        
    }
}
