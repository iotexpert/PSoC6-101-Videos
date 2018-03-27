/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    
    volatile uint8_t myBuffer;
    
    EZI2C_Start();
    EZI2C_SetBuffer1((uint8_t *)&myBuffer,1,1);
    
    PWM_1_Start();
    
    for(;;)
    {
        Cy_TCPWM_PWM_SetCompare0(PWM_1_HW,PWM_1_CNT_NUM,myBuffer);
        Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
