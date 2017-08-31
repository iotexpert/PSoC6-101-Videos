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
    PWM_Start();

    int compareValue=50;
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        Cy_TCPWM_PWM_SetCompare0(PWM_HW,PWM_CNT_NUM,compareValue);
        
        compareValue = (compareValue + 1) %100;
        CyDelay(20);
    }
}

/* [] END OF FILE */
