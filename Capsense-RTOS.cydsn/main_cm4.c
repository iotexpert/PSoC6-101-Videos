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

void capsenseTask(void *arg)
{
    
    CapSense_Start();
    CapSense_ScanAllWidgets();

    PWM_1_Start();
    
    for(;;)
    {
        if(!CapSense_IsBusy())
        {
            CapSense_ProcessAllWidgets();
            int pos;
            pos=CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID);
            if(pos<0xFFFF)
                Cy_TCPWM_PWM_SetCompare0(PWM_1_HW,PWM_1_CNT_NUM,pos);
            
            if(CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
            {
                Cy_TCPWM_PWM_Disable(PWM_1_HW,PWM_1_CNT_NUM);
                Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,0);
            
            }
            if(CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
            {
                //Cy_TCPWM_Enable_Multiple(PWM_1_HW, PWM_1_CNT_MASK);
                Cy_TCPWM_PWM_Enable(PWM_1_HW,PWM_1_CNT_NUM);
                Cy_TCPWM_TriggerStart(PWM_1_HW, PWM_1_CNT_MASK);
                
                Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,1);
            
            }
                
            CapSense_UpdateAllBaselines();
            CapSense_ScanAllWidgets();
        }
    }
}

int main()
{
      __enable_irq(); /* Enable global interrupts. */
  
    xTaskCreate( capsenseTask, "CapSense Task",400,0,1,0);
    vTaskStartScheduler();
}