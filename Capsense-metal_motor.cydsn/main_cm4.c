
#include "project.h"

int percentToCompare(int percent)
{
    return 3*(800 + percent * 10);
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
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
            {
                Cy_TCPWM_PWM_SetCompare0(PWM_1_HW,PWM_1_CNT_NUM,percentToCompare(pos));
            }
            
                            
            CapSense_UpdateAllBaselines();
            CapSense_ScanAllWidgets();
        }
    }
}

/* [] END OF FILE */
