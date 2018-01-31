#include "project.h"
#include "global.h"
#include "pwmTask.h"

void capsenseTask(void *arg)
{
    (void)arg;
    
    motors_t currentMotor=M1;
    PWM_Message_t myMessage;
    

    CapSense_Start();
    CapSense_ScanAllWidgets();
        
    for(;;)
    {
        if(!CapSense_IsBusy())
        {
            
            CapSense_ProcessAllWidgets();
            int pos;
            pos=CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID);
            if(pos<0xFFFF)
            {
                
                myMessage.motor = currentMotor;
                myMessage.percent = pos;
                xQueueSend(pwmQueue,&myMessage,0);
            }
            
            if(CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
            {
                currentMotor = M1;
            }
            if(CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
            {
               currentMotor = M2;
            }
               
            CapSense_UpdateAllBaselines();
            CapSense_ScanAllWidgets();
        }
        else
            vTaskDelay(50);
    }
}