#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bleTask.h"
#include "motionTask.h"

void capsenseTask(void *arg)
{
    (void)arg;
    
    motors_t currentMotor=M1;
    

    CapSense_Start();
    CapSense_ScanAllWidgets();
        
    for(;;)
    {
        if(!CapSense_IsBusy())
        {
            
            CapSense_ProcessAllWidgets();
            int pos;
            pos=CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID);
            if(pos<0xFFFF && xEventGroupGetBits(systemInputMode) == MODE_CAPSENSE)
            {
                writeMotorPosition(currentMotor,ABSOLUTE,pos);            
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