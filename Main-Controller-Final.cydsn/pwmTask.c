#include <project.h>
#include "pwmTask.h"
#include "global.h"


static int inline compareToPercent(int compare)
{
    return (compare - 2400)/30;
}

static inline int percentToCompare(int percent)
{
    return 3*(800 + percent * 10);
}
int getMotorPercent(motors_t motor)
{
    TCPWM_Type* hw;
    int cntrNum=0;
    switch(motor)
    {
        case M1:
            hw = PWM_1_HW;
            cntrNum = PWM_1_CNT_NUM;
        break;
        case M2:
            hw = PWM_2_HW;
            cntrNum = PWM_2_CNT_NUM;
        break;
       
    }
    return(compareToPercent(Cy_TCPWM_PWM_GetCompare0 (hw, cntrNum)));
}


void pwmTask(void *arg)
{
    (void)arg;
    PWM_Message_t pwmMessage;
    TCPWM_Type* hw;
    int cntrNum=0;
    int compareTmp;
    int percentTmp;
    
    PWM_1_Start();
    PWM_2_Start();
    
    while(1)
    {
        xQueueReceive(pwmQueue,&pwmMessage,portMAX_DELAY);
    
        switch(pwmMessage.motor)
        {
            case M1:
                hw = PWM_1_HW;
                cntrNum = PWM_1_CNT_NUM;
            break;
            case M2:
                hw = PWM_2_HW;
                cntrNum = PWM_2_CNT_NUM;
            break;
            
        
        }
    
        if(pwmMessage.percent == -1) // they want a percent change
        {
            compareTmp = Cy_TCPWM_PWM_GetCompare0(hw,cntrNum);
            percentTmp = compareToPercent(compareTmp);
            percentTmp += pwmMessage.percentChange;
        
            if(percentTmp < 0)
                percentTmp = 0;
            if (percentTmp>100)
                percentTmp=100;
   
        }
        else // they want an absolute percent
        {
            percentTmp = pwmMessage.percent;
        }
        
        Cy_TCPWM_PWM_SetCompare0 (hw, cntrNum, percentToCompare(percentTmp) );
        xEventGroupSetBits(pwmEventGroup ,PWM_EVENT_ALL);

    }
}

