
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
TaskHandle_t bleTaskHandle;

void customEventHandler(uint32_t event, void *eventParameter)
{
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        PWM_BLINK_Start();    
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

        Cy_TCPWM_TriggerReloadOrIndex(PWM_DIM_HW,PWM_DIM_CNT_NUM);
        Cy_TCPWM_PWM_Disable(PWM_DIM_HW,PWM_DIM_CNT_NUM);
    
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            Cy_TCPWM_TriggerReloadOrIndex(PWM_BLINK_HW,PWM_BLINK_CNT_NUM);
            Cy_TCPWM_PWM_Disable(PWM_BLINK_HW,PWM_BLINK_CNT_NUM);
            
            PWM_DIM_Start();
            
        break;
        
        
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *)eventParameter;
        
            if(CY_BLE_LED_GREEN_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {

                uint32_t val = ((uint32_t)(writeReqParameter->handleValPair.value.val[0]) > Cy_TCPWM_PWM_GetPeriod0(PWM_DIM_HW,PWM_DIM_CNT_NUM))?
                 Cy_TCPWM_PWM_GetPeriod0(PWM_DIM_HW,PWM_DIM_CNT_NUM):writeReqParameter->handleValPair.value.val[0];
                    
                Cy_TCPWM_PWM_SetCompare0(PWM_DIM_HW,PWM_DIM_CNT_NUM,  val);
            }
                     
            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;
        
        default:
            break;
    }
}
void bleInterruptNotify()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(bleTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken); 
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void bleTask(void *arg)
{
    (void)arg;
    uint32 flags = 0;
    
    Cy_BLE_Start(customEventHandler);
  
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }
    Cy_BLE_IPC_RegisterAppHostCallback(bleInterruptNotify);
   
    for(;;)
    {
        xTaskNotifyWait(0, 0, &flags, portMAX_DELAY);
        Cy_BLE_ProcessEvents();      
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    xTaskCreate(bleTask,"bleTask",8*1024,0,1,&bleTaskHandle);
    vTaskStartScheduler();
 
}

/* [] END OF FILE */
