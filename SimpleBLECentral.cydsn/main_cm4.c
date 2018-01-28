
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define SIMPLEBLEPERIPHERAL_NAME "P6LED"
#define SIMPLEBLEPERIPHERAL_LEN 28
const uint8_t ledService[] = {0x27, 0x74, 0x70,0xF7,0x6D, 0x4B, 0x49, 0x82, 0x01, 0x4a, 0x9e, 0xcd, 0x56, 0x04, 0x55, 0x50};

void customEventHandler(uint32_t event, void *eventParameter)
{
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;
    cy_stc_ble_gapc_adv_report_param_t  *scanProgressParam;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("Starting Scan\n\r");
          
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,0);  
        
        break;
        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
            printf("Found Device ");
            scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t  *)eventParameter;
            printf("BD Addr = ");
            for(unsigned int i=0;i<CY_BLE_BD_ADDR_SIZE;i++)
            {
                printf("%2X",scanProgressParam->peerBdAddr[i]);
            }
            
            printf(" Length = %d\r\n",scanProgressParam->dataLen);
            
            if(scanProgressParam->dataLen == SIMPLEBLEPERIPHERAL_LEN &&
                strncmp(SIMPLEBLEPERIPHERAL_NAME,&scanProgressParam->data[5],strlen(SIMPLEBLEPERIPHERAL_NAME)) == 0 
               &&   memcmp(&ledService[0],&scanProgressParam->data[12],16) == 0)
            {
                printf("Found P6LED\r\n");
                // make a connection
                cy_stc_ble_bd_addr_t connectAddr;
                memcpy(&connectAddr.bdAddr[0], &scanProgressParam->peerBdAddr[0] , CY_BLE_BD_ADDR_SIZE);
                connectAddr.type = scanProgressParam->peerAddrType;
                Cy_BLE_GAPC_ConnectDevice(&connectAddr,0);
                Cy_BLE_GAPC_StopScan();
            }
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("Made a connection\r\n");
            
            
        break;
        
        
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        break;
        
        default:
            break;
    }
}

void bleTask(void *arg)
{
    (void)arg;
    
    Cy_BLE_Start(customEventHandler);
    
    PWM_DIM_Start();
    
    for(;;)
    {
         Cy_BLE_ProcessEvents();
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    printf("Started\n");

    xTaskCreate(bleTask,"bleTask",8*1024,0,1,0);
    vTaskStartScheduler();
 
}

/* [] END OF FILE */
