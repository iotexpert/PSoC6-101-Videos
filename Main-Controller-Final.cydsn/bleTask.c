/***************************************************************************//**
* \file bleTask.c
* \version 
*
* \brief
* This file handles all the BLE Processing and the FreeRTOS task
*
*******************************************************************************/
#include <project.h>
#include "global.h"
#include "pwmTask.h"
#include "semphr.h"
#include <stdio.h>

SemaphoreHandle_t bleSemaphore;
TaskHandle_t bleTaskHandle;

/*******************************************************************************
* Function Name: updateMotorsGatt
********************************************************************************
*
* This function writes the GATT database with current Motor percent.  The write 
* can be CY_BLE_GATT_DB_LOCALLY_INITIATED or CY_BLE_GATT_DB_PEER_INITIATED
* Once the write is done it will notify all connections who have CCCD notify set
*
* \param 
*  motor and enumerated motor_t that is either M1 or M2
* \param
*  percent a number from 0 to 100 as a uint8_t
* \param
*  flags - either CY_BLE_GATT_DB_LOCALLY_INITIATED or CY_BLE_GATT_DB_PEER_INITIATED
* \param
* connectionHandle - either a connectionHandle or a bunch or a zero'd structure 
* (if LOCALLY_INITIATED)
* 
* \return
* void
*
*******************************************************************************/
void updateMotorsGatt(motors_t motor,uint8_t percent,uint8_t flags)
{    
    cy_stc_ble_gatt_handle_value_pair_t myHvp;
    
    if(percent<0)
        percent = 0;
    if(percent>100)
        percent = 100;
  
    switch(motor)
    {
        case M1:
            myHvp.attrHandle = CY_BLE_MOTOR_M1_CHAR_HANDLE;
        break;
        case M2:
            myHvp.attrHandle = CY_BLE_MOTOR_M2_CHAR_HANDLE;
        break;
    }

    myHvp.value.val = (uint8_t *)&percent;
    myHvp.value.actualLen = 1;
    myHvp.value.len = 1;

    if(flags == CY_BLE_GATT_DB_PEER_INITIATED)
    {
        Cy_BLE_GATTS_WriteAttributeValuePeer(&cy_ble_connHandle[0],&myHvp);
        PWM_Message_t myMessage;
        myMessage.motor = motor;
        myMessage.percent = percent;
        xQueueSend(pwmQueue,&myMessage,0); /// ARH might think about a different timeout  
    }
    else
    {
        Cy_BLE_GATTS_WriteAttributeValueLocal(&myHvp);   
        Cy_BLE_GATTS_SendNotification (&cy_ble_connHandle[0], &myHvp);  
    }
}

/*******************************************************************************
* Function Name: customEventHandler
********************************************************************************
*
* This function is the BLE Event Handler callback.
*
* \param 
*  event
* \param
*  eventParameter
* 
* \return
* void
*
*******************************************************************************/
void customEventHandler(uint32_t event, void *eventParameter)
{
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;   
     
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            Cy_GPIO_Write(LED8_PORT,LED8_NUM,1);
            printf("BLE: ON/Disconnect\n");
        break;
         

        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("BLE: Connection\n");
            Cy_GPIO_Write(LED8_PORT,LED8_NUM,0); 
            updateMotorsGatt(M1,getMotorPercent(M1),CY_BLE_GATT_DB_LOCALLY_INITIATED);
            updateMotorsGatt(M2,getMotorPercent(M2),CY_BLE_GATT_DB_LOCALLY_INITIATED);
        break;
        
        
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) eventParameter;
   
            if(CY_BLE_MOTOR_M1_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {
               updateMotorsGatt(M1,writeReqParameter->handleValPair.value.val[0],CY_BLE_GATT_DB_PEER_INITIATED);
            }
            
            if(CY_BLE_MOTOR_M2_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {     
               updateMotorsGatt(M2,writeReqParameter->handleValPair.value.val[0],CY_BLE_GATT_DB_PEER_INITIATED);
            }
            
            if(CY_BLE_MOTOR_M1_REL_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {
                PWM_Message_t myMessage;
                myMessage.motor = M1;
                myMessage.percent = -1;
                myMessage.percentChange = (uint8_t)writeReqParameter->handleValPair.value.val[0];
                xQueueSend(pwmQueue,&myMessage,0); /// ARH might think about a different timeout            
            }
            
            if(CY_BLE_MOTOR_M2_REL_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {     
                PWM_Message_t myMessage;
                myMessage.motor = M2;
                myMessage.percent = -1;
                myMessage.percentChange = (uint8_t)writeReqParameter->handleValPair.value.val[0];
                xQueueSend(pwmQueue,&myMessage,0);            
            }
            
            if(CY_BLE_MOTOR_M1_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE  ==  writeReqParameter->handleValPair.attrHandle ||
                CY_BLE_MOTOR_M2_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE  ==  writeReqParameter->handleValPair.attrHandle
            )
            { 
                cy_stc_ble_gatts_db_attr_val_info_t myWrite;
                myWrite.offset = 0;
                myWrite.flags = CY_BLE_GATT_DB_PEER_INITIATED;
                myWrite.connHandle = writeReqParameter->connHandle;
                myWrite.handleValuePair = writeReqParameter->handleValPair;
                
                Cy_BLE_GATTS_WriteAttributeValueCCCD(&myWrite);
            }
            
            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;

        /* Do nothing for all other events */
        default:
        break;
    }
}

/*******************************************************************************
* Function Name: bleInterruptNotify
********************************************************************************
*
* This function is the ISR which is called when the BLE stack need to do somethign
* All it does is unlock the semaphore to get the BLE processing events 
*
* \param 
* void - None
* \return
* void - None
*
*******************************************************************************/

void bleInterruptNotify()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken); 
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: bleTask
********************************************************************************
*
* This function is the the main task for running BLE.  It is launched by the FreeRTOS
* task scheduler in main_cm4.c
*
* \param 
* void * arg is not used by this task
* \return
* void - this function never returns
*
*******************************************************************************/

void bleTask(void *arg)
{
    (void)arg;
    printf("Started BLE\n");

    bleSemaphore = xSemaphoreCreateCounting(0xFFFFFFFF,0);
    
    Cy_BLE_Start(customEventHandler);
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
   
    // Get the stack started...
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }

    for(;;)
    {
        xSemaphoreTake(bleSemaphore,portMAX_DELAY);
        Cy_BLE_ProcessEvents();
        
        // If the PWM tasks says we need to update the GATT database and send out the motor percent
        // then do it.. 
        if(xEventGroupGetBits(pwmEventGroup) & PWM_EVENT_BLE)
        {
            xEventGroupClearBits(pwmEventGroup,PWM_EVENT_BLE);
            updateMotorsGatt(M1,getMotorPercent(M1),CY_BLE_GATT_DB_LOCALLY_INITIATED);
            updateMotorsGatt(M2,getMotorPercent(M2),CY_BLE_GATT_DB_LOCALLY_INITIATED);
        }
    }
}
