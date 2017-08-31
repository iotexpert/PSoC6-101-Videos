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


// A temporary extern for the internal array of connection handles
extern cy_stc_ble_conn_handle_t cy_ble_connHandle[CY_BLE_CONN_COUNT];

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
void updateMotorsGatt(motors_t motor,uint8_t percent,uint8_t flags,cy_stc_ble_conn_handle_t connectionHandle)
{    
    cy_ble_gatt_db_attr_handle_t cccdHandle;
    cy_stc_ble_gatts_db_attr_val_info_t myWrite;
         
    memset(&cccdHandle,0,sizeof(cccdHandle));
    
    if(percent<0)
        percent = 0;
    if(percent>100)
        percent = 100;
  
    switch(motor)
    {
        case M1:
            myWrite.handleValuePair.attrHandle = CY_BLE_MOTOR_M1_CHAR_HANDLE;
            cccdHandle = CY_BLE_MOTOR_M1_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE;
        break;
        case M2:
            myWrite.handleValuePair.attrHandle = CY_BLE_MOTOR_M2_CHAR_HANDLE;
            cccdHandle = CY_BLE_MOTOR_M2_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE;
        break;
    }
    
    
    myWrite.offset = 0;
    myWrite.flags = flags;
    myWrite.connHandle = connectionHandle;            
    myWrite.handleValuePair.value.val = (uint8_t *)&percent;
    myWrite.handleValuePair.value.actualLen = 1;
    myWrite.handleValuePair.value.len = 1;
    
    Cy_BLE_GATTS_WriteAttributeValue(&myWrite);

    if(flags == CY_BLE_GATT_DB_PEER_INITIATED)
    {
        PWM_Message_t myMessage;
        myMessage.motor = motor;
        myMessage.percent = percent;
        xQueueSend(pwmQueue,&myMessage,0); /// ARH might think about a different timeout  
    }
    
    // If notifications are on... then send notification
    cy_stc_ble_gatts_db_attr_val_info_t 	param;
    uint8_t cccd[2];
    param.connHandle = cy_ble_connHandle[0];
    param.handleValuePair.attrHandle = cccdHandle;
    param.flags = CY_BLE_GATT_DB_LOCALLY_INITIATED;
    param.offset = 0;  
    param.handleValuePair.value.val = cccd;
    param.handleValuePair.value.len = 2;
    param.handleValuePair.value.actualLen = 2;
   
    Cy_BLE_GATTS_ReadAttributeValueCCCD(&param);
        
    if(param.handleValuePair.value.val[0] & 0x01) // if CCCD is on... notify.
    {   cy_stc_ble_gatts_handle_value_ntf_t v1;
        v1.connHandle = cy_ble_connHandle[0];
        v1.handleValPair = myWrite.handleValuePair;
        Cy_BLE_GATTS_Notification(&v1);
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
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

        #if 0
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            
            /* Check if the advertisement has stopped */
            if (Cy_BLE_GetState() == CY_BLE_STATE_STOPPED) // if it is stopped... restart the advertising
            {
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            }
            break;
        #endif
         
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
          
            updateMotorsGatt(M1,getMotorPercent(M1),CY_BLE_GATT_DB_LOCALLY_INITIATED,*(cy_stc_ble_conn_handle_t *) eventParameter);
            updateMotorsGatt(M2,getMotorPercent(M2),CY_BLE_GATT_DB_LOCALLY_INITIATED,*(cy_stc_ble_conn_handle_t *) eventParameter);

        break;
        
        
        case CY_BLE_EVT_GATTS_WRITE_REQ:
          
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) 
                                eventParameter;
   
            if(CY_BLE_MOTOR_M1_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {
               updateMotorsGatt(M1,writeReqParameter->handleValPair.value.val[0],CY_BLE_GATT_DB_PEER_INITIATED,writeReqParameter->connHandle);
            }
            
            if(CY_BLE_MOTOR_M2_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {     
               updateMotorsGatt(M2,writeReqParameter->handleValPair.value.val[0],CY_BLE_GATT_DB_PEER_INITIATED,writeReqParameter->connHandle);
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
    cy_stc_ble_conn_handle_t zeroHandle;
    memset(&zeroHandle,0,sizeof(zeroHandle));
    
    Cy_BLE_Start(customEventHandler);
    
    for(;;)
    {
        Cy_BLE_ProcessEvents();
        
        // If the PWM tasks says we need to update the GATT database and send out the motor percent
        // then do it.. 
        if(xEventGroupGetBits(pwmEventGroup) & PWM_EVENT_BLE)
        {
            xEventGroupClearBits(pwmEventGroup,PWM_EVENT_BLE);
            updateMotorsGatt(M1,getMotorPercent(M1),CY_BLE_GATT_DB_LOCALLY_INITIATED,zeroHandle);
            updateMotorsGatt(M2,getMotorPercent(M2),CY_BLE_GATT_DB_LOCALLY_INITIATED,zeroHandle);
        }
        vTaskDelay(5); // not very happy about this 5.. not sure what the right thing to do is
    }
}
