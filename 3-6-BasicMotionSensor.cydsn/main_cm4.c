#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "bmi160.h"

static struct bmi160_dev bmi160Dev;

static int8_t BMI160BurstWrite(uint8_t dev_addr, uint8_t reg_addr,uint8_t *data, uint16_t len)
{
    
    Cy_SCB_I2C_MasterSendStart(I2C_1_HW,dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_1_context);
    Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,reg_addr,0,&I2C_1_context);
    for(int i = 0;i<len; i++)
    { 
        Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,data[i],0,&I2C_1_context);
    }
    
    Cy_SCB_I2C_MasterSendStop(I2C_1_HW,0,&I2C_1_context);
    
    return 0;
}

// This function supports the BMP180 library and read I2C Registers
static int8_t BMI160BurstRead(uint8_t dev_addr, uint8_t reg_addr,uint8_t *data, uint16_t len)
{
    
    Cy_SCB_I2C_MasterSendStart(I2C_1_HW,dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_1_context);
    Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,reg_addr,0,&I2C_1_context);
    Cy_SCB_I2C_MasterSendReStart(I2C_1_HW,dev_addr,CY_SCB_I2C_READ_XFER,0,&I2C_1_context);
    for(int i = 0;i<len-1; i++)
    {
        Cy_SCB_I2C_MasterReadByte(I2C_1_HW,CY_SCB_I2C_ACK,&data[i],0,&I2C_1_context);
    }
    Cy_SCB_I2C_MasterReadByte(I2C_1_HW,CY_SCB_I2C_NAK,&data[len-1],0,&I2C_1_context);
    
    Cy_SCB_I2C_MasterSendStop(I2C_1_HW,0,&I2C_1_context);
    
    
    return 0;
}


static void sensorsDeviceInit(void)
{

  int8_t rslt;
  vTaskDelay(500); // guess

  /* BMI160 */
  // assign bus read function
  bmi160Dev.read = (bmi160_com_fptr_t)BMI160BurstRead;
  // assign bus write function
  bmi160Dev.write = (bmi160_com_fptr_t)BMI160BurstWrite;
  // assign delay function
  bmi160Dev.delay_ms = (bmi160_delay_fptr_t)vTaskDelay;
  bmi160Dev.id = BMI160_I2C_ADDR;  // I2C device address

  rslt = bmi160_init(&bmi160Dev); // initialize the device
  if (rslt == 0)
    {
      printf("BMI160 I2C connection [OK].\n");
      /* Select the Output data rate, range of Gyroscope sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */
      bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
      bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
      bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

      /* Select the power mode of Gyroscope sensor */
      bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

      bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
      bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
      bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
      bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Set the sensor configuration */
      rslt |= bmi160_set_sens_conf(&bmi160Dev);
      bmi160Dev.delay_ms(50);
    }
  else
    {
      printf("BMI160 I2C connection [FAIL].\n");
    }
}
#define MAXACCEL 8000
void motionTask(void *arg)
{
    (void)arg;
    I2C_1_Start();
    sensorsDeviceInit();
    struct bmi160_sensor_data acc;
    
    float m1,m2;
    float iirm1 = 50.0;
    float iirm2 = 50.0;
    
    while(1)
    {
        
        bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &acc, NULL, &bmi160Dev);
        
        acc.x = (acc.x>MAXACCEL)?MAXACCEL:acc.x;
        acc.x = (acc.x<-MAXACCEL)?-MAXACCEL:acc.x;

        acc.y = (acc.y>MAXACCEL)?MAXACCEL:acc.y;
        acc.y = (acc.y<-MAXACCEL)?-MAXACCEL:acc.y;
  
        m1 = acos(acc.y  / MAXACCEL)*360/(2*3.14*1.8);
        m2 = acos(acc.x  / MAXACCEL)*360/(2*3.14*1.8);
        // IIR Filter to get rid of noise
        iirm1 = m1 / 8.0 + iirm1*7.0/8.0;
        iirm2 = m2 / 8.0 + iirm2*7.0/8.0;
        
        printf("x=%4d y=%4d z=%4d m1=%3.0f m2=%2.0f\r\n",acc.x,acc.y,acc.z,iirm1,iirm2);
        
        vTaskDelay(200);
    }
}



int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    UART_1_Start();

    xTaskCreate( motionTask, "Motion Task",400,0,1,0);
    vTaskStartScheduler();

    while(1);
}

/* [] END OF FILE */
