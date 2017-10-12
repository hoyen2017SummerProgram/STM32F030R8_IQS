/******************************************************************************
*                                                                             *
*                            Module specification                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              HOYEN TECH Co., Ltd                            *
*                                                                             *
*******************************************************************************
Name             :  IQS316_driver.c
Description      :  STM32M0 specific functions for IQS316 I2C Firmware library
*******************************************************************************/

#include "iqs316_driver.h"
#include "stdio.h"
//*****************************************************************************
//
//! Initialise
//!
//! Initializes the STM32F0 I2C
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void IQS316_Init(void)
{
    Comms_init();
    //
    // Add delay here to allow IQS316 to startup
    // According to datasheet first comms is available roughly 16ms
    // after MCLR is released
    //
    delay_msSysTick(15);

    // Place other functions responsible for hardware initialization here.

    IQS316_Settings();
}

//*****************************************************************************
//
//! I2C Initialise
//!
//! Initializes the I2C module on the PIC18F4550 
//! Note that the SSPADD acts as a counter to determine the I2C frequency. A   
//! smaller value will increase the frequency.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void Comms_init()
{
//     原先程式碼
//    TRISB = TRISB | 0x03;	//set TRISB<1:0> SDA and SCL
//    TRISA = TRISA | 0x02;	//I2C ready line input

//    PIR1bits.SSPIF = 0;		//clear I2C interrupt flag

//    SSPADD = 0x1C;		//settings for I2C frequency - 416kHz
//    SSPSTAT |= 0x80;		//slew rate control for high speed (400kHz)

//    SSPCON1 = 0x28;		//enables I2C module on the PIC18F4550

//    LATB = LATB | 0x04;		//IQS316 MCLR High
  I2C_InitTypeDef  I2C_InitStructure;
  
  HW_TP_LowLevel_Init();
  //TODO: 1.設定 I2C_ready GPIO input，如同按鈕
  //      2.設定 MCLR High
  
  
  
  /* I2C configuration */
  /* sEE_I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = TP_I2C_TIMING;
  
  /* Apply sEE_I2C configuration after enabling it */
  I2C_Init(TP_I2C, &I2C_InitStructure);
   
  /* sEE_I2C Peripheral Enable */
  I2C_Cmd(TP_I2C, ENABLE);
}

void DebugLED(uint8_t ui8LEDState, uint8_t ui8LEDNumber)
{
   if(ui8LEDState == CLEAR_LED)
   {
      // LED is inactive on an I/O HIGH state (sink)
      //
      LED_On(LED1);
   }
   else
   {
       // LED is active on an I/O LOW state (sink)
      LED_Off(LED1);
   }
}

void
DebugIO(uint8_t ui8IOState, uint8_t ui8IONumber)
{
   if(ui8IOState == SET_IO)
   {
     printf("debug IO set");
   }
   else
   {
      printf("debug IO reset");
   }
}


