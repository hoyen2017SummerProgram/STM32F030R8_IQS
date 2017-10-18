/******************************************************************************
*                                                                             *
*                            Module specification                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              Azoteq (Pty) Ltd                               *
*                          Republic of South Africa                           *
*                                                                             *
*                           Tel: +27(0)21 863 0033                            *
*                          E-mail: info@azoteq.com                            *
*                                                                             *
*******************************************************************************
Name             :  IQS316.c
Description      :  IQS316 specific functions for SPI/I2C Firmware library
*******************************************************************************/


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      revision   :  IQS316 SPI/I2C Firmware library rev 1.01
          date   :  2013-12-03
            by   :  Robert Samuel
   description   :  Basic functions to implement IQS316 via SPI/I2C (PIC18F4550 was used)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "iqs316_driver.h"
#include "stdio.h"
#define DEBUG_IQS316

/*
********************************************************************************
*		GLOBAL VARIABLES
********************************************************************************
*/

struct tIQS316 IQS316;

//*****************************************************************************
//
//! Configure the on-chip IQS316 settings
//!
//! This function is modified to suit the design requirements.  The basic
//! procedure remains the same, namely to configure the channels used, followed
//! by setting up the ATI parameters and executing the auto-ATI algorithm, then
//! followed by configuring the thresholds, followed by all the other required
//! settings such as Low-Power configuration, charging modes used etc.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void IQS316_Settings(void)
{
    uint8_t ui8StartGroup, ui8CurrentGroup;
    uint8_t ui8ProdNo, ui8VersionNo;
    uint8_t ui8DataArray[20];
    //
    // Confirm comms are working correctly, and also that expected IQS316
    // IC version is used.  Do this by reading back the Product and Version
    // numbers from the IQS316
    //
    IQS316_Read(PROD_NUM, ui8DataArray, 2);
    ui8ProdNo = ui8DataArray[0];
    ui8VersionNo = ui8DataArray[1];
    IQS316_End_Comms_Window();

#ifdef DEBUG_IQS316
    if((ui8ProdNo != 27) || (ui8VersionNo != 1))
    {
        // Error condition, handle this here
        // (fix comms or get correct IQS316 version)
        //
        while(1);
    }
#endif
    //
    // Acknowledge the reset by sending an ACK_RESET to the IQS316.  This will
    // clear the SHOW_RESET bit in UI_FLAGS0 register.  From here on further, if
    // this SHOW_RESET bit ever becomes set, we know an unexpected reset has
    // occurred on the IQS316, and we should repeat the setup
    //
    ui8DataArray[0] = (ACK_RESET | LTN_DISABLE | WDT_DISABLE);
    IQS316_Write(PROX_SETTINGS_2, ui8DataArray, 1);
    IQS316_End_Comms_Window();

#ifdef DEBUG_IQS316
    IQS316_Read(UI_FLAGS0, ui8DataArray, 1);
    IQS316_End_Comms_Window();

    if((ui8DataArray[0] & SHOW_RESET) != 0)
    {
        // The show reset bit should be cleared after writing the ACK_RESET
        // previously.  Check write procedures, and make sure comms window is
        // closed after sending ACK_RESET.
        while(1);
    }
#endif
    //
    // IQS316 Application specific SETUP
    // 1 - CHANNEL SETUP
    //
    ui8DataArray[0] = 0x03;         // CHAN_ACTIVE0
    ui8DataArray[1] = 0x0F;         // CHAN_ACTIVE1
    ui8DataArray[2] = 0x0F;         // CHAN_ACTIVE2
    ui8DataArray[3] = 0x0F;         // CHAN_ACTIVE3
    ui8DataArray[4] = 0x0F;         // CHAN_ACTIVE4

    IQS316_Write(CHAN_ACTIVE0, ui8DataArray, 5);
    IQS316_End_Comms_Window();
    //
    // 2 - Setup ATI and thresholds (settings which must be sent in specific
    // comms window - depending which group is active)
    //
    IQS316_Read(GROUP_NUM, ui8DataArray, 1);
    ui8StartGroup = ui8DataArray[0];
    //
    // Enable skip conversions, so that IQS316 cycles through the the groups
    // 0, 1, 2, 3, 4, 0, 1, ....   to allow configuring settings which must be
    // setup while in a specific cycle.
    //
    ui8DataArray[0] = (SKIP_CONV | LTN_DISABLE | WDT_DISABLE);
    IQS316_Write(PROX_SETTINGS_2, ui8DataArray, 1);
    ui8CurrentGroup = ui8StartGroup;

    do
    {
        switch(ui8CurrentGroup)
        {
            case 0:
            {
                // ATI C and ATI I settings
                //
                ui8DataArray[0] = 0x00;          // ATI_MULT1
                ui8DataArray[1] = 0x00;          // ATI_MULT2
                IQS316_Write(ATI_MULT1, ui8DataArray, 2);
                //
                // Set thresholds (in upper nibble of LTA)
                // NOTE: this will overwrite the LTA value also, but auto-ATI
                // will be done later, which will reseed the LTAs correctly
                //
                ui8DataArray[0] = PROX_THRES_8;  // LTA_04_HI
                ui8DataArray[1] = 0x00;          // low byte - irrelevant
                ui8DataArray[2] = PROX_THRES_8;  // LTA_15_HI
                ui8DataArray[3] = 0x00;          // low byte - irrelevant
                ui8DataArray[4] = PROX_THRES_8;  // LTA_26_HI
                ui8DataArray[5] = 0x00;          // low byte - irrelevant
                ui8DataArray[6] = PROX_THRES_8;  // LTA_37_HI
                IQS316_Write(LTA_04_HI, ui8DataArray, 7);
                break;
            }
            case 1:
            {
                ui8DataArray[0] = 0x00;          // ATI_MULT1
                ui8DataArray[1] = 0x00;          // ATI_MULT2
                IQS316_Write(ATI_MULT1, ui8DataArray, 2);

                ui8DataArray[0] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_04_HI
                ui8DataArray[1] = 0x00;          // low byte - irrelevant
                ui8DataArray[2] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_15_HI
                ui8DataArray[3] = 0x00;          // low byte - irrelevant
                ui8DataArray[4] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_26_HI
                ui8DataArray[5] = 0x00;          // low byte - irrelevant
                ui8DataArray[6] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_37_HI
                IQS316_Write(LTA_04_HI, ui8DataArray, 7);
                break;
            }
            case 2:
            {
                ui8DataArray[0] = 0x00;          // ATI_MULT1
                ui8DataArray[1] = 0x00;          // ATI_MULT2
                IQS316_Write(ATI_MULT1, ui8DataArray, 2);

                ui8DataArray[0] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_04_HI
                ui8DataArray[1] = 0x00;          // low byte - irrelevant
                ui8DataArray[2] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_15_HI
                ui8DataArray[3] = 0x00;          // low byte - irrelevant
                ui8DataArray[4] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_26_HI
                ui8DataArray[5] = 0x00;          // low byte - irrelevant
                ui8DataArray[6] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_37_HI
                IQS316_Write(LTA_04_HI, ui8DataArray, 7);
                break;
            }
            case 3:
            {
                ui8DataArray[0] = 0x00;          // ATI_MULT1
                ui8DataArray[1] = 0x00;          // ATI_MULT2
                IQS316_Write(ATI_MULT1, ui8DataArray, 2);

                ui8DataArray[0] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_04_HI
                ui8DataArray[1] = 0x00;          // low byte - irrelevant
                ui8DataArray[2] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_15_HI
                ui8DataArray[3] = 0x00;          // low byte - irrelevant
                ui8DataArray[4] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_26_HI
                ui8DataArray[5] = 0x00;          // low byte - irrelevant
                ui8DataArray[6] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_37_HI
                IQS316_Write(LTA_04_HI, ui8DataArray, 7);
                break;
            }
            case 4:
            {
                ui8DataArray[0] = 0x00;          // ATI_MULT1
                ui8DataArray[1] = 0x00;          // ATI_MULT2
                IQS316_Write(ATI_MULT1, ui8DataArray, 2);

                ui8DataArray[0] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_04_HI
                ui8DataArray[1] = 0x00;          // low byte - irrelevant
                ui8DataArray[2] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_15_HI
                ui8DataArray[3] = 0x00;          // low byte - irrelevant
                ui8DataArray[4] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_26_HI
                ui8DataArray[5] = 0x00;          // low byte - irrelevant
                ui8DataArray[6] = PROX_THRES_20 | TOUCH_THRES_3_16;  // LTA_37_HI
                IQS316_Write(LTA_04_HI, ui8DataArray, 7);
                break;
            }
        }
        IQS316_End_Comms_Window();

        IQS316_Read(GROUP_NUM, ui8DataArray, 1);
        ui8CurrentGroup = ui8DataArray[0];
    } while (ui8CurrentGroup != ui8StartGroup);
    //
    // Now Group specific settings are done, so disable the skip conversions
    //
    ui8DataArray[0] = (LTN_DISABLE | WDT_DISABLE);
    IQS316_Write(PROX_SETTINGS_2, ui8DataArray, 1);
    //
    // Set the high/low settings for prox and touch thresholds
    //
    ui8DataArray[0] = (PROX_THRES_RANGE | ND);
    IQS316_Write(UI_SETTINGS0, ui8DataArray, 1);
    //
    // Set ATI Target - For Prox Mode
    //
    ui8DataArray[0] = 0x03;
    ui8DataArray[1] = 0x20;
    IQS316_Write(AUTO_ATI_TARGET_HI, ui8DataArray, 2);
    IQS316_End_Comms_Window();
    //
    // Perform automated ATI routine (to setup ATI Compensation values)
    // NOTE: ATI_MODE already set to ProxMode, no need to configure.
    //
    ui8DataArray[0] = CXVSS | HALT0 | AUTO_ATI | CXDIV1;
    IQS316_Write(PROX_SETTINGS_1, ui8DataArray, 1);
    IQS316_End_Comms_Window();
    //
    // Read ATI Busy flag until it clears, then ProxMode ATI is done
    //
    do
    {
        IQS316_Read(UI_FLAGS0, ui8DataArray, 1);
        IQS316_End_Comms_Window();

    } while ((ui8DataArray[0] & ATI_BUSY) != 0);
    //
    // Perform ATI for Touch Mode
    // Set ATI_MODE to Touch
    //
    ui8DataArray[0] = ATI_MODE | PROX_THRES_RANGE | ND;
    IQS316_Write(UI_SETTINGS0, ui8DataArray, 1);
    IQS316_End_Comms_Window();
    //
    // Set ATI Target - For Touch Mode
    //
    ui8DataArray[0] = 0x03;
    ui8DataArray[1] = 0x20;
    IQS316_Write(AUTO_ATI_TARGET_HI, ui8DataArray, 2);
    IQS316_End_Comms_Window();
    //
    // Perform automated ATI routine (to setup ATI Compensation values)
    //
    ui8DataArray[0] = CXVSS | HALT0 | AUTO_ATI | CXDIV1;
    IQS316_Write(PROX_SETTINGS_1, ui8DataArray, 1);
    IQS316_End_Comms_Window();
    //
    // Read ATI Busy flag until it clears, then ATI is done
    //
    do
    {
        IQS316_Read(UI_FLAGS0, ui8DataArray, 1);
        IQS316_End_Comms_Window();

    } while ((ui8DataArray[0] & ATI_BUSY) != 0);
    //
    // Now setup the advanced settings as required by the design, such as the
    // following:  Low-Power, charging mode, eventMode
    //
}

//*****************************************************************************
//
//! Obtain new data from the IQS316
//!
//! This fucntion reads the group number  to determine whether the IQS316 is in
//! PROX mode or TOUCH mode. If the group number is 0, PROX mode, the
//! prox_detected flag is cleared and the routine finishes. If the group number
//! is not 0, with Auto-Mode assumed, then the prox_detected flag is set.  TOUCH
//! and PROX status registers for each group are read and stored in the global
//! data structures.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void IQS316_Refresh_Data(void)
{
    uint8_t ui8CurrentGroup, ui8TempTouch, ui8TempProx;
    uint8_t ui8DataArray[5], ui8TempUIFlags0;

    IQS316_ReadCurrentAddress(ui8DataArray, 5);
    //
    // Comms window is now ended. Note if other data is required then obtain
    // this before ending the window
    //
    IQS316_End_Comms_Window();
    //
    // Temporarily store the received data
    //
    ui8TempUIFlags0 = ui8DataArray[0];
    ui8TempProx =ui8DataArray[1];
    ui8TempTouch =ui8DataArray[2];
    ui8CurrentGroup =ui8DataArray[4];
    //
    // Make sure an unexpected reset has not occurred
    //
    if((ui8TempUIFlags0 & SHOW_RESET) != 0)
    {
        // handle reset here, suggestion is to repeat IQS316 init
        //
        IQS316_Settings();
    }

    //
    // Here an example is given of how the data can be placed into an IQS316
    // structure.  This is purely for example purposes
    //
    if(ui8CurrentGroup == 0)
    {
        if(ui8TempProx == 0)
        {
            IQS316.prox_detected = 0;
        }
        else
        {
            IQS316.prox_detected = 1;
        }
    }
    else
    {
        // Update the specific groups data
        //
        switch(ui8CurrentGroup)
        {
            case 1:
                IQS316.prox4_11 &= 0xF0;
                IQS316.touch4_11 &= 0xF0;

                IQS316.prox4_11 |= (ui8TempProx & 0x0F);
                IQS316.touch4_11 |= (ui8TempTouch & 0x0F);
                break;
            case 2:
                IQS316.prox4_11 &= 0x0F;
                IQS316.touch4_11 &= 0x0F;

                IQS316.prox4_11 |= ((ui8TempProx & 0x0F) << 4);
                IQS316.touch4_11 |= ((ui8TempTouch & 0x0F) << 4);
                break;
            case 3:
                IQS316.prox12_19 &= 0xF0;
                IQS316.touch12_19 &= 0xF0;

                IQS316.prox12_19 |= (ui8TempProx & 0x0F);
                IQS316.touch12_19 |= (ui8TempTouch & 0x0F);
                break;
            case 4:
                IQS316.prox12_19 &= 0x0F;
                IQS316.touch12_19 &= 0x0F;

                IQS316.prox12_19 |= ((ui8TempProx & 0x0F) << 4);
                IQS316.touch12_19 |= ((ui8TempTouch & 0x0F) << 4);
                break;
        }
    }
}

//*****************************************************************************
//
//! Process the new data from the IQS316
//!
//! Function to be called after obtaining new data from the IQS316
//! This function processes the data contained in the IQS316 struct, and is
//! only done here as an example.  The channel number that is in touch is
//! displayed on 4 I/O's (channel 4-19 displayed as value 0-15 on PD7..4)
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void
IQS316_Process_Data(void)
{
    uint16_t ui16TempTouch;
    //
    // Place code here to process data available in the IQS316 structure.
    // this example places the binary value of the pressed button on 4 LEDs
    //

    //
    // place the touch bits 4 to 19 into a word
    //
    ui16TempTouch = (uint16_t)IQS316.touch4_11 | (((uint16_t)IQS316.touch12_19)<<8);

    printf("touch value= %d", ui16TempTouch);

}

//*****************************************************************************
//
//! Perform an I2C read from the IQS316 slave device
//!
//! The function waits for RDY to be set, to show a comms window is active, and
//! then gives a START, ADR+WRITE, ADR, REPEAT-START, ADR+READ, DATA1..DATA2..
//! NOTE: no i2c STOP is given, since this is handled seperately to control the
//! IQS316 comms window.
//!
//! \param ui8Address is the address on the IQS316 from which to read
//! \param ui8Data[] is the array where the read data is packed into
//! \param ui8Length is the number of bytes to read from the IQS316
//!
//! \return None
//
//*****************************************************************************
uint8_t IQS316_Read(uint8_t ui8Address, uint8_t *ui8Data, uint8_t ui8Length)
{
 uint32_t I2C_TimeOut = 0x2000;
   /* Test on BUSY Flag */
  while(I2C_GetFlagStatus(TP_I2C, I2C_ISR_BUSY) != RESET)
  {
    if((I2C_TimeOut--) == 0) 
      return 0; // sEE_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(TP_I2C, TP_SLAVE_READ_ADDRESS, ui8Length, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  
  /* Wait until all data are received */
  while (ui8Length)
  {   
    /* Wait until RXNE flag is set */
    I2C_TimeOut = 0x3000;
    while(I2C_GetFlagStatus(TP_I2C, I2C_ISR_RXNE) == RESET)    
    {
      if((I2C_TimeOut--) == 0) 
        return 1;//sEE_TIMEOUT_UserCallback();
    }
    
    /* Read data from RXDR */
    *ui8Data = I2C_ReceiveData(TP_I2C);
    /* Point to the next location where the byte read will be saved */
    ui8Data++;
    
    /* Decrement the read bytes counter */
    ui8Length--;
  } 
  
  /* Wait until STOPF flag is set */
  I2C_TimeOut = 0x3000;
  while(I2C_GetFlagStatus(TP_I2C, I2C_ISR_STOPF) == RESET)   
  {
    if((I2C_TimeOut--) == 0)
      return 1; //sEE_TIMEOUT_UserCallback();
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(TP_I2C, I2C_ICR_STOPCF);
  
  /* If all operations OK */
  return 0;
}

//*****************************************************************************
//
//! Perform an I2C read from the IQS316 slave device
//!
//! The function waits for RDY to be set, to show a comms window is active, and
//! then gives a START, ADR+READ, DATA1..DATA2..
//! NOTE: no i2c STOP is given, since this is handled seperately to control the
//! IQS316 comms window.  Also no address is written, the data is read from the
//! current address pointer position.  This function is mainly used to read each
//! cycles data, since the default pointer is already at the correct position.
//!
//! \param ui8Address is the address on the IQS316 from which to read
//! \param ui8Data[] is the array where the read data is packed into
//! \param ui8Length is the number of bytes to read from the IQS316
//!
//! \return None
//
//*****************************************************************************
uint8_t IQS316_ReadCurrentAddress(uint8_t *ui8Data, uint8_t ui8Length)
{

  return 0;
}

//*****************************************************************************
//
//! Perform an I2C write to the IQS316 slave device
//!
//! The function waits for RDY to be set, to show a comms window is active, and
//! then gives a START, ADR+WRITE, ADR, DATA1..DATA2..
//! NOTE: no i2c STOP is given, since this is handled seperately to control the
//! IQS316 comms window.
//!
//! \param ui8Address is the address on the IQS316 where the data is to be written
//! \param ui8Data[] is the array where the data is pre-loaded
//! \param ui8Length is the number of bytes to write to the IQS316
//!
//! \return None
//
//*****************************************************************************
uint8_t IQS316_Write(uint8_t ui8Address, uint8_t *ui8Data, uint8_t ui8Length)
{
  return 0;					
}

//*****************************************************************************
//
//! End the comms window with the IQS316
//!
//! The IQS316 comms window is ended by sending an I2C STOP condition.  This
//! function then also waits for RDY to go LOW to make sure a following comms
//! does not catch the RDY high just before it is cleared by the IQS316.
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void IQS316_End_Comms_Window(void)
{
  uint16_t I2C_TimeOut = 300;
  while(I2C_GetFlagStatus(TP_I2C, I2C_ISR_STOPF) == RESET)   
  {
    if((I2C_TimeOut--) == 0)
      break; //sEE_TIMEOUT_UserCallback();
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(TP_I2C, I2C_ICR_STOPCF);
}

