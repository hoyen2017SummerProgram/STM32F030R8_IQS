#include "IQS316.h"
#include "hwcfg.h"
#include "stdint.h"

#define IQS316_ADDR         0x74
#define TP_SLAVE_READ_ADDRESS         0x74
#define RDY                 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)    //TODO : 改成用的GPIO
#define TP_I2C_TIMING           0x00501953


struct tIQS316 {
    unsigned char prox_detected;
    unsigned char prox4_11;
    unsigned char prox12_19;
    unsigned char touch4_11;
    unsigned char touch12_19;
};

// IQS316.c prototypes
void IQS316_Settings(void);
void IQS316_Refresh_Data(void);
void IQS316_Process_Data(void);

uint8_t IQS316_Read(uint8_t ui8Address, uint8_t *ui8Data, uint8_t ui8Length);
uint8_t IQS316_ReadCurrentAddress(uint8_t *ui8Data, uint8_t ui8Length);
uint8_t IQS316_Write(uint8_t ui8Address, uint8_t *ui8Data, uint8_t ui8Length);
void IQS316_End_Comms_Window(void);

// lower level PIC18F4550.c prototypes
void IQS316_Init(void);
void Comms_init(void);
//void CommsIQS316_send(uint8_t send_data);
//uint8_t CommsIQS316_read_ack(void);
//uint8_t CommsIQS316_read_nack(void);
//void CommsIQS316_start(void);
//void CommsIQS316_stop(void);
//void CommsIQS316_repeat_start(void);
void DebugLED(uint8_t ui8LEDState, uint8_t ui8LEDNumber);

void DelayMS(uint8_t ui8Milliseconds);

// =============================================================================
// =========== PURELY FOR DEBUGGING - not part of library ======================

#define LED1                (uint8_t)0x01        // PD0
#define LED2                (uint8_t)0x02        // PD1
#define LED3                (uint8_t)0x04        // PD2
#define LED4                (uint8_t)0x08        // PD3

#define SET_LED              0x00
#define CLEAR_LED            0x01

#define DEBUG1                (uint8_t)0x40        // PD6
#define DEBUG2                (uint8_t)0x80        // PD7

#define SET_IO              0x00
#define CLEAR_IO            0x01