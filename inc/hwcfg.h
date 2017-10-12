/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWCFG_H
#define __HWCFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/** @addtogroup WDL_HW_LOW_LEVEL
  * @{
  */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3,
  LED5 = 4,
  LED6 = 5,
  LED7 = 6,
  LED8 = 7,
  LED9 = 8,
  LED0 = 9,         // 0
  LEDA = 10,         // C
  LEDB = 11,         // #
  LEDC = 12,         // nodify 1
  LEDD = 13,         // nodify 2
  LEDE = 14,          // Low Battery
  LEDALL = 15
} Led_TypeDef;

typedef enum
{
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;

#ifdef DISCOVERY030
#define LEDn                             2
#define LED1_PIN                         GPIO_Pin_9
#define LED1_GPIO_PORT                   GPIOC
#define LED1_GPIO_CLK                    RCC_AHBPeriph_GPIOC

#define LED2_PIN                         GPIO_Pin_8
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOC
#else
#define LEDn                             1

#define LED1_PIN                         GPIO_Pin_5
#define LED1_GPIO_PORT                   GPIOA
#define LED1_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LED2_PIN                         GPIO_Pin_7
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#endif

#define LED3_PIN                         GPIO_Pin_1
#define LED3_GPIO_PORT                   GPIOB
#define LED3_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LED4_PIN                         GPIO_Pin_14
#define LED4_GPIO_PORT                   GPIOB
#define LED4_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LED5_PIN                         GPIO_Pin_6
#define LED5_GPIO_PORT                   GPIOA
#define LED5_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LED6_PIN                         GPIO_Pin_5
#define LED6_GPIO_PORT                   GPIOA
#define LED6_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LED7_PIN                         GPIO_Pin_12
#define LED7_GPIO_PORT                   GPIOA
#define LED7_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LED8_PIN                         GPIO_Pin_6
#define LED8_GPIO_PORT                   GPIOF
#define LED8_GPIO_CLK                    RCC_AHBPeriph_GPIOF

#define LED9_PIN                         GPIO_Pin_4
#define LED9_GPIO_PORT                   GPIOA
#define LED9_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LED0_PIN                         GPIO_Pin_1
#define LED0_GPIO_PORT                   GPIOA
#define LED0_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LEDA_PIN                         GPIO_Pin_8
#define LEDA_GPIO_PORT                   GPIOA
#define LEDA_GPIO_CLK                    RCC_AHBPeriph_GPIOA

#define LEDB_PIN                         GPIO_Pin_12
#define LEDB_GPIO_PORT                   GPIOB
#define LEDB_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LEDC_PIN                         GPIO_Pin_9
#define LEDC_GPIO_PORT                   GPIOB
#define LEDC_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LEDD_PIN                         GPIO_Pin_2
#define LEDD_GPIO_PORT                   GPIOB
#define LEDD_GPIO_CLK                    RCC_AHBPeriph_GPIOB

#define LEDE_PIN                         GPIO_Pin_13
#define LEDE_GPIO_PORT                   GPIOB
#define LEDE_GPIO_CLK                    RCC_AHBPeriph_GPIOB


/* Limit SENSOR 1 */
#define GPIO_IN1_PIN                     GPIO_Pin_7
#define GPIO_IN1_GPIO_PORT               GPIOF
#define GPIO_IN1_GPIO_CLK                RCC_AHBPeriph_GPIOF
/* Limit SENSOR 2 */
#define GPIO_IN2_PIN                     GPIO_Pin_15
#define GPIO_IN2_GPIO_PORT               GPIOA
#define GPIO_IN2_GPIO_CLK                RCC_AHBPeriph_GPIOA
/* RESET Button */
#define GPIO_IN3_PIN                     GPIO_Pin_5
#define GPIO_IN3_GPIO_PORT               GPIOB
#define GPIO_IN3_GPIO_CLK                RCC_AHBPeriph_GPIOB

/** Buzzer */
#define GPIO_BZ_PIN                     GPIO_Pin_5
#define GPIO_BZ_GPIO_PORT               GPIOB
#define GPIO_BZ_GPIO_CLK                RCC_AHBPeriph_GPIOB
#define BZ_PWM_TIMMER                   TIM1
#define BZ_PWM_TIMER_CLOCK							RCC_APB2Periph_TIM1

/** Motor */
#define MOTOR_SLEEP_PIN                 GPIO_Pin_11             //  PA.11
#define MOTOR_SLEEP_GPIO_PORT           GPIOA
#define MOTOR_SLEEP_GPIO_CLK            RCC_AHBPeriph_GPIOA

#define MOTOR_PIN1                      GPIO_Pin_3              //  PB.03
#define MOTOR_PIN2                      GPIO_Pin_4              //  PB.04
#define MOTOR_PIN_GPIO_PORT             GPIOB
#define MOTOR_PIN_GPIO_CLK              RCC_AHBPeriph_GPIOB

#define M_SLEEP()       GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define M_ACTIVE()      GPIO_SetBits  (GPIOA, GPIO_Pin_11)

#define M_IN1_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define M_IN1_HIGH()      GPIO_SetBits  (GPIOB, GPIO_Pin_3)

#define M_IN2_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define M_IN2_HIGH()      GPIO_SetBits  (GPIOB, GPIO_Pin_4)

/* TWCC ENABLE */
#define TWCC_ENABLE_PIN                   GPIO_Pin_13
#define TWCC_ENABLE_GPIO_PORT             GPIOC
#define TWCC_ENABLE_GPIO_CLK              RCC_AHBPeriph_GPIOC

/** @addtogroup WDL001_LOW_LEVEL_COM
  * @{
  */
#define COMn                             2

/**
 * @brief Definition for COM port1, connected to USART1
 */
#define COM_DEBUG                        USART1
#define COM_DEBUG_CLK                    RCC_APB2Periph_USART1

#define COM_DEBUG_TX_PIN                 GPIO_Pin_9
#define COM_DEBUG_TX_GPIO_PORT           GPIOA
#define COM_DEBUG_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define COM_DEBUG_TX_SOURCE              GPIO_PinSource9
#define COM_DEBUG_TX_AF                  GPIO_AF_1

#define COM_DEBUG_RX_PIN                 GPIO_Pin_10
#define COM_DEBUG_RX_GPIO_PORT           GPIOA
#define COM_DEBUG_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define COM_DEBUG_RX_SOURCE              GPIO_PinSource10
#define COM_DEBUG_RX_AF                  GPIO_AF_1

#define COM_MODEM                        USART2
#define COM_MODEM_CLK                    RCC_APB1Periph_USART2

#define COM_MODEM_TX_PIN                 GPIO_Pin_2
#define COM_MODEM_TX_GPIO_PORT           GPIOA
#define COM_MODEM_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define COM_MODEM_TX_SOURCE              GPIO_PinSource2
#define COM_MODEM_TX_AF                  GPIO_AF_1

#define COM_MODEM_RX_PIN                 GPIO_Pin_3
#define COM_MODEM_RX_GPIO_PORT           GPIOA
#define COM_MODEM_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define COM_MODEM_RX_SOURCE              GPIO_PinSource3
#define COM_MODEM_RX_AF                  GPIO_AF_1

#define COM_DEBUG_IRQn                   USART1_IRQn
#define COM_MODEM_IRQn                   USART2_IRQn

/** @addtogroup TOUCH_PAD_I2C
  * @{
  */
/**
  * @brief  Touch pad I2C Interface pins
  */
#define TP_I2C                           I2C2
#define TP_I2C_CLK                       RCC_APB1Periph_I2C2

#define TP_I2C_SCL_PIN                   GPIO_Pin_10                  /* PB.10 */
#define TP_I2C_SCL_GPIO_PORT             GPIOB                       /* GPIOB */
#define TP_I2C_SCL_GPIO_CLK              RCC_AHBPeriph_GPIOB
#define TP_I2C_SCL_SOURCE                GPIO_PinSource10
#define TP_I2C_SCL_AF                    GPIO_AF_1

#define TP_I2C_SDA_PIN                   GPIO_Pin_11                  /* PB.11 */
#define TP_I2C_SDA_GPIO_PORT             GPIOB                       /* GPIOB */
#define TP_I2C_SDA_GPIO_CLK              RCC_AHBPeriph_GPIOB
#define TP_I2C_SDA_SOURCE                GPIO_PinSource11
#define TP_I2C_SDA_AF                    GPIO_AF_1


/**
  * @}
  */



void HW_LEDInitIndividual(Led_TypeDef Led);
void HW_LEDInitAll(void);
void HW_GPIO_IN_Init(void);

void HW_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);

void GPIO_INIT(void);
void UART_INIT(void);
void UART_SendBuffer(USART_TypeDef* USARTx, uint8_t *pCommand, uint8_t length);
void LED_On(Led_TypeDef Led);
void LED_Off(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);

void decrement_delay( void );
void delay_msSysTick(uint16_t delay);
void HW_TP_LowLevel_Init(void);
void HW_TP_LowLevel_DeInit(void);

/**
  * @}
  */

#endif

