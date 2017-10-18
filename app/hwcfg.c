#include "hwcfg.h"


/** @addtogroup WDL_HW_LOW_LEVEL
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef DISCOVERY030
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};
#else
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN};
#endif
/*, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT, LED5_GPIO_PORT, LED6_GPIO_PORT,
                                 LED7_GPIO_PORT, LED8_GPIO_PORT, LED9_GPIO_PORT,
                                 LED0_GPIO_PORT, LEDA_GPIO_PORT, LEDB_GPIO_PORT,
                                 LEDC_GPIO_PORT, LEDD_GPIO_PORT, LEDE_GPIO_PORT};*/


/*, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN,
	                               LED6_PIN, LED7_PIN, LED8_PIN, LED9_PIN, LED0_PIN,
                                 LEDA_PIN, LEDB_PIN, LEDC_PIN, LEDD_PIN, LEDE_PIN}; */

const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK};
/*, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK, LED5_GPIO_CLK, LED6_GPIO_CLK,
                                 LED7_GPIO_CLK, LED8_GPIO_CLK, LED9_GPIO_CLK,
                                 LED0_GPIO_CLK, LEDA_GPIO_CLK, LEDB_GPIO_CLK,
                                 LEDC_GPIO_CLK, LEDD_GPIO_CLK, LEDE_GPIO_CLK}; */

USART_TypeDef* COM_USART[COMn] = {COM_DEBUG, COM_MODEM};

GPIO_TypeDef* COM_TX_PORT[COMn] = {COM_DEBUG_TX_GPIO_PORT, COM_MODEM_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {COM_DEBUG_RX_GPIO_PORT, COM_MODEM_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {COM_DEBUG_CLK, COM_MODEM_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {COM_DEBUG_TX_GPIO_CLK, COM_MODEM_TX_GPIO_CLK};

const uint32_t COM_RX_PORT_CLK[COMn] = {COM_DEBUG_RX_GPIO_CLK, COM_MODEM_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {COM_DEBUG_TX_PIN, COM_MODEM_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {COM_DEBUG_RX_PIN, COM_MODEM_RX_PIN};

const uint8_t COM_TX_PIN_SOURCE[COMn] = {COM_DEBUG_TX_SOURCE, COM_MODEM_TX_SOURCE};

const uint8_t COM_RX_PIN_SOURCE[COMn] = {COM_DEBUG_RX_SOURCE, COM_MODEM_RX_SOURCE};

const uint8_t COM_TX_AF[COMn] = {COM_DEBUG_TX_AF, COM_MODEM_TX_AF};

const uint8_t COM_RX_AF[COMn] = {COM_DEBUG_RX_AF, COM_MODEM_RX_AF};

__IO uint16_t counter_delay_ms;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures ALL LED GPIO.
  * @param  None
  * @retval None
  */
void HW_LEDInitAll(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED0_PIN | LED9_PIN | LED6_PIN | LED5_PIN | LED2_PIN | LEDA_PIN | LED7_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIOA->BRR = LED0_PIN | LED9_PIN | LED6_PIN | LED5_PIN | LED2_PIN | LEDA_PIN | LED7_PIN;

 /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LEDD_PIN | LED3_PIN | LEDB_PIN | LEDE_PIN | LED4_PIN | LED1_PIN | LEDC_PIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIOB->BRR = LEDD_PIN | LED3_PIN | LEDB_PIN | LEDE_PIN | LED4_PIN | LED1_PIN | LEDC_PIN;

 /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF , ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED8_PIN;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIOF->BRR = LED8_PIN;
}


/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *          This parameter can be one of following parameters:
  *          @arg LED0 ~ LED9, LEDA, LEDB, LEDC, LEDD, LEDE, LEDF
  * @retval None
  */
void HW_LEDInitIndividual(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}


/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *          This parameter can be one of following parameters:
  *
  *          @arg LED0 ~ LED9, LEDA, LEDB, LEDC, LEDD, LEDE, LEDF
  *
  * @retval None
  */
void LED_On(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *          This parameter can be one of following parameters:
  *
  *          @arg LED0 ~ LED9, LEDA, LEDB, LEDC, LEDD, LEDE, LEDF
  *
  * @retval None
  */
void LED_Off(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}


/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *          This parameter can be one of following parameters:
  *
  *          @arg LED0 ~ LED9, LEDA, LEDB, LEDC, LEDD, LEDE, LEDF
  *
  * @retval None
  */
void LED_Toggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures ALL GPIO INPUT.
  * @param  None
  * @retval None
  */
void HW_GPIO_IN_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF , ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_IN1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_IN2_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_IN3_PIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI7 Line to PF7 pin (GPIO_IN1_PIN)*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource7); 
  /* Connect EXTI15 Line to PA15 pin (GPIO_IN2_PIN)*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);
  /* Connect EXTI5 Line to PB3 pin (GPIO_IN3_PIN)*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5); 

  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line =  EXTI_Line5 | EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_InitStructure.EXTI_Line =  EXTI_Line15;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI15 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *          This parameter can be one of following parameters:
  *            @arg COM1
  *            @arg COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *         contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void HW_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);

  /* Enable USART clock */
  if( COM_USART[COM] == USART1)
    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  else
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);

  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);

  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}


/**
 * @brief  Send a byte array over UART
 * @param  USARTx : where x can be 1, 2, 3 to select the USART peripheral
 * @param  pData  : buffer to send
 * @param  length : number of bytes to send
 * @return None
 */
void UART_SendBuffer(USART_TypeDef* USARTx, uint8_t *pCommand, uint8_t length)
{
	uint8_t i;


	for(i=0; i<length; i++)
	{
    /* Wait for USART Tx buffer empty */
	  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
		USART_SendData(USARTx, pCommand[i]);
	}
}

/** @addtogroup TOUCH_PAD_I2C
  * @{
  */
/**
  * @brief  Touch pad I2C Interface pins
  */
/**
  * @brief  DeInitializes peripherals used by the I2C Touch driver.
  * @param  None
  * @retval None
  */
void HW_I2C_Touch_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* sEE_I2C Peripheral Disable */
  I2C_Cmd(TP_I2C, DISABLE);

  /* sEE_I2C DeInit */
  I2C_DeInit(TP_I2C);

  /* sEE_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(TP_I2C_CLK, DISABLE);

  /* GPIO configuration */
  /* Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SDA_PIN;
  GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the I2C source clock and IOs used to drive the Touch.
  * @param  None
  * @retval None
  */
void HW_I2C_Touch_LowLevelInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Configure the I2C clock source. The clock is derived from the HSI */
  RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  /* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_AHBPeriphClockCmd(TP_I2C_SCL_GPIO_CLK | TP_I2C_SDA_GPIO_CLK, ENABLE);

  /* sEE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(TP_I2C_CLK, ENABLE);

  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(TP_I2C_SCL_GPIO_PORT, TP_I2C_SCL_SOURCE, TP_I2C_SCL_AF);

  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_SOURCE, TP_I2C_SDA_AF);

  /* GPIO configuration */
  /* Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SDA_PIN;
  GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}


void UART_INIT(void)
{
  USART_InitTypeDef uf;
  NVIC_InitTypeDef NVIC_InitStructure;

  uf.USART_BaudRate = 115200;
  uf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  uf.USART_WordLength = USART_WordLength_8b;
  uf.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  uf.USART_Parity = USART_Parity_No;
  uf.USART_StopBits = USART_StopBits_1;

  /* Configure USART */
  HW_COMInit(COM2, &uf);
  HW_COMInit(COM1, &uf);
  /* NVIC configuration */
  /* Enable the COM_MODEM Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = COM_MODEM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(COM_MODEM, USART_IT_RXNE, ENABLE);

  /* Enable the COM_DEBUG Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = COM_DEBUG_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(COM_DEBUG, USART_IT_RXNE, ENABLE);
}


/**
 *  @brief  This function decrements the counter every millisecond used by the function delay_ms
 *  @param  None
 *  @retval None
 */
void decrement_delay(void)
{
  if(counter_delay_ms > 0)
  {
    /* Decrements the counter */
    counter_delay_ms--;
  }
}

void delay_msSysTick(uint16_t delay)
{
  counter_delay_ms = delay;

  while(counter_delay_ms != 0);
}


/**
  * @brief  DeInitializes peripherals used by the I2C TOUCH PAD.
  * @param  None
  * @retval None
  */
void HW_TP_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* sEE_I2C Peripheral Disable */
  I2C_Cmd(TP_I2C, DISABLE);

  /* sEE_I2C DeInit */
  I2C_DeInit(TP_I2C);

  /* sEE_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(TP_I2C_CLK, DISABLE);

  /* GPIO configuration */
  /* Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SDA_PIN;
  GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the I2C source clock and IOs used to drive the Touch.
  * @param  None
  * @retval None
  */
void HW_TP_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Configure the I2C clock source. The clock is derived from the HSI */
  RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

  /* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_AHBPeriphClockCmd(TP_I2C_SCL_GPIO_CLK | TP_I2C_SDA_GPIO_CLK, ENABLE);

  /* sEE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(TP_I2C_CLK, ENABLE);

  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(TP_I2C_SCL_GPIO_PORT, TP_I2C_SCL_SOURCE, GPIO_AF_1);

  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_SOURCE, GPIO_AF_1);

  /* GPIO configuration */
  /* Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = TP_I2C_SDA_PIN;
  GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}
