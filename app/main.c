#include "stm32f0xx.h"
#include "hwcfg.h"
#include "stdio.h"
#include "IQS316_driver.h"


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private function prototypes -----------------------------------------------*/
static void SysTickConfig(void);

int main(void)
{
  SysTickConfig();
  HW_LEDInitIndividual(LED1);
 // HW_GPIO_IN_Init();
  UART_INIT();
  LED_On(LED1);
  delay_msSysTick(400);
  LED_Off(LED1);
  IQS316_Init();
  printf("IQS316 init test\r\n");

  while(1)
  {
    //
    // Get data from latest comms window
    //
    IQS316_Refresh_Data();
    //
    // Process this new data accordingly
    //
    IQS316_Process_Data();
    
    delay_msSysTick(50);
  }
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(COM_DEBUG, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(COM_DEBUG, USART_FLAG_TC) == RESET)
  {}

  return ch;
}


/**
  * @brief  Configure a SysTick Base time to 10 ms.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
