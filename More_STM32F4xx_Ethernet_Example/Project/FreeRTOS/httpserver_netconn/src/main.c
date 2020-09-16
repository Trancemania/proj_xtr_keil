/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    main.c
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   Main program body      
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB and
  *          STM32F4DIS-LCD modules. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tcpip.h"
#include "httpserver-netconn.h"
#include "udp.h"

#include "custom_stm32f4_board.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_usart_dma.h"

#include <stdio.h>
#include <stdarg.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   "Basic WebServer Demo"
#define MESSAGE4   "                    "

/*--------------- Tasks Priority -------------*/
#define DHCP_TASK_PRIO   ( tskIDLE_PRIORITY + 2 )      
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )
#define PRINTF_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )
#define UDP_TASK_PRIO		 ( tskIDLE_PRIORITY + 1 )
#define ETH_PRINTF_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )
#define PROCESS_COM_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
//xSemaphoreHandle xSemaphore_DHCP = NULL;
//xSemaphoreHandle xSemaphore_UDP = NULL;
//struct udp_pcb *pcb;
//struct pbuf *p;
//struct ip_addr dst_addr;

extern struct udp_pcb *pcb;
extern struct pbuf *p;
extern struct ip_addr dst_addr;

extern struct netif xnetif;

extern const unsigned short dst_port;
extern const unsigned short src_port;

uint8_t ETH_send_buffer[128];
uint8_t ETH_recv_buffer[128];
//uint8_t command_H;
//uint8_t command_L;
uint8_t command;
int ETH_length;

xTaskHandle printf_xHandle = NULL;
xTaskHandle process_command_xHandle = NULL;

//const unsigned short dst_port = 8080;

	
/* Private function prototypes -----------------------------------------------*/
						
void LCD_LED_Init(void);
void ToggleLed4(void * pvParameters);
void Printf_task(void * pvParameters);
//void LwIP_UDP_task(void * pvParameters);
void ETH_Printf_task(void * pvParameters);
void process_command_task(void * pvParameters);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void DMA_printf(USART_TypeDef* USARTx, const char *format,...);
void ETH_printf(const char *format,...);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured to 
       144 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
  /*Initialize LCD and Leds */ 
  LCD_LED_Init();
	CUSTOM_GPIO_Init();
	CUSTOM_PWM_Init();
	
	//test code
//	TIM_SetAutoreload(TIM3, 1333);
//	TIM_SetCompare1(TIM3, 100);

  /*Initialize Serial COM */ 
  STM_CUSTOM_COMInit(CUSTOM_NO_COM1, &USART_InitStructure);
	STM_CUSTOM_COMInit(CUSTOM_NO_COM2, &USART_InitStructure);
	STM_CUSTOM_COMInit(CUSTOM_NO_COM3, &USART_InitStructure);
	STM_CUSTOM_COMInit(CUSTOM_NO_COM4, &USART_InitStructure);
	TM_USART_DMA_Init(USART1);
	TM_USART_DMA_Init(USART2);
	TM_USART_DMA_Init(USART3);
	TM_USART_DMA_Init(UART4);
  DMA_printf(USART1, "\n\rSerial communication complete!\n\r");
	
  /* configure Ethernet (GPIOs, clocks, MAC, DMA) */ 
  ETH_BSP_Config();
    
  /* Initilaize the LwIP stack */
  LwIP_Init();
  
  /* Initialize webserver demo */
  http_server_netconn_init();

#ifdef USE_DHCP
  /* Start DHCPClient */
  xTaskCreate(LwIP_DHCP_task, "DHCPClient", configMINIMAL_STACK_SIZE * 4, NULL,DHCP_TASK_PRIO, NULL);
#endif

  /* Start UDP task */
//  xTaskCreate(LwIP_UDP_task, "UDP", configMINIMAL_STACK_SIZE * 4, NULL, UDP_TASK_PRIO, NULL);

  /* Start toogleLed4 task : Toggle LED4  every 250ms */
  xTaskCreate(ToggleLed4, "LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);
	
  /* Start custom task : printf */
  xTaskCreate(Printf_task, "Printf", configMINIMAL_STACK_SIZE * 4, NULL, PRINTF_TASK_PRIO, NULL);

  /* Start custom task : ETH_printf */
//  xTaskCreate(ETH_Printf_task, "ETH_Printf", configMINIMAL_STACK_SIZE * 4, NULL, ETH_PRINTF_TASK_PRIO, NULL);

//ETH send ready
	
  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );

}

/**
  * @brief  custom printf task
  * @param  pvParameters not used
  * @retval None
  */
void Printf_task(void * pvParameters)
{
  int ch;
	int cnt = 0;
	uint8_t buffer[128];
  for ( ;; ) {
    
    while (USART_GetFlagStatus(CUSTOM_COM1, USART_FLAG_RXNE) == RESET);
  	
    ch = USART_ReceiveData(CUSTOM_COM1);
		
		if (ch == 0x0A){
				cnt = 0;
				buffer[cnt] = ch;
				cnt++;
				while (buffer[cnt-1] != 0x0D){
					while (USART_GetFlagStatus(CUSTOM_COM1, USART_FLAG_RXNE) == RESET);
					ch = USART_ReceiveData(CUSTOM_COM1);
					switch (ch){
						case 0x0D:   //0A send 
							buffer[cnt] = ch;
							ETH_length = cnt + 1;
							p = pbuf_alloc(PBUF_TRANSPORT, ETH_length, PBUF_RAM);
							memcpy(p->payload, &buffer, ETH_length);
							p->len = ETH_length;
							udp_sendto_if(pcb, p, &dst_addr, dst_port, &xnetif);
							pbuf_free(p);
							cnt++;
							break;
						default:
							//if cnt>=127
							buffer[cnt] = ch;
							cnt++;
							break;
					}

				}
			
			
		}
	  
//    switch (ch){
//			case 0x0A:   //0A send 
//				buffer[cnt] = ch;
//				ETH_length = cnt + 1;
//				p = pbuf_alloc(PBUF_TRANSPORT, ETH_length, PBUF_RAM);
//				memcpy(p->payload, &buffer, ETH_length);
//				p->len = ETH_length;
//				udp_sendto_if(pcb, p, &dst_addr, dst_port, &xnetif);
//				pbuf_free(p);
//				cnt = 0;
//				break;
//			default:
//				//if cnt>=127
//				buffer[cnt] = ch;
//				cnt++;
//				break;
//		}
//		if( xSemaphore_UDP != NULL )    {
			
//				ETH_printf("%c", ch&0xff);    
//    DMA_printf(USART1, "%c", ch&0xff);
//		}
  }
}


void udp_recv_fn(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port){
	if (p != NULL) {
		//check addr port
		
		// copy command to buffer
		memcpy(ETH_recv_buffer, &p->payload, p->len);
		
		// for command + last time mode
//		command_H = ETH_recv_buffer[1];
//		command_L = ETH_recv_buffer[0];
		
		// state machine mode
		command = ETH_recv_buffer[0];
		
		//check if command available
		if (command == 0x00	||	\
			  command == 0xFF ||  \
		    command == 0x01 ||  \
			  command == 0x02 ||  \
				command == 0x03 ||  \
				command == 0x04 ||  \
				command == 0x05 ||  \
				command == 0x06 ||  \
				command == 0x07 ||  \
				command == 0x08 ||  \
				command == 0x09 ||  \
				command == 0x0A ||  \
				command == 0x0B ||  \
				command == 0x0C ||  \
				command == 0x0D ||  \
				command == 0x0E ||  \
				command == 0x0F ||  \
				command == 0x10 ||  \
				command == 0x11 ||  \
				command == 0x12 ||  \
				command == 0x13 ||  \
				command == 0x14 ||  \
				command == 0x15 ||  \
				command == 0x16 ||  \
				command == 0x17 ||  \
				command == 0x18 ||  \
				command == 0x19 ||  \
				command == 0x1A ||  \
				command == 0x1B ||  \
				command == 0x1D ||  \
				command == 0x1F ||  \
				command == 0x20 ||  \
				command == 0x21 ||  \
				command == 0x22 ||  \
				command == 0x23 ||  \
				command == 0x24) {
			//delete command process task
			if( process_command_xHandle != NULL ) {
				vTaskDelete( process_command_xHandle );
			}
			
			//create command process task
			xTaskCreate(process_command_task, "PROCESS COMMAND", configMINIMAL_STACK_SIZE * 4, NULL, PROCESS_COM_TASK_PRIO, &process_command_xHandle);
			
			//free pbuf
			pbuf_free(p);
		}
		else {
			//free pbuf
			pbuf_free(p);
//			udp_sendto(pcb, p, &dst_addr, 1234); //dest port
		}
	}
}

//void ETH_Printf_task(void * pvParameters)
//{
//  int ch;
//  for ( ;; ) {
//    
//    while (USART_GetFlagStatus(CUSTOM_COM1, USART_FLAG_RXNE) == RESET);
//  	
//    ch = USART_ReceiveData(CUSTOM_COM1);
//	  
//    ETH_printf("%c", ch&0xff);    
//  }
//}


/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed4(void * pvParameters)
{
  for ( ;; ) {
    /* Toggle LED4 each 250ms */
    STM_EVAL_LEDToggle(LED4);
    vTaskDelay(250);
  }
}

//void LwIP_UDP_task(void * pvParameters)
//{
//  err_t err;
//  const unsigned short src_port = 12345;
//	
//#ifdef USE_DHCP
//	while( xSemaphore_DHCP == NULL )    {
//	}
//		// See if we can obtain the semaphore.  If the semaphore is not available
//		// wait 10 ticks to see if it becomes free. 
//	if( xSemaphoreTake( xSemaphore_DHCP, ( portTickType ) 5 ) == pdTRUE )
//	{
//#endif
//			// We were able to obtain the semaphore and can now access the
//			// shared resource.
//		
//			IP4_ADDR(&dst_addr,192,168,1,97);
//			pcb = udp_new();
//			err = udp_bind(pcb, IP_ADDR_ANY, src_port);
//		
//			// We have finished accessing the shared resource.  Release the 
//			// semaphore.
//#ifdef USE_DHCP
//			xSemaphoreGive( xSemaphore_DHCP );
//#endif
//			vSemaphoreCreateBinary( xSemaphore_UDP );
//			vTaskDelete(NULL);
//#ifdef USE_DHCP
//	}
//	else
//	{
//			vTaskDelete(NULL);
//			// We could not obtain the semaphore and can therefore not access
//			// the shared resource safely.
//	}
//#endif
//}

/**
  * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
void LCD_LED_Init(void)
{
#ifdef USE_LCD
  /* Initialize the STM324xG-EVAL's LCD */
  STM32f4_Discovery_LCD_Init();
#endif

  STM_EVAL_LEDInit(LED4); 
#ifdef USE_LCD
  /* Clear the LCD */
  LCD_Clear(Black);

  /* Set the LCD Back Color */
  LCD_SetBackColor(Black);

  /* Set the LCD Text Color */
  LCD_SetTextColor(White);

  /* Display message on the LCD*/
  LCD_DisplayStringLine(Line0, (uint8_t*)MESSAGE1);
  LCD_DisplayStringLine(Line1, (uint8_t*)MESSAGE2);
  LCD_DisplayStringLine(Line2, (uint8_t*)MESSAGE3);
  LCD_DisplayStringLine(Line3, (uint8_t*)MESSAGE4); 
#endif
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(CUSTOM_COM1, (uint8_t) ch);
	  
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(CUSTOM_COM1, USART_FLAG_TC) == RESET);
					 
  return ch;
}

void DMA_printf(USART_TypeDef* USARTx, const char *format,...)
{
	uint32_t length;
	va_list args;
	uint8_t buffer[256];	

//	while (huart1.gState != HAL_UART_STATE_READY);
	va_start(args, format);
	length = vsnprintf((char*)buffer, sizeof(buffer), (char*)format, args);
	va_end(args);
	
	TM_USART_DMA_Send(USARTx, (uint8_t *)&buffer, length);	
}

void ETH_printf(const char *format,...)
{
	uint32_t length;
	va_list args;
	uint8_t buffer[256];	

//  err_t err;

//	while (huart1.gState != HAL_UART_STATE_READY);
	va_start(args, format);
	length = vsnprintf((char*)buffer, sizeof(buffer), (char*)format, args);
	va_end(args);
	
//	p->payload = (void *)buffer;
//	pbuf_take(p, &buffer, length);
//	memcpy(p->payload, buffer, length);
		
	p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
//	p->len = length;
	memcpy(p->payload, &buffer, length);
	p->len = length;
	udp_sendto_if(pcb, p, &dst_addr, dst_port, &xnetif);
	pbuf_free(p);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
  {}
}
#endif


/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
