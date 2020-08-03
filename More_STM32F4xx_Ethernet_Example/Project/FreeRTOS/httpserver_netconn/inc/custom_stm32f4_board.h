/**
  ******************************************************************************
  * @file    custom_stm32f4_board.h
  * @author  Trancemania Wang
  * @version V1.0
  * @date    21-July-2020
  * @brief   This file contains definitions for peripherals of custom board.
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
#ifndef __CUSTOM_STM32F4_BOARD_H
#define __CUSTOM_STM32F4_BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

typedef enum 
{
  CUSTOM_NO_COM1 = 0,
	CUSTOM_NO_COM2 = 1,
	CUSTOM_NO_COM3 = 2,
	CUSTOM_NO_COM4 = 3,
} CUSTOM_NO_COM_TypeDef; 

#define CUSTOM_COMn                             4
/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define CUSTOM_COM1                        USART1
#define CUSTOM_COM1_CLK                    RCC_APB2Periph_USART1
#define CUSTOM_COM1_TX_PIN                 GPIO_Pin_6
#define CUSTOM_COM1_TX_GPIO_PORT           GPIOB
#define CUSTOM_COM1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CUSTOM_COM1_TX_SOURCE              GPIO_PinSource6
#define CUSTOM_COM1_TX_AF                  GPIO_AF_USART1
#define CUSTOM_COM1_RX_PIN                 GPIO_Pin_7
#define CUSTOM_COM1_RX_GPIO_PORT           GPIOB
#define CUSTOM_COM1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CUSTOM_COM1_RX_SOURCE              GPIO_PinSource7
#define CUSTOM_COM1_RX_AF                  GPIO_AF_USART1
#define CUSTOM_COM1_IRQn                   USART1_IRQn

#define CUSTOM_COM1_TX_DMA                 DMA2
#define CUSTOM_COM1_TX_DMA_CLK             RCC_AHB1Periph_DMA2

#define CUSTOM_COM1_RX_DMA                 DMA2
#define CUSTOM_COM1_RX_DMA_CLK             RCC_AHB1Periph_DMA2

#define CUSTOM_COM1_TX_DMA_STREAM          DMA2_Stream7
#define CUSTOM_COM1_TX_DMA_CHANNEL         DMA_Channel_4

#define CUSTOM_COM1_RX_DMA_STREAM          DMA2_Stream2
#define CUSTOM_COM1_RX_DMA_CHANNEL         DMA_Channel_4

//Definition for COM port2, connected to USART2

#define CUSTOM_COM2                        USART2
#define CUSTOM_COM2_CLK                    RCC_APB1Periph_USART2
#define CUSTOM_COM2_TX_PIN                 GPIO_Pin_5
#define CUSTOM_COM2_TX_GPIO_PORT           GPIOD
#define CUSTOM_COM2_TX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CUSTOM_COM2_TX_SOURCE              GPIO_PinSource5
#define CUSTOM_COM2_TX_AF                  GPIO_AF_USART2
#define CUSTOM_COM2_RX_PIN                 GPIO_Pin_6
#define CUSTOM_COM2_RX_GPIO_PORT           GPIOD
#define CUSTOM_COM2_RX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CUSTOM_COM2_RX_SOURCE              GPIO_PinSource6
#define CUSTOM_COM2_RX_AF                  GPIO_AF_USART2
#define CUSTOM_COM2_IRQn                   USART2_IRQn

#define CUSTOM_COM2_TX_DMA                 DMA1
#define CUSTOM_COM2_TX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM2_RX_DMA                 DMA1
#define CUSTOM_COM2_RX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM2_TX_DMA_STREAM          DMA1_Stream6
#define CUSTOM_COM2_TX_DMA_CHANNEL         DMA_Channel_4

#define CUSTOM_COM2_RX_DMA_STREAM          DMA1_Stream5
#define CUSTOM_COM2_RX_DMA_CHANNEL         DMA_Channel_4

//Definition for COM port3, connected to USART3

#define CUSTOM_COM3                        USART3
#define CUSTOM_COM3_CLK                    RCC_APB1Periph_USART3
#define CUSTOM_COM3_TX_PIN                 GPIO_Pin_8
#define CUSTOM_COM3_TX_GPIO_PORT           GPIOD
#define CUSTOM_COM3_TX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CUSTOM_COM3_TX_SOURCE              GPIO_PinSource8
#define CUSTOM_COM3_TX_AF                  GPIO_AF_USART3
#define CUSTOM_COM3_RX_PIN                 GPIO_Pin_9
#define CUSTOM_COM3_RX_GPIO_PORT           GPIOD
#define CUSTOM_COM3_RX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CUSTOM_COM3_RX_SOURCE              GPIO_PinSource9
#define CUSTOM_COM3_RX_AF                  GPIO_AF_USART3
#define CUSTOM_COM3_IRQn                   USART3_IRQn

#define CUSTOM_COM3_TX_DMA                 DMA1
#define CUSTOM_COM3_TX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM3_RX_DMA                 DMA1
#define CUSTOM_COM3_RX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM3_TX_DMA_STREAM          DMA1_Stream3
#define CUSTOM_COM3_TX_DMA_CHANNEL         DMA_Channel_4

#define CUSTOM_COM3_RX_DMA_STREAM          DMA1_Stream1
#define CUSTOM_COM3_RX_DMA_CHANNEL         DMA_Channel_4

//Definition for COM port4, connected to UART4

#define CUSTOM_COM4                        UART4
#define CUSTOM_COM4_CLK                    RCC_APB1Periph_UART4
#define CUSTOM_COM4_TX_PIN                 GPIO_Pin_10
#define CUSTOM_COM4_TX_GPIO_PORT           GPIOC
#define CUSTOM_COM4_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define CUSTOM_COM4_TX_SOURCE              GPIO_PinSource10
#define CUSTOM_COM4_TX_AF                  GPIO_AF_UART4
#define CUSTOM_COM4_RX_PIN                 GPIO_Pin_11
#define CUSTOM_COM4_RX_GPIO_PORT           GPIOC
#define CUSTOM_COM4_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define CUSTOM_COM4_RX_SOURCE              GPIO_PinSource11
#define CUSTOM_COM4_RX_AF                  GPIO_AF_UART4
#define CUSTOM_COM4_IRQn                   UART4_IRQn

#define CUSTOM_COM4_TX_DMA                 DMA1
#define CUSTOM_COM4_TX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM4_RX_DMA                 DMA1
#define CUSTOM_COM4_RX_DMA_CLK             RCC_AHB1Periph_DMA1

#define CUSTOM_COM4_TX_DMA_STREAM          DMA1_Stream4
#define CUSTOM_COM4_TX_DMA_CHANNEL         DMA_Channel_4

#define CUSTOM_COM4_RX_DMA_STREAM          DMA1_Stream2
#define CUSTOM_COM4_RX_DMA_CHANNEL         DMA_Channel_4

void STM_CUSTOM_COMInit(CUSTOM_NO_COM_TypeDef CUSTOM_COM, USART_InitTypeDef* USART_InitStruct);
void CUSTOM_GPIO_Init(void);
void CUSTOM_PWM_Init(void);


#endif /* __STM32F4_DISCOVERY_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

 

/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
