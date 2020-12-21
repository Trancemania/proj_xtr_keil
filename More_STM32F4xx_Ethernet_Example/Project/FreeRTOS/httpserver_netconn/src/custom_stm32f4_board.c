/**
  ******************************************************************************
  * @file    custom_stm32f4_board.c
  * @author  Trancemania Wang
  * @version V1.0
  * @date    21-July-2020
  * @brief   This file provides set of firmware functions to manage peripherals on
  * 		 		 custom board based on your own project requires.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "custom_stm32f4_board.h"
#include "FreeRTOS.h"
#include "task.h"

USART_TypeDef* CUSTOM_COM_USART[CUSTOM_COMn] = {CUSTOM_COM1, CUSTOM_COM2, CUSTOM_COM3, CUSTOM_COM4}; 

GPIO_TypeDef* CUSTOM_COM_TX_PORT[CUSTOM_COMn] = {CUSTOM_COM1_TX_GPIO_PORT, CUSTOM_COM2_TX_GPIO_PORT, CUSTOM_COM3_TX_GPIO_PORT, CUSTOM_COM4_TX_GPIO_PORT};
 
GPIO_TypeDef* CUSTOM_COM_RX_PORT[CUSTOM_COMn] = {CUSTOM_COM1_RX_GPIO_PORT, CUSTOM_COM2_RX_GPIO_PORT, CUSTOM_COM3_RX_GPIO_PORT, CUSTOM_COM4_RX_GPIO_PORT};

const uint32_t CUSTOM_COM_USART_CLK[CUSTOM_COMn] = {CUSTOM_COM1_CLK, CUSTOM_COM2_CLK, CUSTOM_COM3_CLK, CUSTOM_COM4_CLK};

const uint32_t CUSTOM_COM_TX_PORT_CLK[CUSTOM_COMn] = {CUSTOM_COM1_TX_GPIO_CLK, CUSTOM_COM2_TX_GPIO_CLK, CUSTOM_COM3_TX_GPIO_CLK, CUSTOM_COM4_TX_GPIO_CLK};
 
const uint32_t CUSTOM_COM_RX_PORT_CLK[CUSTOM_COMn] = {CUSTOM_COM1_RX_GPIO_CLK, CUSTOM_COM2_RX_GPIO_CLK, CUSTOM_COM3_RX_GPIO_CLK, CUSTOM_COM4_RX_GPIO_CLK};

const uint16_t CUSTOM_COM_TX_PIN[CUSTOM_COMn] = {CUSTOM_COM1_TX_PIN, CUSTOM_COM2_TX_PIN, CUSTOM_COM3_TX_PIN, CUSTOM_COM4_TX_PIN};

const uint16_t CUSTOM_COM_RX_PIN[CUSTOM_COMn] = {CUSTOM_COM1_RX_PIN, CUSTOM_COM2_RX_PIN, CUSTOM_COM3_RX_PIN, CUSTOM_COM4_RX_PIN};
 
const uint16_t CUSTOM_COM_TX_PIN_SOURCE[CUSTOM_COMn] = {CUSTOM_COM1_TX_SOURCE, CUSTOM_COM2_TX_SOURCE, CUSTOM_COM3_TX_SOURCE, CUSTOM_COM4_TX_SOURCE};

const uint16_t CUSTOM_COM_RX_PIN_SOURCE[CUSTOM_COMn] = {CUSTOM_COM1_RX_SOURCE, CUSTOM_COM2_RX_SOURCE, CUSTOM_COM3_RX_SOURCE, CUSTOM_COM4_RX_SOURCE};
 
const uint16_t CUSTOM_COM_TX_AF[CUSTOM_COMn] = {CUSTOM_COM1_TX_AF, CUSTOM_COM2_TX_AF, CUSTOM_COM3_TX_AF, CUSTOM_COM4_TX_AF};
 
const uint16_t CUSTOM_COM_RX_AF[CUSTOM_COMn] = {CUSTOM_COM1_RX_AF, CUSTOM_COM2_RX_AF, CUSTOM_COM3_RX_AF, CUSTOM_COM4_RX_AF};

NVIC_InitTypeDef   CUSTOM_NVIC_InitStructure;



void STM_CUSTOM_COMInit(CUSTOM_NO_COM_TypeDef CUSTOM_COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CUSTOM_COM_TX_PORT_CLK[CUSTOM_COM] | CUSTOM_COM_RX_PORT_CLK[CUSTOM_COM], ENABLE);

   /* Enable UART clock */
	if (CUSTOM_COM == 0) {
  RCC_APB2PeriphClockCmd(CUSTOM_COM_USART_CLK[CUSTOM_COM], ENABLE);    //uart1/6 apb2, uart2/3/4/5 apb1
	}
	
	else{
    /* Enable UART clock */
  RCC_APB1PeriphClockCmd(CUSTOM_COM_USART_CLK[CUSTOM_COM], ENABLE);	 
	}

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(CUSTOM_COM_TX_PORT[CUSTOM_COM], CUSTOM_COM_TX_PIN_SOURCE[CUSTOM_COM], CUSTOM_COM_TX_AF[CUSTOM_COM]);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(CUSTOM_COM_RX_PORT[CUSTOM_COM], CUSTOM_COM_RX_PIN_SOURCE[CUSTOM_COM], CUSTOM_COM_RX_AF[CUSTOM_COM]);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = CUSTOM_COM_TX_PIN[CUSTOM_COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CUSTOM_COM_TX_PORT[CUSTOM_COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CUSTOM_COM_RX_PIN[CUSTOM_COM];
  GPIO_Init(CUSTOM_COM_RX_PORT[CUSTOM_COM], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(CUSTOM_COM_USART[CUSTOM_COM], USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(CUSTOM_COM_USART[CUSTOM_COM], ENABLE);
}

void CUSTOM_GPIO_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_15);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOD, GPIO_Pin_11);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOE, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

//http://hogepga.blog.jp/archives/1020407703.html
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);

//https://blog.csdn.net/zxh1592000/article/details/80280715
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);
//	vTaskDelay(1);

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);
//	vTaskDelay(1);
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);
//	vTaskDelay(1);
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
//	vTaskDelay(1);
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
//	vTaskDelay(1);
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	vTaskDelay(1);
	
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	vTaskDelay(1);

//	NVIC_Init(&NVIC_InitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//	vTaskDelay(1);
//	NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//	vTaskDelay(1);
//	NVIC_Init(&NVIC_InitStructure);
	
}

void CUSTOM_PWM_Init() {
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	uint16_t Period;
	Period = 1333; // 20 KHz for 10MHz prescaled
	
	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Initialize PB4, Alternative Function, 100Mhz, Output, Push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	/* Timer Base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock  / 2 / 10000)) - 1; // Get clock to 10 KHz on STM32F4; 
	//https://qiita.com/forest1/items/b02042714b6ed30dc625
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = Period - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //only for tim1 and tim8
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* Enable TIM3 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	/* TIM PWM1 Mode configuration */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // 0%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	//negative pulse
	/* Output Compare PWM1 Mode configuration: Channel1 PB.04 */
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
//	TIM_ITConfig(TIM3, TIM_FLAG_CC1, ENABLE);   //open by varying frequency task
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	
	/* TIM3 NVIC config */
	//https://community.st.com/s/question/0D50X00009XkXvvSAF/timer-interrupt-irqhandler-error
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void CUSTOM_EXTI_Init() {
    /* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	/* Enable clock for GPIOC */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	
	
	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	/* Tell system that you will use PC10/11/12/13 for EXTI_Line */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	
	/* Tell system that you will use PD0/1 for EXTI_Line */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	
	/* PC10/11/12/13 is connected to EXTI_Line10/11/12/13 */
	/* PD0/1 is connected to EXTI_Line0/1 */
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line10;
	EXTI_Init(&EXTI_InitStruct);	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line11;
	EXTI_Init(&EXTI_InitStruct);	
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	EXTI_Init(&EXTI_InitStruct);	

	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_Init(&EXTI_InitStruct);	

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_Init(&EXTI_InitStruct);	

	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_Init(&EXTI_InitStruct);	

	/* Add IRQ vector to NVIC */
	/*  is connected to EXTI_Line0/1/10/11/12/13, which has EXTI0/1/10/11/12/13_IRQn vector */
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStruct);	
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStruct);	

	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStruct);	

}

void CUSTOM_UART_INT_Init() {               //https://stackoverflow.com/questions/12543076/usart-receive-interrupt-stm32
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//#define LED4_PIN                         GPIO_Pin_12
//#define LED4_GPIO_PORT                   GPIOD
//#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOD  
