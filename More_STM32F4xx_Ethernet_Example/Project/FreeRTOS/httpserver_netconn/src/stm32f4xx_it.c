/**
  ******************************************************************************
  * @file    stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
  * @file    stm32f4xx_it.c
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.  
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
#include "stm32f4xx_it.h"
#include "main.h"
#include "stm32f4x7_eth.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwip includes */
#include "lwip/sys.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile int pwm_count = 0;
extern xSemaphoreHandle s_xSemaphore;
extern xSemaphoreHandle exti_xSemaphore;
/* Private function prototypes -----------------------------------------------*/
extern void xPortSysTickHandler(void); 
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  xPortSysTickHandler(); 
}

/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Frame received */
  if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) 
  {
    /* Give the semaphore to wakeup LwIP task */
    xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );   
  }
	
  /* Clear the interrupt flags. */
  /* Clear the Eth DMA Rx IT pending bits */
  ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
  ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
	
  /* Switch tasks if necessary. */	
  if( xHigherPriorityTaskWoken != pdFALSE )
  {
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}

/* Set interrupt handlers */
/* Handle PD0 interrupt */
//https://community.st.com/s/question/0D50X00009XkZzrSAF/how-to-know-if-interrupt-was-triggered-by-falling-or-rising-edge
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
			
        /* Do your stuff when PD0 is changed */
				GPIOA->ODR = ((GPIOA->ODR)&(~0x20)) | ((~GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0))<<5);  //PA5
        
    }
}

/* Handle PD1 interrupt */
void EXTI1_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
      /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line1);
			
			/* Do your stuff when PD1 is changed */
				GPIOA->ODR = ((GPIOA->ODR)&(~0x8)) | ((~GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))<<3);		//PA3
        
    }
}
 
/* Handle PC10/11/12/13 interrupt */
void EXTI15_10_IRQHandler(void) {
	
		portBASE_TYPE xHigherPriorityTaskWoken;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line10);
			
        /* Do your stuff when PC10 is changed */
				GPIOE->ODR = ((GPIOE->ODR)&(~0x80)) | ((~GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10))<<7);		//PE7
        
    }
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line11);
			
        /* Do your stuff when PC11 is changed */
				GPIOD->ODR = ((GPIOD->ODR)&(~0x800)) | ((~GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11))<<11);	//PD11
        
    }
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
			
        /* Do your stuff when PC12 is changed */
				GPIOA->ODR = ((GPIOA->ODR)&(~0x100)) | ((~GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))<<8);		//PA8
        
    }
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
			
				xHigherPriorityTaskWoken = pdFALSE;
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line13);
			
        /* Do your stuff when PC13 is changed */
				GPIOE->ODR = ((GPIOE->ODR)&(~0x40)) | ((~GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))<<6);		//PE6
			
				if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1){
					xSemaphoreGiveFromISR( exti_xSemaphore, &xHigherPriorityTaskWoken );
				}
    }
}


/* Handle TIM3 interrupt */
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
      switch(pwm_count){
			case 0 : 
						TIM_SetAutoreload(TIM3, 1400);
						pwm_count++;
						break;
			case 1 : 
						TIM_SetAutoreload(TIM3, 1510);
						pwm_count++;
						break;
			case 2 : 
						TIM_SetAutoreload(TIM3, 1600);
						pwm_count++;
						break;
			case 3 : 
						TIM_SetAutoreload(TIM3, 1570);
						pwm_count++;
						break;
			case 4 : 
						TIM_SetAutoreload(TIM3, 1510);
						pwm_count++;
						break;
			case 5 : 
						TIM_SetAutoreload(TIM3, 1470);
						pwm_count++;
						break;
			case 6 : 
						TIM_SetAutoreload(TIM3, 1380);
						pwm_count++;
						break;
			case 7 : 
						TIM_SetAutoreload(TIM3, 1300);
						pwm_count++;
						break;
			case 8 : 
						TIM_SetAutoreload(TIM3, 1210);
						pwm_count++;
						break;
			case 9 : 
						TIM_SetAutoreload(TIM3, 1160);
						pwm_count++;
						break;
			case 10 : 
						TIM_SetAutoreload(TIM3, 1130);
						pwm_count++;
						break;
			case 11 : 
						TIM_SetAutoreload(TIM3, 1170);
						pwm_count++;
						break;
			case 12 : 
						TIM_SetAutoreload(TIM3, 1090);
						pwm_count++;
						break;
			case 13 : 
						TIM_SetAutoreload(TIM3, 1090);
						pwm_count++;
						break;
			case 14 : 
						TIM_SetAutoreload(TIM3, 1333);
						pwm_count=0;
						break;
			default:
						break;

}		
		
		
	}
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
