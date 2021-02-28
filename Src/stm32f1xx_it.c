/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buttons.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	for(int a = 0; a<BT_TOTAL;a++){
		if( bt[a].buttons_mstick > 0 )
			bt[a].buttons_mstick++;
	}

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
//      if(auto_mode_delay > 0)
//              auto_mode_delay--;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	TIM1->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	if(LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_DOWN){
		jog1--;
		jog1cmd = jog_ccw;
	} else {
		jog1++;
		jog1cmd = jog_cw;
	}
	menu_changed = 1;

	
	////	debug7();
//	// enable corresponding channel for sub-step:
//	debug1();
//	TIM3->CCER = state_hw.substep_mask;
//	// start pulse:
//	LL_TIM_EnableCounter(TIM3); 
//	// stop sub-step timer:
//	LL_TIM_DisableCounter(TIM1);
//	LL_TIM_SetCounter(TIM1,0);
//	TIM1->SR = 0;
//	return;
  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
//  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
//  {
//    /* Clear the update interrupt flag*/
//    LL_TIM_ClearFlag_UPDATE(TIM1);
//  }

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	TIM2->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	if(LL_TIM_GetDirection(TIM2) == LL_TIM_COUNTERDIRECTION_DOWN){
		jog2--;
		jog2cmd = jog_ccw;
	} else {
		jog2++;
		jog2cmd = jog_cw;
	}
	menu_changed = 1;

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	TIM3->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	if(LL_TIM_GetDirection(TIM3) == LL_TIM_COUNTERDIRECTION_DOWN){
		jog3--;
		jog3cmd = jog_ccw;
	} else {
		jog3++;
		jog3cmd = jog_cw;
	}
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	TIM4->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	if(LL_TIM_GetDirection(TIM4) == LL_TIM_COUNTERDIRECTION_DOWN){
		if(jog4resolution == 0){
			jog4++;
			jog4cmd = jog1l;
		}
		else{
			jog4+=10;
			jog4cmd = jog1L;
		}
	} else {
		if(jog4resolution == 0){
			jog4--;
			jog4cmd = jog1r;
		}
		else{
			jog4-=10;
			jog4cmd = jog1R;
		}
	}
	menu_changed = 1;
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
	{
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
		USART_CharReception_Callback();
	} else {
		
//		while(1);
//		Error_Handler();
	}
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART3) && LL_USART_IsEnabledIT_RXNE(USART3))
	{
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
		USART_CharReception_Callback_UP();
	} else {
		
//		while(1);
//		Error_Handler();
	}

  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//void TIM1_UP_IRQHandler_old(void)
//{
//  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
////	debug7();
//	// enable corresponding channel for sub-step:
//	debug1();
//	TIM3->CCER = state_hw.substep_mask;
//	// start pulse:
//	LL_TIM_EnableCounter(TIM3); 
//	// stop sub-step timer:
//	LL_TIM_DisableCounter(TIM1);
//	LL_TIM_SetCounter(TIM1,0);
//	TIM1->SR = 0;
//	return;
//  /* USER CODE END TIM1_UP_IRQn 0 */
//  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
//  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
//  {
//    /* Clear the update interrupt flag*/
//    LL_TIM_ClearFlag_UPDATE(TIM1);
//  }

//  /* USER CODE END TIM1_UP_IRQn 1 */
//}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
