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
#include "fixedptc.h"
#include "buttons.h"
#include "fsm.h"
#include "i2c_interface.h"
#include "ssd1306.h"
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

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
//      if(auto_mode_delay > 0)
//              auto_mode_delay--;
	for(int a = 0; a<BT_TOTAL;a++){
		if( bt[a].buttons_mstick > 0 )
			bt[a].buttons_mstick++;
	}
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
  if(LL_DMA_IsActiveFlag_TC6(DMA1))
  {
    LL_DMA_ClearFlag_GI6(DMA1);
		
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
		LL_DMA_ClearFlag_TC6(DMA1);
    Transfer_Complete_Callback();
//    DMA1_Transfer_Complete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE6(DMA1))
  {
    Transfer_Error_Callback();
  }

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
  if(LL_DMA_IsActiveFlag_TC7(DMA1))
  {
    LL_DMA_ClearFlag_GI7(DMA1);

		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
		LL_DMA_ClearFlag_TC7(DMA1);

    Transfer_Complete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE7(DMA1))
  {
    Transfer_Error_Callback();
  }

  /* USER CODE END DMA1_Channel7_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	TIM4->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	if(LL_TIM_GetDirection(TIM4) == LL_TIM_COUNTERDIRECTION_UP){
		if(jog1resolution == 0){
			jog1++;
			jog1cmd = jog1l;
		}
		else{
			jog1+=10;
			jog1cmd = jog1L;
		}
	} else {
		if(jog1resolution == 0){
			jog1--;
			jog1cmd = jog1r;
		}
		else{
			jog1-=10;
			jog1cmd = jog1R;
		}
	}
	menu_changed = 1;
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
  /* Check SB flag value in ISR register */
  if(LL_I2C_IsActiveFlag_SB(I2C1))
  {
    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
    LL_I2C_TransmitData8(I2C1, ubI2C_slave_addr | ubMasterRequestDirection);

    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a ubMasterRequestDirection request */
//    LL_I2C_TransmitData8(I2C2, SLAVE_OWN_ADDRESS | ubMasterRequestDirection);
		
  }
  /* Check ADDR flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
    /* Verify the transfer direction */
    if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
    {
      ubMasterXferDirection = LL_I2C_DIRECTION_READ;

      if(ubMasterNbDataToReceive == 1)
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

        /* Enable DMA transmission requests */
        LL_I2C_EnableDMAReq_RX(I2C1);
      }
      else if(ubMasterNbDataToReceive == 2)
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

        /* Enable Pos */
        LL_I2C_EnableBitPOS(I2C1);
      }
      else
      {
        /* Enable Last DMA bit */
        LL_I2C_EnableLastDMA(I2C1);

        /* Enable DMA transmission requests */
        LL_I2C_EnableDMAReq_RX(I2C1);
      }
    } else {
			/* Enable DMA transmission requests */
			LL_I2C_EnableDMAReq_TX(I2C1);
		}
    /* Clear ADDR flag value in ISR register */
    LL_I2C_ClearFlag_ADDR(I2C1);
  }

  /* USER CODE END I2C1_EV_IRQn 0 */

  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */
  Error_Handler();

  /* USER CODE END I2C1_ER_IRQn 0 */

  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
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



void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
	{
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
		USART_CharReception_Callback();
	} else {
		
//		while(1);
//		Error_Handler();
	}

  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
