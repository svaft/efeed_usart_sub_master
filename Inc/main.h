/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_crc.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "stddef.h"
#include "stdlib.h"
#include "string.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


#define USART_screen USART3
#define USART_controller USART2
#define I2C_port	I2C1

extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern bool menu_changed;
extern bool jog;

extern int32_t jog1, jog2, jog3, jog4; 
extern int8_t jog1resolution, jog1cmd, jog2resolution, jog2cmd, jog3resolution, jog3cmd, jog4resolution, jog4cmd; 


#define jog_cw 1
#define jog_ccw 2


#define jog1l	1
#define jog1r	2
#define jog1L	3
#define jog1R	4

extern __IO uint8_t ubI2C_slave_addr;
extern __IO uint8_t  ubMasterRequestDirection;
extern __IO uint8_t  ubMasterXferDirection;
//uint8_t       aMasterReceiveBuffer[0xF] = {0};
extern __IO uint8_t  ubMasterNbDataToReceive;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USART_CharReception_Callback(void);
void USART_CharReception_Callback_UP(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define FR_Pin LL_GPIO_PIN_14
#define FR_GPIO_Port GPIOC
#define FL_Pin LL_GPIO_PIN_15
#define FL_GPIO_Port GPIOC
#define FB_Pin LL_GPIO_PIN_4
#define FB_GPIO_Port GPIOA
#define FF_Pin LL_GPIO_PIN_5
#define FF_GPIO_Port GPIOA
#define ENC3_A_3v_Pin LL_GPIO_PIN_6
#define ENC3_A_3v_GPIO_Port GPIOA
#define ENC3_B_3v_Pin LL_GPIO_PIN_7
#define ENC3_B_3v_GPIO_Port GPIOA
#define ENC1_A_5v_Pin LL_GPIO_PIN_8
#define ENC1_A_5v_GPIO_Port GPIOA
#define ENC1_B_5v_Pin LL_GPIO_PIN_9
#define ENC1_B_5v_GPIO_Port GPIOA
#define LEFT_TOP_Pin LL_GPIO_PIN_12
#define LEFT_TOP_GPIO_Port GPIOA
#define ENC2_A_5v_Pin LL_GPIO_PIN_15
#define ENC2_A_5v_GPIO_Port GPIOA
#define ENC2_B_5v_Pin LL_GPIO_PIN_3
#define ENC2_B_5v_GPIO_Port GPIOB
#define ENC4_A_5v_Pin LL_GPIO_PIN_6
#define ENC4_A_5v_GPIO_Port GPIOB
#define ENC4_B_5v_Pin LL_GPIO_PIN_7
#define ENC4_B_5v_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
// extract GPIO pin nuber by passing LL_GPIO_PIN_x value to it. 
// Use this value for compute bit-banding address of pin at compiling time by preprocessor


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
