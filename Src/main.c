/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"

#include <string.h>
#include <math.h>

#include "buttons.h"

//#define ARM_MATH_CM3
//#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

__IO uint8_t ubI2C_slave_addr = 0;
__IO uint8_t ubMasterRequestDirection  = 0;


//int count;
extern bool demo;
/*
G18 - XZ-plane
G98 - retract to the position that axis was in just before this series of one or more contiguous canned cycles was started
G50 set spindle speed
G28, G28.1 Go to Predefined Position
G54-G59.3 Select Coordinate System
G96, G97 Spindle Control Mode, G97 (RPM Mode)
*/


// ***** Stepper Motor *****

bool menu_changed = false;

int32_t jog1 = 0, jog2 = 0, jog3 = 0, jog4 = 0; 
int8_t jog1resolution = 0, jog1cmd = 0, jog2resolution = 0, jog2cmd = 0, jog3resolution = 0, jog3cmd = 0, jog4resolution = 0, jog4cmd = 0; 

//#define _SIMU
bool auto_mode = false;
int32_t auto_mode_delay = -1; // default delay between change direction is 6 secons








#define RX_BUFFER_SIZE   64
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
uint8_t aRXBuffer[RX_BUFFER_SIZE];
uint8_t aRXBuffer_screen[RX_BUFFER_SIZE];
__IO uint32_t	uwNbReceivedChars;
__IO uint8_t	uNbReceivedCharsForUser;
__IO uint32_t	uwNbReceivedCharsB;
__IO uint8_t	uNbReceivedCharsForUserB;

		
__IO uint32_t	uwBufferReadyIndication;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;
__IO uint8_t ubUARTReceptionCompleteA = 0;
__IO uint8_t ubUARTReceptionCompleteB = 0;

/* Buffer used for transmission */
const uint8_t aTxBuffer[] = "ok\r\n";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t  ubMasterXferDirection     = 0;
uint8_t       aMasterReceiveBuffer[0xF] = {0};
__IO uint8_t  ubMasterNbDataToReceive   = sizeof(aMasterReceiveBuffer);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART_CharReception_Callback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/**
  * @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed 
  * @param  None
  * @retval None
  */
//void DMA1_ReceiveComplete_Callback(void)
//{
  // DMA Rx transfer completed:
//  ubUART2ReceptionComplete = 1;
//}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	uint8_t symbol = LL_USART_ReceiveData8(USART_controller);
	if(symbol == '\n'){
    /* Set Buffer swap indication */
		if(uwNbReceivedChars == 1){
			memset(&aRXBufferA,0,RX_BUFFER_SIZE);
			uwNbReceivedChars = 0;
			return;
		}

		aRXBufferA[uwNbReceivedChars++] = symbol;
		ubUARTReceptionCompleteA = 1;
		uNbReceivedCharsForUser = uwNbReceivedChars;
		memset(&aRXBuffer,0,RX_BUFFER_SIZE);
		memcpy(&aRXBuffer,&aRXBufferA,uwNbReceivedChars);
		memset(&aRXBufferA,0,RX_BUFFER_SIZE);		
    uwNbReceivedChars = 0;
	} else {
		if(symbol != 0)
			aRXBufferA[uwNbReceivedChars++] = symbol;
	}
}


// get data from external device(phone, bluetooth)
void USART_CharReception_Callback_UP(void)
{
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	uint8_t symbol = LL_USART_ReceiveData8(USART_screen);
	if(symbol == '\n'){
    /* Set Buffer swap indication */
		if(uwNbReceivedCharsB == 1){
			memset(&aRXBufferB,0,RX_BUFFER_SIZE);
			uwNbReceivedCharsB = 0;
			return;
		}
			
		aRXBufferB[uwNbReceivedCharsB++] = symbol;
		ubUARTReceptionCompleteB = 1;
		uNbReceivedCharsForUserB = uwNbReceivedCharsB;
		memset(&aRXBuffer_screen,0,RX_BUFFER_SIZE);
		memcpy(&aRXBuffer_screen,&aRXBufferB,uwNbReceivedCharsB);
		memset(&aRXBufferB,0,RX_BUFFER_SIZE);
    uwNbReceivedCharsB = 0;
	} else {
		if(symbol != 0)
			aRXBufferB[uwNbReceivedCharsB++] = symbol;
	}
}


void Print2Screen(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USART_screen))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in DR register */
    LL_USART_TransmitData8(USART_screen, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART_screen))
  {
  }
}


void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USART_controller))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in DR register */
    LL_USART_TransmitData8(USART_controller, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART_controller))
  {
  }
}

//#define _USEENCODER
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	init_buttons();

	// Enable DMA TX Interrupt
//  LL_USART_EnableDMAReq_TX(USART_controller);
  // Enable DMA Channel Tx
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

  // Clear Overrun flag, in case characters have already been sent to USART
  LL_USART_ClearFlag_ORE(USART_controller);
  // Enable RXNE and Error interrupts
  LL_USART_EnableIT_RXNE(USART_controller);
  LL_USART_EnableIT_ERROR(USART_controller);


// Clear Overrun flag, in case characters have already been sent to USART
  LL_USART_ClearFlag_ORE(USART_screen);
  // Enable RXNE and Error interrupts
  LL_USART_EnableIT_RXNE(USART_screen);
  LL_USART_EnableIT_ERROR(USART_screen);

// init encoder4:

	LL_TIM_ClearFlag_UPDATE(TIM4);
	LL_TIM_EnableUpdateEvent(TIM4);
	LL_TIM_EnableIT_UPDATE(TIM4);
	LL_TIM_EnableCounter(TIM4);

	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableUpdateEvent(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);


	LL_GPIO_AF_EnableRemap_TIM2();
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableUpdateEvent(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);


/*
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
	
	LL_TIM_ENCODER_InitTypeDef TIM_EncoderInitStruct;
	LL_TIM_ENCODER_StructInit(&TIM_EncoderInitStruct);
	TIM_EncoderInitStruct.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;
	TIM_EncoderInitStruct.IC1Filter = LL_TIM_IC_FILTER_FDIV32_N8;
	TIM_EncoderInitStruct.IC2Filter = LL_TIM_IC_FILTER_FDIV32_N8;
	
	LL_GPIO_AF_EnableRemap_TIM2();
	LL_TIM_ENCODER_Init(TIM2, &TIM_EncoderInitStruct);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableIT_CC2(TIM2);
*/	
//	LL_TIM_EnableIT_UPDATE(TIM2);
//	LL_TIM_EnableAllOutputs(TIM2);


  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		process_button();
		switch(buttons_flag_set) {
			case long_press_start_FF:
				PrintInfo("G0 X-200\r\n",10);
				break;
			case long_press_start_FB:
				PrintInfo("G0 X200\r\n",9);
				break;
			case long_press_start_FL:
				PrintInfo("G0 Z-200\r\n",10);
				break;
			case long_press_start_FR:
				PrintInfo("G0 Z200\r\n",9);
				break;
			case long_press_end_FF:
			case long_press_end_FB:
			case long_press_end_FL:
			case long_press_end_FR:
				PrintInfo("!S\r\n",4);
				break;
		}		
		buttons_flag_set = 0; // reset button flags

		if(ubUARTReceptionCompleteA == 1){
			Print2Screen(aRXBuffer,uNbReceivedCharsForUser);
			ubUARTReceptionCompleteA = 0;
		}

		if(ubUARTReceptionCompleteB == 1){
			PrintInfo(aRXBuffer_screen,uNbReceivedCharsForUserB);
			ubUARTReceptionCompleteB = 0;
		}

		if(jog2cmd > 0){
			switch(jog2cmd){
				case jog_cw:
					PrintInfo("!w\r\n",4);
					Print2Screen("!w\r\n",4);
					break;
				case jog_ccw:
					PrintInfo("!s\r\n",4);
					Print2Screen("!s\r\n",4);
					break;
			}
			jog2cmd = 0;
		}

		if(jog3cmd > 0){
			switch(jog3cmd){
				case jog_cw:
					PrintInfo("!t\r\n",4);
					Print2Screen("!t\r\n",4);
					break;
				case jog_ccw:
					PrintInfo("!g\r\n",4);
					Print2Screen("!g\r\n",4);
					break;
			}
			jog3cmd = 0;
		}
		

		if(jog4cmd > 0){
			switch(jog4cmd){
				case jog1l:
					PrintInfo("!a\r\n",4);
					Print2Screen("!a\r\n",4);
					break;
				case jog1r:
					PrintInfo("!d\r\n",4);
					Print2Screen("!d\r\n",4);
					break;
				case jog1L:
					PrintInfo("!A\r\n",4);
					break;
				case jog1R:
					PrintInfo("!D\r\n",4);
					break;
			}
			jog4cmd = 0;
		}
// update display info
		if(menu_changed == 1){
//			menu_changed = update_screen();
		}

			
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(32000000);
  LL_SetSystemCoreClock(32000000);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  */
  GPIO_InitStruct.Pin = ENC1_A_5v_Pin|ENC1_B_5v_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PA15   ------> TIM2_CH1
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = ENC2_A_5v_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ENC2_A_5v_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC2_B_5v_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ENC2_B_5v_GPIO_Port, &GPIO_InitStruct);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM3 GPIO Configuration
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = ENC3_A_3v_Pin|ENC3_B_3v_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  PB7   ------> TIM4_CH2
  */
  GPIO_InitStruct.Pin = ENC4_A_5v_Pin|ENC4_B_5v_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FR_Pin|FL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FB_Pin|FF_Pin|LEFT_TOP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_12
                          |LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_4
                          |LL_GPIO_PIN_5|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*timIC)
{
	if (timIC->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

	}

//	if(ready!=SET)return;
//HAL_TIM_ReadCapturedValue(timIC,TIM_CHANNEL_3);
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	TIM1->CR1 = TIM2->CR1 = TIM3->CR1 = TIM4->CR1 = 0;
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
