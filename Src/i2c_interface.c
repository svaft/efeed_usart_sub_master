#include "i2c_interface.h"
#include "string.h"
#include "main.h"


#define USER_BUTTON_DEBOUNCE 5
sample_log_t i2c_device_logging;
I2C_TypeDef *hi2c_ref;

volatile	uint32_t dma_delay = 0;
uint32_t 	dma_delay2 = 0;
uint8_t		device_ready = 0;
uint8_t		dma_data[6];

__IO uint8_t  ubTransferComplete = 1;

/**
 * Sent the first command to init the device
 * @param hi2c
 * @return
 */
error_code_t i2c_device_init(I2C_TypeDef *hi2c){

  hi2c_ref = hi2c;
	uint8_t  handShake[2];
	handShake[0]=0xf0;
	handShake[1]=0x55;
  LL_mDelay(250);

	Handle_I2C_MasterDMA_IT(hi2c, i2c_device_id, handShake, 2, 10);

//Handle_I2C_MasterDMA_IT
	
//	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		LL_mDelay(250);
//	}

//	Handle_I2C_MasterDMA_IT(hi2c, i2c_device_id, handShake, 2, 10);
	
//	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2) != HAL_OK){
//		return ERROR_INIT_I2C;
//	}
	LL_mDelay(20);
//	LL_mDelay(20);
	dma_delay = 0;
//	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
		LL_mDelay(1);
//	}	
//	LL_mDelay(100);
	handShake[0]=0xfb;
	handShake[1]=0x00;

	dma_delay = 0;
	Handle_I2C_MasterDMA_IT(hi2c, i2c_device_id, handShake, 2, 10);

//if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2)!= HAL_OK){
		dma_delay++;
//		return ERROR_INIT_I2C;
//	}
	LL_mDelay(20);
//	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//	}	
//	LL_mDelay(100);
	device_ready = 1;
	return ERROR_OK;

	
	
	/*
  hi2c_ref = hi2c;
	uint8_t  handShake[2];
	handShake[0]=0xf0;
	handShake[1]=0x55;
  LL_mDelay(250);

	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		LL_mDelay(250);
	}
	
	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2) != HAL_OK){
		return ERROR_INIT_I2C;
	}
	LL_mDelay(20);
//	LL_mDelay(20);
	dma_delay = 0;
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		dma_delay++;
		LL_mDelay(1);
	}	
//	LL_mDelay(100);
	handShake[0]=0xfb;
	handShake[1]=0x00;

	dma_delay = 0;
	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2)!= HAL_OK){
		dma_delay++;
		return ERROR_INIT_I2C;
	}
	LL_mDelay(20);
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		LL_mDelay(1);
	}	
//	LL_mDelay(100);
	device_ready = 1;
	return ERROR_OK;
*/
	return ERROR_OK;
}

error_code_t reqest_sample_i2c_dma(I2C_TypeDef *hi2c){

	if (device_ready == 0)
		return ERROR_OK;
	uint8_t cmd[1];
	cmd[0]=0x00;
//	dma_delay = 0;
//	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
//		LL_mDelay(1);
//	}	

	Handle_I2C_MasterDMA_IT(hi2c, i2c_device_id, cmd, 1, 10);
//	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,cmd, 1) != HAL_OK){
//		return ERROR_I2C_SAMPLE;
//	}
	
//	dma_delay = 0;
//	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
//		LL_mDelay(1);
//	}	
//	LL_mDelay(1);

//	dma_delay = 0;

//	if(hi2c_ref->hdmarx->State == HAL_DMA_STATE_READY)

//HAL_I2C_Master_Receive_DMA(hi2c_ref, i2c_device_id, dma_data,6);

//	dma_delay = 0;
//	LL_mDelay(1);
//	while(hi2c_ref->hdmarx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
//		LL_mDelay(1);
//	}
//	dma_delay2 = dma_delay;

return ERROR_OK;

}

/**
 * Reads the sample from the i2c devices. It reads tree accels and two buttons.
 * @param hi2c - i2c object
 * @param sample - out put sample pointer
 * @return error code
 */
error_code_t read_sample_i2c(i2c_sample_t *sample){
//	LL_I
/*
	if (device_ready == 0) //|| 	hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY)
		return ERROR_OK;
	uint8_t cmd[1];
	uint8_t data[6];
///	memset(&dma_data,);
	cmd[0]=0x00;

	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		LL_mDelay(1);
	}

	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,cmd, 1) != HAL_OK){
		return ERROR_I2C_SAMPLE;
	}
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		LL_mDelay(1);
	}

	if (HAL_I2C_Master_Receive(hi2c_ref, i2c_device_id, data,6,0x1000) != HAL_OK){
		return ERROR_I2C_SAMPLE;
	}
	sample->joy_x = data[0];
	sample->joy_y = data[1];
	sample->accel_x = (data[2] << 2)|((data[5] >> 2) & 0x03) ;
	sample->accel_y = (data[3] << 2)|((data[5] >> 4) & 0x03) ;
	sample->accel_z = (data[4] << 2)|((data[5] >> 6) & 0x03) ;
	sample->button_c=(data[5]&0x02)>>1;
	sample->button_z=data[5]&0x01;

	dma_data[5] = data[5];
	*/
	return ERROR_OK;
}
/**
 * touggle the value value of the temp pointer
 * @param temp
 */
void toggle(uint8_t* temp){
	if (*temp > 0){
		*temp = 0;
	}else {
		*temp = 1;
	}
}

/*
void sampling_task(void const * argument)
{

	button_status_t last_button_status = BUTTON_RELEASED;
	logging_status_t sampling_status = 0;
	uint8_t sampling_time_counter = 0;

	I2C_HandleTypeDef *hi2c2 = (I2C_HandleTypeDef*)argument;
//	read_flash((uint8_t*)FLASH_DATA_ADDR_START, (uint8_t*)&i2c_device_logging, sizeof(i2c_device_logging));

	i2c_device_init(hi2c2);

	for(;;)
	  {

//		if (check_user_button() == BUTTON_PRESSED && last_button_status == BUTTON_RELEASED){
//			toggle(&sampling_status);
//		}
		//update the last button status
//		last_button_status = check_user_button();


		if(sampling_status == LOGGING){
			if (sampling_time_counter > 10){ //10 times 50 miliseconds = 0.5 second
				sampling_time_counter = 0;
				if (i2c_device_logging.index >= SAMPLE_SIZE){
					i2c_device_logging.index = 0;
				}
				else i2c_device_logging.index++;

				read_sample_i2c(hi2c2,&i2c_device_logging.sample[i2c_device_logging.index]);
//				write_flash((uint8_t*)&i2c_device_logging, sizeof(i2c_device_logging));

//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // just to see if is logging
			}
		else sampling_time_counter++;
		}
//		vTaskDelay(50);
	  }
}

*/


/**
  * @brief  This Function handle Master events to perform a transmission then a reception process
  * @note   This function is composed in different steps :
  *         -1- Configure DMA parameters for Command Code transfer.
  *         -2- Enable DMA transfer.
  *         -3- Prepare acknowledge for Master data reception.
  *         -4- Initiate a Start condition to the Slave device.
  *         -5- Loop until end of transfer completed (DMA TC raised).
  *         -6- Prepare acknowledge for Master data reception.
  *         -7- Initiate a ReStart condition to the Slave device.
  *         -8- Loop until end of transfer completed (DMA TC raised).
  *         -9- Generate a Stop condition to the Slave device.
  *         -10- Clear pending flags, Data Command Code are checking into Slave process.
  * @param  None
  * @retval None
  */
void Handle_I2C_Master_Receive(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *data, uint16_t count, uint8_t timeout)
{
  /* Reset ubMasterTransferComplete flag */
  ubTransferComplete = 0;

	ubI2C_slave_addr = address;


  /* (6) Configure DMA to receive data from slave *****************************/
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, 6);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

  /* (6) Prepare acknowledge for Master data reception ************************/
  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

  /* (7) Initiate a ReStart condition to the Slave device *********************/
  /* Master Request direction READ */
  ubMasterRequestDirection = I2C_REQUEST_READ;

  /* Master Generate ReStart condition */
  LL_I2C_GenerateStartCondition(I2Cx);

  /* (8) Loop until end of transfer completed (DMA TC raised) *****************/
#if (USE_TIMEOUT == 1)
//  Timeout = DMA_SEND_TIMEOUT_TC_MS;
#endif /* USE_TIMEOUT */

  /* Loop until DMA transfer complete event */
  while(!ubTransferComplete)
  {
#if (USE_TIMEOUT == 1)
		int Timeout = timeout;
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED to blinking mode */
//        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  }
  /* (9) Generate a Stop condition to the Slave device ************************/
  LL_I2C_GenerateStopCondition(I2Cx);

  /* (10) Clear pending flags, Data Command Code are checking into Slave process */
  /* Disable Last DMA bit */
  LL_I2C_DisableLastDMA(I2Cx);

  /* Disable acknowledge for Master next data reception */
  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);

  /* End of Master Process */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
  /* Display through external Terminal IO the Slave Answer received */
//  printf("%s : %s\n\r", (char*)(aCommandCode[ubMasterCommandIndex][0]), (char*)aMasterReceiveBuffer);

  /* Turn LED2 On */
  /* Master sequence completed successfully*/
//  LED_On();
  /* Keep LED2 On, 500 MilliSeconds */
//  LL_mDelay(500);
// LED_Off();

  /* Clear and Reset process variables and arrays */
  ubTransferComplete = 0;
//  ubNbDataToTransmit = 0;
//  ubReceiveIndex     = 0;
//  FlushBuffer8(aMasterReceiveBuffer);
}





/**
  * @brief  This Function handle Master events to perform a transmission process
  * @note  This function is composed in different steps :
  *        -1- Enable DMA transfer.
  *        -2- Prepare acknowledge for Master data reception.
  *        -3- Initiate a Start condition to the Slave device.
  *        -4- Loop until end of transfer completed (DMA TC raised).
  *        -5- End of tranfer, Data consistency are checking into Slave process.
  * @param  None
  * @retval None
  */
//void Handle_I2C_MasterDMA_IT(void)
int Handle_I2C_MasterDMA_IT(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *data, uint16_t count, uint8_t timeout)
{
	#ifdef _SIMU
		return 0;
	#endif	
	ubI2C_slave_addr = address;
	ubMasterRequestDirection = I2C_REQUEST_WRITE;


	if(ubTransferComplete == 1){
		ubTransferComplete = 0;
	} else {
  	while(!ubTransferComplete)
  	{}
//		return 1;
	}
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, count);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6, (uint32_t)data, (uint32_t)LL_I2C_DMA_GetRegAddr(I2Cx), LL_DMA_DIRECTION_MEMORY_TO_PERIPH ); //  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);

  /* (1) Enable DMA transfer **************************************************/
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
  /* (2) Prepare acknowledge for Master data reception ************************/
  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

  /* (3) Initiate a Start condition to the Slave device ***********************/
  /* Master Generate Start condition */
  LL_I2C_GenerateStartCondition(I2Cx);

  /* (4) Loop until end of transfer completed (DMA TC raised) *****************/

#if (USE_TIMEOUT == 1)
  int Timeout = timeout;
#endif /* USE_TIMEOUT */

  /* Loop until DMA transfer complete event */
  while(!ubTransferComplete)
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED to blinking mode */
        return -1;
      }
    }
#endif /* USE_TIMEOUT */
  }

	return 0;
  /* (5) End of tranfer, Data consistency are checking into Slave process *****/
}

int Handle_I2C_MasterDMA_IT_async(uint8_t address, uint8_t *data, uint16_t count)
{
	#ifdef _SIMU
	return 0;
	#endif	

	ubI2C_slave_addr = address;
  ubMasterRequestDirection = I2C_REQUEST_WRITE;

	if(ubTransferComplete == 1){
		ubTransferComplete = 0;
	} else {
		return 1;
	}
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, count);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6, (uint32_t)data, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C_port), LL_DMA_DIRECTION_MEMORY_TO_PERIPH ); //  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);

  /* (1) Enable DMA transfer **************************************************/
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
  /* (2) Prepare acknowledge for Master data reception ************************/
  LL_I2C_AcknowledgeNextData(I2C_port, LL_I2C_ACK);

  /* (3) Initiate a Start condition to the Slave device ***********************/
  /* Master Generate Start condition */
  LL_I2C_GenerateStartCondition(I2C_port);
	return 0;
}




/**
  * @brief  This function Activate I2C2 peripheral (Master)
  * @note   This function is used to :
  *         -1- Enable I2C2.
  *         -2- Enable I2C2 transfer event/error interrupts.
  * @param  None
  * @retval None
  */
void Activate_I2C_Master(void)
{
  /* (1) Enable I2C_port **********************************************************/
  LL_I2C_Enable(I2C_port);

  /* (2) Enable I2C2 transfer event/error interrupts:
   *  - Enable Events interrupts
   *  - Enable Errors interrupts
  */
  LL_I2C_EnableIT_EVT(I2C_port);
  LL_I2C_EnableIT_ERR(I2C_port);
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void Transfer_Complete_Callback()
{
  /* Generate Stop condition */

	while(!LL_I2C_IsActiveFlag_BTF(I2C_port))
	{
	}
  LL_I2C_GenerateStopCondition(I2C_port);
//  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
  /* DMA transfer completed */
  ubTransferComplete = 1;
}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void Transfer_Error_Callback()
{
  /* Disable DMA1_Channel6_IRQn */
  NVIC_DisableIRQ(DMA1_Channel6_IRQn);
  /* Error detected during DMA transfer */
  Error_Handler();
}
