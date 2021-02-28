#include "main.h"
#include "buttons.h"
//#include "i2c_interface.h"


//uint32_t buttons_flag_set = 0;
uint32_t buttons_flag_set __attribute__((at(BB_VAR)));

uint32_t buttons_flag_set_prev = 0;
BUTTON bt[BT_TOTAL];

void init_buttons(void){
	int num = 0;
	bt[num].clk_mode = 10;
	bt[num].GPIOx = FF_GPIO_Port;// BUTTON_1_GPIO_Port;
	bt[num].button_pin = FF_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;

	num++;
	bt[num].clk_mode = 10;
	bt[num].GPIOx = FB_GPIO_Port;//BUTTON_LEFT_TOP_GPIO_Port;
	bt[num].button_pin = FB_Pin; //BUTTON_LEFT_TOP_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;
	
	num++;
	bt[num].clk_mode = 10;
	bt[num].GPIOx = FL_GPIO_Port;//BUTTON_RIGHT_TOP_GPIO_Port;
	bt[num].button_pin = FL_Pin; //BUTTON_RIGHT_TOP_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;

	num++; //4
	bt[num].clk_mode = 10;
	bt[num].GPIOx = FR_GPIO_Port;//BUTTON_LEFT_BOTTOM_GPIO_Port;
	bt[num].button_pin = FR_Pin; //BUTTON_LEFT_BOTTOM_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;

	num++; //5
	bt[num].clk_mode = 10;
	bt[num].GPIOx = LEFT_TOP_GPIO_Port;//BUTTON_RIGTH_BOTTOM_GPIO_Port;
	bt[num].button_pin = LEFT_TOP_Pin; //BUTTON_RIGTH_BOTTOM_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;
/*
	num++; //6
	bt[num].clk_mode = 10;
	bt[num].GPIOx = GPIO6_GPIO_Port;//BUTTON_JOG1_GPIO_Port;
	bt[num].button_pin = GPIO6_Pin; //BUTTON_JOG1_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;

	num++; //7
	bt[num].clk_mode = 10;
	bt[num].GPIOx = GPIO7_GPIO_Port;//BUTTON_JOG1_GPIO_Port;
	bt[num].button_pin = GPIO7_Pin; //BUTTON_JOG1_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;

	num++; //8
	bt[num].clk_mode = 10;
	bt[num].GPIOx = GPIO8_GPIO_Port;//BUTTON_JOG1_GPIO_Port;
	bt[num].button_pin = GPIO8_Pin; //BUTTON_JOG1_Pin;
	bt[num].buttons = bt[num].buttons_mask = LL_GPIO_IsInputPinSet(bt[num].GPIOx,bt[num].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;
*/

	return;
/*
	bt[1] = bt[0];
	bt[1].clk_mode = 10;
	bt[1].GPIOx = BUTTON_2_GPIO_Port;
	bt[1].button_pin = BUTTON_2_Pin;
	bt[1].buttons = bt[1].buttons_mask = bt[1].GPIOx->IDR & bt[1].button_pin;

	if(device_ready == 1){
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);		
		reqest_sample_i2c_dma();
//		while(ubTransferComplete == 0){
//		}
	}
	bt[2].clk_mode = 10;
	bt[2].button_pin = 0x02; // button_c code
	bt[2].buttons = bt[2].buttons_mask = dma_data[5]&bt[2].button_pin; // = bt[1].GPIOx->IDR & bt[1].button_pin;
	
	bt[3].clk_mode = 10;
	bt[3].button_pin = 0x01; // button_c code
	bt[3].buttons = bt[3].buttons_mask = dma_data[5]&bt[3].button_pin; // = bt[1].GPIOx->IDR & bt[1].button_pin;
*/
}

// реализация конечного автомата обработки событий кнопки
inline void process_button()
{
	for(int a =0; a<BT_TOTAL;a++){
	/*
	click Nondeterministic finite automaton(NFA):
	10. ждем сигнала с кнопки
	20. кнопка нажата, считаем тики. если тиков > 1000 это лонг пресс, идем в 30
	30. сигнал long_press_start, идем в 40
	40. ждем отпуска кнопки, далее в 50
	50. кнопку отпустили, если тиков меньше 200 идем в 70, иначе в 60
	60. если тиков < 1000 генерим сигнал CLICK, если тиков больше генерим сигнал long_press_end, идем в 10
	70. тиков меньше 200, это может быть дабл-клик, ждем еще 100, если ничего идем в 60, если клик идем в 80
	80. ждем отпуска кнопки, далее в 90
	90. кнопку отпустили, генерим DOUBLE_CLICK, идем в 10
	*/
 
//	#if defined ( _SIMU )
//		uint32_t tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin;
//	#else
		uint32_t tmp_buttons;
		if(bt[a].GPIOx != 0)
			tmp_buttons = LL_GPIO_IsInputPinSet(bt[a].GPIOx,bt[a].button_pin);
//			tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin; //BUTTON_1_GPIO_Port->IDR & bt[a].button_pin;
		else{
			
//			if(ubTransferComplete == 0)
//				continue;
			//	dma_delay = 0;
//	while(hi2c2->hdmarx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
//		HAL_Delay(1);
//	}
//	dma_delay2 = dma_delay;

			tmp_buttons = bt[a].button_pin;
		}
//	#endif

		if( tmp_buttons != bt[a].buttons ) { // start debounce
			bt[a].buttons = tmp_buttons;
			// reset debounce counter and start count every one ms
			bt[a].buttons_mstick = 1;
			return;
		}
#ifdef _SIMU
	#define released 0
		#else
	#define released 1
#endif		
		
		if( bt[a].buttons_mstick > DEBOUNCE_MS ) {
			switch(bt[a].clk_mode) {
			case 10: {
				if ( tmp_buttons == released ) {   // released
				} else { // pressed
//					buttons_mstick = 1;
					bt[a].clk_mode = 20;
				}
				break;
			}
			case 20: {
				if ( tmp_buttons == released ) { // released
					bt[a].clk_mode = 50;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				if (bt[a].downTime > CLICKTIME_MS ) { // long press detected
					bt[a].clk_mode = 30;
				}
				break;
			}
			case 30: { // long_press_start event
				buttons_flag_setbb[(a<<2)+long_press_start_Pos]  = 1; //long_press_start = 1;
				bt[a].clk_mode = 40;
				break;
			}
			case 40: {
				if ( tmp_buttons == released ) { //released
					bt[a].clk_mode = 50;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				break;
			}
			case 50: {
				bt[a].clk_mode = bt[a].downTime < CLICKTIME_MS ? 70 : 60;
				break;
			}
			case 60: {//60 if tick count < 1000 generate CLICK event, else generate long_press_end event, go to 10 state
				if(bt[a].downTime < CLICKTIME_MS) { //single CLICK event
					buttons_flag_setbb[(a<<2)+single_click_Pos]  = 1; //single_click = 1;
				} else { //  long_press_end event
					buttons_flag_setbb[(a<<2)+long_press_end_Pos]  = 1; //long_press_end = 1;
				}
				bt[a].downTime = bt[a].buttons_mstick = 0;
				bt[a].clk_mode = 10;
				break;
			}
			case 70: { //70. тиков меньше 200, это может быть дабл-клик, ждем нажатия еще 100, если ничего идем в 60, если клик идем в 80
				if ( tmp_buttons == released ) {
					bt[a].downTime = bt[a].buttons_mstick;
					if( bt[a].downTime > DOUBLECLICK_GAP_MS ) {
						bt[a].clk_mode = 60;
					}
				} else {
					bt[a].clk_mode = 80;
				}
				break;
			}
			case 80: {
				if ( tmp_buttons == released ) { // released
					bt[a].clk_mode = 90;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				break;
			}
			case 90: { // сигнал DOUBLE_CLICK
				buttons_flag_setbb[(a<<2)+double_click_Pos]  = 1; //double_click = 1;
				bt[a].clk_mode = 10;
				bt[a].buttons_mstick = 0;
				break;
			}
			}
		}
	}
//	ubTransferComplete = 0;
}
