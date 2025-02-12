/*
 * buttons.h
 *
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

//#include "stm32f1xx_hal.h"
#include "main.h"


#define BT_TOTAL	5
// BB_VAR must be greater then 0x20000000 + RW-data (see at build output after linking). 
// if there are error L6971E just enlarge BB_VAR variable
//#define BB_VAR   	0x20004000 
#define BB_VAR 		SRAM_BASE + 0xEC//5440 //0x4000
#define BITBAND_SRAM(a,b) ((SRAM_BB_BASE + (a-SRAM_BASE)*32 + (b*4)))  // Convert SRAM address


// Button timing variables
#ifdef _SIMU
#define DEBOUNCE_MS 1                 // ms debounce period to prevent flickering when pressing or releasing the button
#define DOUBLECLICK_GAP_MS 5                       // max ms between clicks for a double click event
#define HOLDTIME_MS 40                // ms hold period: how long to wait for press+hold event
#define CLICKTIME_MS 10 
#else
#define DEBOUNCE_MS 20                 // ms debounce period to prevent flickering when pressing or releasing the button
#define DOUBLECLICK_GAP_MS 110//150                       // max ms between clicks for a double click event
//#define HOLDTIME_MS 500                // ms hold period: how long to wait for press+hold event
#define CLICKTIME_MS 250 
#endif
//#define LONGHOLDTIME_MS 3000       // ms long hold period: how long to wait for press+hold event

#define long_press_start_Pos	(0U)
#define long_press_start_Msk  (0x1U << long_press_start_Pos)
#define long_press_start_Msk2	(long_press_start_Msk << 1*4)
#define long_press_start_Msk3	(long_press_start_Msk << 2*4)
#define long_press_start_Msk4	(long_press_start_Msk << 3*4)
#define long_press_start_Msk5	(long_press_start_Msk << 4*4)
#define long_press_start_Msk6	(long_press_start_Msk << 5*4)
#define long_press_start_Msk7	(long_press_start_Msk << 6*4)
#define long_press_start_Msk8	(long_press_start_Msk << 7*4)


#define long_press_start_FF	long_press_start_Msk
#define long_press_start_FB	long_press_start_Msk2
#define long_press_start_FL	long_press_start_Msk3
#define long_press_start_FR	long_press_start_Msk4
#define long_press_start_LEFT_TOP	long_press_start_Msk5


#define long_press_end_Pos    (1U)
#define long_press_end_Msk    (0x1U << long_press_end_Pos)
#define long_press_end_Msk2    (long_press_end_Msk << 1*4)
#define long_press_end_Msk3    (long_press_end_Msk << 2*4)
#define long_press_end_Msk4    (long_press_end_Msk << 3*4)
#define long_press_end_Msk5    (long_press_end_Msk << 4*4)
#define long_press_end_Msk6    (long_press_end_Msk << 5*4)
#define long_press_end_Msk7    (long_press_end_Msk << 6*4)
#define long_press_end_Msk8    (long_press_end_Msk << 7*4)

#define long_press_end_FF			long_press_end_Msk
#define long_press_end_FB			long_press_end_Msk2
#define long_press_end_FL			long_press_end_Msk3
#define long_press_end_FR			long_press_end_Msk4
#define long_press_end_LEFT_TOP			long_press_end_Msk5


#define single_click_Pos      (2U)
#define single_click_Msk      (0x1U << single_click_Pos)
#define single_click_Msk2      (single_click_Msk << 1*4)
#define single_click_Msk3      (single_click_Msk << 2*4)
#define single_click_Msk4      (single_click_Msk << 3*4)
#define single_click_Msk5      (single_click_Msk << 4*4)
#define single_click_Msk6      (single_click_Msk << 5*4)
#define single_click_Msk7      (single_click_Msk << 6*4)
#define single_click_Msk8      (single_click_Msk << 7*4)
//#define single_click_CENTER				single_click_Msk
#define single_click_LEFT_TOP			single_click_Msk5
#define single_click_FF			single_click_Msk
#define single_click_FB			single_click_Msk2
#define single_click_FL			single_click_Msk3
#define single_click_FR			single_click_Msk4
#define single_click_LEFT_TOP			single_click_Msk5
#define single_click_RIGHT_BOTTOM		single_click_Msk6

//#define single_click_RIGHT_TOP		single_click_Msk3
//#define single_click_LEFT_BOTTOM	single_click_Msk4
//#define single_click_RIGHT_BOTTOM	single_click_Msk5


#define double_click_Pos      (3U)
#define double_click_Msk      (0x1U << double_click_Pos)
#define double_click_Msk2      (double_click_Msk << 1*4)
#define double_click_Msk3      (double_click_Msk << 2*4)
#define double_click_Msk4      (double_click_Msk << 3*4)
#define double_click_Msk5      (double_click_Msk << 4*4)
#define double_click_Msk6      (double_click_Msk << 5*4)
#define double_click_Msk7      (double_click_Msk << 6*4)
#define double_click_Msk8      (double_click_Msk << 7*4)

#define double_click_LEFT_TOP			double_click_Msk5
//#define double_click_RIGHT_TOP		double_click_Msk3
//#define double_click_LEFT_BOTTOM	double_click_Msk4
//#define double_click_RIGHT_BOTTOM	double_click_Msk5


typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint32_t button_pin;
	uint32_t downTime;               // time the button was pressed down
	uint32_t buttons, buttons_mstick, buttons_flag, buttons_mask, clk_mode;
} BUTTON;

extern BUTTON bt[BT_TOTAL];


//extern volatile *uint32_t;
extern uint32_t buttons_flag_set;// __attribute__((at(BB_VAR)));
extern __IO uint8_t  ubTransferComplete;


//#define buttons_flag_setbb1 ((BITBAND_SRAM_BASE + (&buttons_flag_set-BITBAND_SRAM_REF)*32))  // Convert SRAM address
#define buttons_flag_setbb ((uint32_t *)((SRAM_BB_BASE  + ((BB_VAR)-SRAM_BASE)*32)))

//#define buttons_flag_setbb ((uint32_t *)((0x22000000  + ((BB_VAR)-0x20000000)*32)))
//uint32_t gap = 0;


void init_buttons(void);
void process_button(void);
//void process_joystick(void);
//uint32_t downTime = 0;               // time the button was pressed down
//uint32_t buttons = 0, buttons_mstick = 0, buttons_flag = 0, buttons_mask = 0;






#endif /* BUTTONS_H_ */
