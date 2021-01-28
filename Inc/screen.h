/*
 * screen.h
 *
 */

#ifndef SCREEN_H_
#define SCREEN_H_

//#include "stm32f1xx_hal.h"
#include "main.h"

#define auto_symbol 0
#define left_arrow	1
#define right_arrow 2

typedef struct {
	uint8_t submenu;
	char Text[6];
	char Unit[6];
	uint8_t level;
	char infeed_mm[6];
	char infeed_inch[6];
	uint8_t infeed_strategy;
	char cmd[64];
} THREAD_INFO;

extern const THREAD_INFO Thread_Info[];
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)


void init_screen(I2C_TypeDef *hi2c);
int update_screen(void);
void print_info(void);


#endif /* SCREEN_H_ */
