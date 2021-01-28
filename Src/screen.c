#include "screen.h"
#include "ssd1306.h"
#include "i2c_interface.h"

extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern bool feed_direction;
extern __IO uint8_t  ubTransferComplete;






//	uint8_t submenu;
//	char Text[6];
//	char Unit[6];
//	uint8_t level;
//	char infeed_mm[6];
//	char infeed_inch[6];
//	uint8_t infeed_strategy;



// основное меню. Считаем по формуле:
// Enc_Line/(Step_Per_Revolution/Feed_Screw*Thread_mm)
// перегенерация есть в excel файле
const THREAD_INFO Thread_Info[] = {
	{ 0, "0.10", "mm", 	0, 	".65", 	".026", 0	,"G01 Z-200 F0.1\r\n"},
	{ 0, "0.05", "mm", 	0, "", 	"", 0 ,"G01 Z-200 F0.05\r\n"},
	{ 0, "0.01", "mm", 	0, "", 	"", 0 ,"G01 Z-200 F0.01\r\n"},
	{ 0, "1.50", "mm", 	0, 	".95", 	".037", 0 ,"G33 Z-200 K1.5\r\n"},
	{ 0, "2.50", "mm", 	0, 	".65", 	".026", 1 ,"G33 Z-200 K2.5\r\n"},
	{ 0, "3.00", "mm", 	0, 	"", 		"", 		0 ,"G33 Z-200 K3.0\r\n"},
	{ 0, "0.20", "mm", 	0, 	"", 		"", 		0	,"G01 Z-200 F0.2\r\n"},
	{ 10,"T", 	 "mm", 	0, 	"", 		"", 		0 ,""},
	{ 0, "1.25", "mm", 	10, ".79", ".031", 	0 ,"G33 Z-200 K1.25\r\n"},
	{ 0, "1.75", "mm", 	10, "1.11", ".044", 0 ,"G33 Z-200 K1.75\r\n"},
	{ 0, "2.00", "mm", 	10, "1.26", ".050", 0 ,"G33 Z-200 K2.0\r\n"},
	{ 0, "0.50", "mm", 	10, ".34", ".013", 	0 ,"G33 Z-200 K0.5\r\n"},
	{ 0, "0.75", "mm", 	10, ".50", 	".020", 0 ,"G33 Z-200 K0.75\r\n"},
	{ 0, "0.01", "mm", 	10, "", 	"", 0 ,"G01 Z-200 F0.01\r\n"},
	{ 0, "0.05", "mm", 	10, "", 	"", 0 ,"G01 Z-200 F0.05\r\n"},

	{ 0,	"T27", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K0.941\r\n"},
	{ 0,	"T26", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K0.977\r\n"},
	{ 0,	"T24", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K1.058\r\n"},
	{ 0,	"T22", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K1.155\r\n"},
	{ 0,	"T20", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K1.27\r\n"},
	{ 0,	"T19", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K1.337\r\n"},
	{ 0,	"T18", "tpi",	10, "", 		"", 		0 ,"G33 Z-200 K1.411\r\n"},
	{ 0,  "..",  "up",  10, "", 		"", 		0 ,""},
	};


uint8_t Menu_Step = 0;																					// выборка из массива по умолчанию (1.5mm)
const uint8_t Menu_size = sizeof(Thread_Info)/sizeof(Thread_Info[0]);





void init_screen(I2C_TypeDef *hi2c){
	SSD1306_Init(hi2c);
}


char * utoa_builtin_div_1(uint32_t value, char *buffer)
{
	buffer += 11;
// 11 байт достаточно для десятичного представления 32-х байтного числа и завершающего нуля
	*--buffer = 0;
	do {
		*--buffer = value % 10 + '0';
		value /= 10;
	} while (value != 0);
	return buffer;
}

void print_info(void){
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(0, 16*0);

	SSD1306_Puts2("fw: 2.0", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

//	SSD1306_GotoXY(0, 16*3);

	
}

int update_screen(void){
#ifndef _SIMU
	if(ubTransferComplete == 0) {
		return 1;
	}
	SSD1306_Fill(SSD1306_COLOR_BLACK);
// first line
	SSD1306_GotoXY(0, 16*0);
	feed_direction == feed_direction_left ? SSD1306_Putc2big(left_arrow, &consolas_18ptFontInfo) : SSD1306_Putc2big(right_arrow, &consolas_18ptFontInfo);
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].Unit, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].infeed_inch, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // infeed recommendation

	char text_buffer[11];

	SSD1306_GotoXY(SSD1306_WIDTH - 20, 16*0);
	if(jog1resolution == 0){
		SSD1306_Puts2("x1", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	} else {
		SSD1306_Puts2("x10", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	}

	
//	SSD1306_GotoXY(SSD1306_WIDTH - 16, 0);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.mode, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode

//	SSD1306_GotoXY(SSD1306_WIDTH - 60, 16);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.current_pos, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode

//	SSD1306_GotoXY(SSD1306_WIDTH - 60, 32);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.ramp_step, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode
/*
	if(i2c_device_logging.sample[i2c_device_logging.index].button_c > 0){
		SSD1306_GotoXY(SSD1306_WIDTH - 60, 32);
		SSD1306_Puts2(utoa_builtin_div_1(i2c_device_logging.sample[i2c_device_logging.index].button_c, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode
	}
*/
// second line

	SSD1306_GotoXY(0, 16*1); //Устанавливаем курсор в позицию 0;16. Сначала по горизонтали, потом вертикали.
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].Text, &microsoftSansSerif_20ptFontInfo, SSD1306_COLOR_WHITE);
	//			SSD1306_GotoXY(50, 16*1);
	//			SSD1306_Puts2(Thread_Info[Menu_Step].infeed_mm, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	//			SSD1306_GotoXY(50, 16*2);
	//			SSD1306_Puts2(Thread_Info[Menu_Step].infeed_inch, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

// line 3
	SSD1306_GotoXY(SSD1306_WIDTH - 60, 16);
	int32_t jog1local = jog1;
	if(jog1local < 0){
		SSD1306_Putc2big(right_arrow, &consolas_18ptFontInfo);
		jog1local = -jog1local;
	} else if(jog1local > 0) {
		SSD1306_Putc2big(left_arrow, &consolas_18ptFontInfo);
	}
//	SSD1306_GotoXY(SSD1306_WIDTH - 54, 16*2);
	SSD1306_Puts2(utoa_builtin_div_1(jog1local, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

// line 4
	SSD1306_GotoXY(0, 16*3);
	switch(Thread_Info[Menu_Step].infeed_strategy) {
	case 0:
		SSD1306_Puts2("radial", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	case 1:
		SSD1306_Puts2("flank", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	case 2:
		SSD1306_Puts2("incremental", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	}


/*
	if(auto_mode == true) {
		SSD1306_GotoXY(SSD1306_WIDTH - 32, 0);
		SSD1306_Putc2big('A', &microsoftSansSerif_12ptFontInfo);
//  SSD1306_Putc2big(auto_symbol, &consolas_18ptFontInfo);
	}
*/
#endif /* _SIMU */
	return 0;
}
