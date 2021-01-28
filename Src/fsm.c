#include "fsm.h"
#include "buttons.h"
#include "stdlib.h"
#include "screen.h"
#include "gcode.h"
#include "nuts_bolts.h"



#define steps_z 30
#define steps_x 1

bool g_lock = false;



extern bool feed_direction;
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern const uint8_t Menu_size;

bool demo = true;

void do_fsm_menu(state_t* s){
	uint8_t level = Thread_Info[Menu_Step].level;
	switch(buttons_flag_set) {
	case single_click_Msk3: {
		feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
		menu_changed = 1;
		break;
	}
	case single_click_Msk2: {
		feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
		menu_changed = 1;
		break;
	}
	case single_click_Msk: {
		int end_pos = 0; // todo
		if(end_pos != 0) {
			// first pass of thread cut was complete, so just use single click
			//	to switch between modes to process all other cuts

//			z_move(feed_direction, s->end_pos, s->main_feed_direction == feed_direction ? true : false, true);
//			if(demo)
//				z_move(feed_direction, s->end_pos, false, true); //test case, always async
//			else
//				z_move(feed_direction, s->end_pos, s->main_feed_direction == feed_direction ? true : false, true);
//			z_move(feed_direction, 400*2, false, true);
		} else { // controller in initial state, scroll menu
			s->function = do_fsm_menu_lps;
			for (int a = Menu_Step+1; a<Menu_size; a++) {
				if(Thread_Info[a].level == level) {
					Menu_Step = a;
					menu_changed = 1;
					break;
				}
			}
			if(menu_changed != 1) {
				for (int a = 0; a<Menu_Step; a++) {
					if(Thread_Info[a].level == level) {
						Menu_Step = a;
						menu_changed = 1;
						break;
					}
				}
			}
		}
		break;
	}
	case double_click_Msk: {
		feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
		menu_changed = 1;
		break;
	}
	case (long_press_start_Msk | long_press_start_Msk2): { // two buttons long pressed same time
		// todo check if it work
		break;
	}
	case long_press_start_Msk: {
		if(s->function == do_fsm_menu_lps){
			for (int a = 0; a<Menu_size; a++) {
				if(Thread_Info[a].level == Thread_Info[Menu_Step].submenu) {
					Menu_Step = a;
					menu_changed = 1;
					break;
				}
			}
		} 
		break;
	}
	case long_press_end_Msk: {
//		if(s->function == do_fsm_move)
//			s->function = do_long_press_end_callback;
		break;
	}
	}
}

void do_long_press_end_callback(state_t* s){          // direct movement: first pass, thread recording: long press release callback
	// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
	// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
	// не перескакивая на соседние шаги при потере питания.
//	if(s->end_pos == 0) //s->sync?
//		s->end_pos = ( s->ramp_step + s->current_pos ) | (step_divider - 1);
//	s->function = do_fsm_move;
//	do_fsm_move(s);
}



void do_fsm_menu_lps(state_t* s){
}

void do_fsm_wait_sclick(state_t* s){
}

