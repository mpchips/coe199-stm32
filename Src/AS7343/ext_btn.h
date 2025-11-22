/*
 * ext_btn.h
 *
 *  Created on: Nov 20, 2025
 *      Author: trishamarieherras
 */

#ifndef SRC_AS7343_EXT_BTN_H_
#define SRC_AS7343_EXT_BTN_H_

typedef enum {
	BTN_IDLE,
	BTN_PRESS,
	BTN_RELEASE,
} btn_event_t;

typedef enum {
  NOT_PRESSED,
  SHORT_PRESS,
  LONG_PRESS,
} btn_press_t;

typedef struct {
	btn_press_t status;
	btn_event_t event;
	int 				long_press;
} button_t ;

void EXT_BTN_init(button_t btn);

void EXT_BTN_init_TIMER(void);

void EXT_BTN_reset(button_t button);

#endif /* SRC_AS7343_EXT_BTN_H_ */
