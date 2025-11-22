/*
 * ext_btn.c
 *
 *  Created on: Nov 20, 2025
 *      Author: trishamarieherras
 */

#include <stm32f4xx.h>
#include <stm32f411xe.h>
#include <stdio.h>
#include <ext_btn.h>




void EXT_BTN_init(button_t btn) {
	RCC->AHB1ENR |= (1 << 0); // enable peripheral CLK

	// initialize GPIO pin PA2
	GPIOA->MODER &= ~(0b11 << 4); // input mode
	GPIOA->PUPDR &= ~(0b11 << 4); // no pull-up no pull-down

	// configure interrupt
	EXTI->IMR |= (1 << 2);
	EXTI->RTSR &= ~(1 << 2); // no rising edge detection
	EXTI->FTSR |=  (1 << 2); // enable falling edge detection

	NVIC_EnableIRQ(EXTI2_IRQn);

	EXT_BTN_reset(btn);
}

void EXT_BTN_init_TIMER(void) {
	// let's do TIM5 for not reason
	RCC->APB1ENR |= (1 << 3); // enable TIM5 peripheral CLK

	TIM5->PSC =  49999; // set timer CLK @ 1 kHz

	TIM5->CR1 &= ~(11 << 8); // no CLK division
	TIM5->CR1 &= ~(11 << 5); // follow DIR bit
	TIM5->CR1 &= ~(1 << 4); // DIR: up-counting
	TIM5->CR1 |=  (1 << 4); // one-pulse mode

	TIM5->ARR = 1000; // stop after 1 sec

	TIM5->DIER |= (1 << 0); // UEV interrupt enable
	NVIC_EnableIRQ(TIM5_IRQn);

	TIM5->CR1 &= ~(1 << 0); // disable timer for now
}

void EXT_BTN_reset(button_t button) {
	button.status = NOT_PRESSED;
	button.event = BTN_RELEASE;
	button.long_press = 0;
}
