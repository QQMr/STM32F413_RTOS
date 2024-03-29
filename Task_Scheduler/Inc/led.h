/*
 * led.h
 *
 *  Created on: 28-Dec-2019
 *      Author: nieki
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f413xx.h"

#define LED_GREEN  			0
#define LED_RED 			1
#define LED_GREEN_SECOND  	2

#define DELAY_COUNT_1MS 		 1250U
#define DELAY_COUNT_1S  		1000//(1000U * DELAY_COUNT_1MS)
#define DELAY_COUNT_500MS  		500//(500U  * DELAY_COUNT_1MS)
#define DELAY_COUNT_250MS 		250//(250U  * DELAY_COUNT_1MS)
#define DELAY_COUNT_125MS 		125//(125U  * DELAY_COUNT_1MS)

void led_init_all(void);
void led_on(uint8_t led_no);
void led_off(uint8_t led_no);
void delay(uint32_t count);

#endif /* LED_H_ */
