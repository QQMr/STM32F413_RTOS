
#include<stdint.h>
#include "led.h"



void delay(uint32_t count)
{
  for(uint32_t i = 0 ; i < count ; i++);
}

void led_init_all(void)
{

//	uint32_t *pRccAhb1enr = (uint32_t*)0x40023830;
//	uint32_t *pGpiodModeReg = (uint32_t*)0x40020C00;
//
//
//	*pRccAhb1enr |= ( 1 << 3);
//	//configure LED_GREEN
//	*pGpiodModeReg |= ( 1 << (2 * LED_GREEN));
//	*pGpiodModeReg |= ( 1 << (2 * LED_ORANGE));
//	*pGpiodModeReg |= ( 1 << (2 * LED_RED));
//	*pGpiodModeReg |= ( 1 << (2 * LED_BLUE));
//
//#if 0
//	//configure the outputtype
//	*pGpioOpTypeReg |= ( 1 << (2 * LED_GREEN));
//	*pGpioOpTypeReg |= ( 1 << (2 * LED_ORANGE));
//	*pGpioOpTypeReg |= ( 1 << (2 * LED_RED));
//	*pGpioOpTypeReg |= ( 1 << (2 * LED_BLUE));
//#endif
//
//    led_off(LED_GREEN);
//    led_off(LED_ORANGE);
//    led_off(LED_RED);
//    led_off(LED_BLUE);

	/*Enable GREEN LED*/
    uint32_t *pRccAhb1enr = (uint32_t*)0x40023830;
    uint32_t *pGpiodModeReg = (uint32_t*)0x40020800;

    *pRccAhb1enr |= ( 1 << 2);
	//configure LED_GREEN
    *pGpiodModeReg |= ( 1 << (2 * 5));//*pGpiodModeReg |= ( 1 << (2 * LED_GREEN));

    uint32_t *pGpiodDataReg = (uint32_t*)0x40020814;
    *pGpiodDataReg |= ( 1 << 5);


    /*Enable Red LED*/
    uint32_t *pGpiodModeReg2 = (uint32_t*)0x40021000;
    *pRccAhb1enr |= ( 1 << 4);
	//configure LED_GREEN
    *pGpiodModeReg2 |= ( 1 << (2 * 3));//*pGpiodModeReg |= ( 1 << (2 * LED_GREEN));

    uint32_t *pGpiodDataReg2 = (uint32_t*)0x40021014;
    *pGpiodDataReg2 |= ( 1 << 3);


    /*Enable Green LED 2*/
    uint32_t *pGpiodModeReg3 = (uint32_t*)0x40020400;
    *pRccAhb1enr |= ( 1 << 1);
	//configure Green LED 2
    *pGpiodModeReg3 |= ( 1 << (2 * 12));//*pGpiodModeReg |= ( 1 << (2 * LED_GREEN));

    uint32_t *pGpiodDataReg3 = (uint32_t*)0x40020414;
    *pGpiodDataReg3 |= ( 1 << 12);



}

void led_on(uint8_t led_no)
{
  uint32_t *pGpiodDataReg = (uint32_t*)0x40020C14;
  *pGpiodDataReg |= ( 1 << led_no);

}

void led_off(uint8_t led_no)
{
	  uint32_t *pGpiodDataReg = (uint32_t*)0x40020C14;
	  *pGpiodDataReg &= ~( 1 << led_no);

}


