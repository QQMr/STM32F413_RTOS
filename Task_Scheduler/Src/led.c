
#include<stdint.h>
#include "led.h"


void delay(uint32_t count)
{
  for(uint32_t i = 0 ; i < count ; i++);
}

void led_init_all(void)
{

	/*Enable GREEN LED*/
    uint32_t *pRccAhb1enr = (uint32_t*)0x40023830;
    uint32_t *pGpiodModeReg = (uint32_t*)0x40020800;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //  *pRccAhb1enr |= ( 1 << 2);
	//configure LED_GREEN
    GPIOC->MODER |= ( 1 << (2 * 5));

    GPIOC->ODR |= ( 1 << 5);


    /*Enable Red LED*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //*pRccAhb1enr |= ( 1 << 4);
	//configure LED_GREEN
    GPIOE->MODER |= ( 1 << (2 * 3));

    GPIOE->ODR |= ( 1 << 3);


    /*Enable Green LED 2*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	//configure Green LED 2
    GPIOB->MODER |= ( 1 << (2 * 12));

    GPIOB->ODR |= ( 1 << 12);



}

void led_on(uint8_t led_no)
{
	if(led_no == 0)
	{
	  GPIOC->ODR |= ( 1 << 5);
	}
	else if(led_no == 1)
	{
	  GPIOE->ODR |= ( 1 << 3);
	}
	else
	{
	  GPIOB->ODR |= ( 1 << 12);
	}
}

void led_off(uint8_t led_no)
{
	if(led_no == 0)
	{
	  GPIOC->ODR &= ~( 1 << 5);
	}
	else if(led_no == 1)
	{
	  GPIOE->ODR &= ~( 1 << 3);
	}
	else
	{
	  GPIOB->ODR &= ~( 1 << 12);
	}

}


