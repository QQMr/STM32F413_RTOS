/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define MAX_TASKS	4
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);


void init_systick_timer(uint32_t tick_hz);
__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
void init_tasks_stack(void);

/* some stack memory calculation */
#define SIZE_TASK_STACK						1024U
#define SIZE_SCHED_STACK					1024U

#define SRAM_START							0X20000000U
#define SIZE_SRAM							( 125 * 1024 )
#define SRAM_END							(SRAM_START + SIZE_SRAM)

#define T1_STACK_START						SRAM_END
#define T2_STACK_START						(SRAM_END - 1 * SIZE_TASK_STACK)
#define T3_STACK_START						(SRAM_END - 2 * SIZE_TASK_STACK)
#define T4_STACK_START						(SRAM_END - 3 * SIZE_TASK_STACK)
#define SCHED_STACK_START					(SRAM_END - 4 * SIZE_TASK_STACK)

#define TICK_HZ 1000U

#define HSI_CLOCK 16000000U
#define SYSTICK_TIM_CLK HSI_CLOCK


uint32_t psp_of_tasks[MAX_TASKS] = {T1_STACK_START,T2_STACK_START,T3_STACK_START,T4_STACK_START};

uint32_t task_handlers[MAX_TASKS];

int main(void)
{
	printf("Hello Task Scheduler\n");
	init_scheduler_stack(SCHED_STACK_START);

	task_handlers[0] = (uint32_t) task1_handler;
	task_handlers[1] = (uint32_t) task2_handler;
	task_handlers[2] = (uint32_t) task3_handler;
	task_handlers[3] = (uint32_t) task4_handler;

	init_tasks_stack();
	init_systick_timer(TICK_HZ);
    /* Loop forever */
	printf("Hello World\n");
	for(;;);
}

void task1_handler(void)
{
	while(1)
	{
		printf("This is task1\n");
	}
}

void task2_handler(void)
{
	while(1)
	{
		printf("This is task2\n");
	}
}

void task3_handler(void)
{
	while(1)
	{
		printf("This is task3\n");
	}
}

void task4_handler(void)
{
	while(1)
	{
		printf("This is task4\n");
	}
}



void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;
	uint32_t count_value = SYSTICK_TIM_CLK/tick_hz;

	//load the value in to SVR
	*pSRVR = count_value;

	//do some settings
	*pSCSR |= (1 << 1); //Enable SysTick exception
	*pSCSR |= (1 << 2); //Indicates the clock source, processor clock source

	//enable the systick
	*pSCSR |= (1 << 0); //enable the counter

}

__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack)
{
	__asm volatile("MSR MSP,R0");
	__asm volatile("MSR MSP,%0": : "r" (sched_top_of_stack) );
	__asm volatile("BX LR");
}

#define DUMMY_XPSR 0X01000000U
void init_tasks_stack(void)
{
	uint32_t *pPSP;

	for(int i=0; i<MAX_TASKS; i++)
	{
		pPSP = (uint32_t *) psp_of_tasks[i];

		pPSP--; //XPSR
		*pPSP = DUMMY_XPSR; //0X01000000

		pPSP--; //PC
		*pPSP = task_handlers[i]; //0X01000000

		pPSP--; //LR
		*pPSP = 0xFFFFFFFD; //0X01000000

		for(int j=0 ; j<13; j++)
		{
			pPSP--;
			*pPSP = 0;
		}

		psp_of_tasks[i] = (uint32_t)pPSP;
	}

}


void SysTick_Handler(void)
{

}

