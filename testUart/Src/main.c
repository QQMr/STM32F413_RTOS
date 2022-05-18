// Standard library includes.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
//#ifdef VVC_F1
//  #include "stm32f1xx.h"
//#elif VVC_L4
//  #include "stm32l4xx.h"
//#endif

//#define USE_INTERRUPT_USART6_BURTON
#include "stm32f413xx.h"

volatile uint8_t isReceiveEvent = 0;

uint32_t SystemCoreClock = 0;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

//// Reset handler: set the stack pointer and branch to main().
//__attribute__( ( naked ) ) void reset_handler( void ) {
//  // Set the stack pointer to the 'end of stack' value.
//  __asm__( "LDR r0, =_estack\n\t"
//           "MOV sp, r0" );
//  // Branch to main().
//  __asm__( "B main" );
//}

/**
 * Main program.
 */
int main( void ) {
//  // Copy initialized data from .sidata (Flash) to .data (RAM)
//  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
//  // Clear the .bss section in RAM.
//  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Set the core system clock speed.

    // Enable floating-point unit.
    //SCB->CPACR    |=  ( 0xF << 20 );

    // Default clock source is the "multi-speed" internal oscillator.
    // Switch to the 16MHz HSI oscillator.
    RCC->CR |=  ( RCC_CR_HSION );
    while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
    RCC->CFGR &= ~( RCC_CFGR_SW );
    RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
    while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
    SystemCoreClock = 16000000;



    // Enable peripheral clocks: GPIOA, USART2.
    RCC->APB2ENR |= ( RCC_APB2ENR_USART6EN );//RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
    RCC->AHB1ENR  |= ( RCC_AHB1ENR_GPIOGEN );//RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN );
    // Configure pins A2, A15 for USART2 (AF7).
    GPIOG->MODER    &= ~( ( 0x3 << ( 9 * 2 ) ) |
                          ( 0x3 << ( 14 * 2 ) ) );
    GPIOG->MODER    |=  ( ( 0x2 << ( 9 * 2 ) ) |
                          ( 0x2 << ( 14 * 2 ) ) );
    GPIOG->OTYPER   &= ~( ( 0x1 << 9 ) |
                          ( 0x1 << 14 ) );
    GPIOG->OSPEEDR  &= ~( ( 0x3 << ( 9 * 2 ) ) |
                          ( 0x3 << ( 14 * 2 ) ) );
    GPIOG->OSPEEDR  |=  ( ( 0x2 << ( 9 * 2 ) ) |
                          ( 0x2 << ( 14 * 2 ) ) );

    GPIOG->AFR[ 1 ] &= ~( ( 0xF << ( ( 9 - 8 ) * 4 ) ) );
    GPIOG->AFR[ 1 ] |=  ( ( 0x8 << ( ( 9 - 8 ) * 4 ) ) );
    GPIOG->AFR[ 1 ] &= ~( ( 0xF << ( ( 14 - 8 ) * 4 ) ) );
    GPIOG->AFR[ 1 ] |=  ( ( 0x8 << ( ( 14 - 8 ) * 4 ) ) );

#ifdef USE_INTERRUPT_USART6_BURTON
    // Setup the NVIC to enable interrupts.
	 // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
	 NVIC_SetPriorityGrouping( 0 );
	 // UART receive interrupts should be high priority.
	 uint32_t uart_pri_encoding = NVIC_EncodePriority( 0, 1, 0 );
	 NVIC_SetPriority( USART6_IRQn, uart_pri_encoding );
	 NVIC_EnableIRQ( USART6_IRQn );
#endif

  // Set the baud rate to 9600.
  uint16_t uartdiv = SystemCoreClock / 9600;

  USART6->BRR = uartdiv;

  // Enable the USART peripheral.
#ifndef USE_INTERRUPT_USART6_BURTON
  USART6->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
#else
  USART6->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE);
#endif
  // Main loop: wait for a new byte, then echo it back.
  char rxb = 'A';
  while ( 1 ) {
    // Receive a byte of data.
#ifndef USE_INTERRUPT_USART6_BURTON
      while( !( USART6->SR & USART_SR_RXNE ) ) {};
      rxb = USART6->DR;

    // Re-transmit the byte of data once the peripheral is ready.
#define USART_FLAG_TXE	((uint16_t) 0x0080)
      while( !( USART6->SR & USART_SR_TXE ) ) {};
      USART6->DR = rxb;
#endif
  }
}


// USART2 interrupt handler
void USART6_IRQHandler( void ) {

    // 'Receive register not empty' interrupt.
#ifdef USE_INTERRUPT_USART6_BURTON
	char rxb = 'A';
    if ( USART6->SR & USART_SR_RXNE ) {
      // Copy new data into the buffer.
      rxb = USART6->DR;
      USART6->DR = rxb+1;
    }
#endif

}
