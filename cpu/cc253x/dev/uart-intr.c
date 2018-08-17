/**
 * \file
 *
 *   uart write routines
 *
 * \author
 *
 *   Anthony "Asterisk" Ambuehl
 *
 *   interrupt routines which must be in HOME bank.  handles received data from UART.
 *
 */
#include "cc253x.h"

#include "dev/uart0.h"
#include "dev/uart1.h"
#include "sys/energest.h"

#if UART0_ENABLE
static int (* uart0_input_handler)(unsigned char c);
extern uint8_t uart0_bitmask;
#endif
#if UART1_ENABLE
static int (* uart1_input_handler)(unsigned char c);
extern uint8_t uart1_bitmask;
#endif

#if UART0_ENABLE
/*---------------------------------------------------------------------------*/
void
uart0_set_input(int (* input)(unsigned char c))
{
  uart0_input_handler = input;
}
/*---------------------------------------------------------------------------*/
#if UART0_CONF_WITH_INPUT
/* avoid referencing bits since we're not using them */
#if defined(__SDCC_mcs51) || defined(SDCC_mcs51)
#pragma save
#if CC_CONF_OPTIMIZE_STACK_SIZE
#pragma exclude bits
#endif
#endif
#if defined __IAR_SYSTEMS_ICC__
#pragma vector=URX0_VECTOR
__near_func __interrupt void uart0_rx_isr(void)
#else
void
uart0_rx_isr(void) __interrupt(URX0_VECTOR)
#endif
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  URX0IF = 0;
  if(uart0_input_handler != NULL) {
    uart0_input_handler(U0DBUF & uart0_bitmask);
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#if defined(__SDCC_mcs51) || defined(SDCC_mcs51)
#pragma restore
#endif
#endif
#endif /* UART0_ENABLE */
#if UART1_ENABLE
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (* input)(unsigned char c))
{
  uart1_input_handler = input;
}
/*---------------------------------------------------------------------------*/
#if UART1_CONF_WITH_INPUT
/* avoid referencing bits since we're not using them */
#if defined(__SDCC_mcs51) || defined(SDCC_mcs51)
#pragma save
#if CC_CONF_OPTIMIZE_STACK_SIZE
#pragma exclude bits
#endif
#endif
#if defined __IAR_SYSTEMS_ICC__
#pragma vector=URX1_VECTOR
__near_func __interrupt void uart1_rx_isr(void)
#else
void
uart1_rx_isr(void) __interrupt(URX1_VECTOR)
#endif
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  URX1IF = 0;
  if(uart1_input_handler != NULL) {
    uart1_input_handler(U1DBUF & uart1_bitmask);
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#if defined(__SDCC_mcs51) || defined(SDCC_mcs51)
#pragma restore
#endif
/*---------------------------------------------------------------------------*/
#endif /* UART1_CONF_WITH_INPUT */
#endif /* UART1_ENABLE */
