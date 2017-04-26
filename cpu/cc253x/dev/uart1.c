/**
 * \file
 *
 *   uart1 write routines
 *
 * \author
 *
 *   Anthony "Asterisk" Ambuehl
 *
 */
#include <stdlib.h>
#include <string.h>

#include "cc253x.h"
#include "sfr-bits.h"
#include "dev/uart1.h"

#ifdef RS485_CONF_ENABLE
#include "dev/rs485-arch.h"
#endif

#if UART1_ENABLE
/*---------------------------------------------------------------------------*/
/* UART1 initialization */
void
uart1_init()
{
#if UART1_CONF_HIGH_SPEED
  UART_SET_SPEED(1, UART_460_M, UART_460_E);
#else
  UART_SET_SPEED(1, UART_9_M, UART_9_E);
#endif

#ifdef UART1_ALTERNATIVE_1
  PERCFG &= ~PERCFG_U1CFG; /* alternative port 1 = P0.5-2 */
#ifdef UART1_RTSCTS
  P0SEL |= 0x3C;    /* peripheral select for TX and RX, RTS, CTS */
  P0DIR |= 0x08;    /* RTS out */
  P0DIR &= 0x04;    /* CTS in */
#else
  P0SEL |= 0x30;    /* peripheral select for TX and RX */
  P0 &= ~0x08;      /* RTS down*/
#endif
  P0DIR |= 0x10;    /* TX out*/
  P0DIR &= ~0x20;   /* RX in*/
#else
  PERCFG |= PERCFG_U1CFG;  /* alternative port 2 = P1.7-4 */
#ifdef UART1_RTSCTS
  P1SEL |= 0xF0;    /* peripheral select for TX and RX */
  P1DIR |= 0x20;    /* RTS out */
  P1DIR &= 0x10;    /* CTS in */
#else
  P1SEL |= 0xC0;    /* peripheral select for TX and RX */
  P1 &= ~0x20;      /* RTS down */
#endif
  P1DIR |= 0x40;    /* TX out */
  P1DIR &= ~0x80;   /* RX in */
#endif

#ifdef UART1_RTSCTS
  U1UCR = 0x42; /* defaults: 8N1, RTS/CTS, high stop bit */
#else
  U1UCR = 0x02; /* defaults: 8N1, no flow control, high stop bit */
#endif

  U1CSR = UCSR_MODE; /* UART mode */
  U1UCR |= 0x80; /* Flush */
  UART1_RX_EN();

  UART1_RX_INT(1);

#ifdef RS485_CONF_ENABLE
  rs485_de_nre_init();
#endif
}
/*---------------------------------------------------------------------------*/
/* Write one byte over the UART. */
void
uart1_writeb(uint8_t byte)
{
  U1CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
#ifdef RS485_CONF_ENABLE
  rs485_de_nre_high();
#endif
  U1DBUF = byte;
  while(!(U1CSR & UCSR_TX_BYTE)); /* Wait until byte has been transmitted. */
#ifdef RS485_CONF_ENABLE
  rs485_de_nre_low();
#endif
  U1CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
}
/*---------------------------------------------------------------------------*/
#endif
