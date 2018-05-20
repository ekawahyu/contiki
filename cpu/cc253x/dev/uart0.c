/**
 * \file
 *
 *   uart0 write routines
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
#include "dev/uart0.h"

#if UART0_ENABLE
#include "dev/uart-arch.h"
/*---------------------------------------------------------------------------*/
void
uart0_init()
{
#if UART0_CONF_HIGH_SPEED
  UART_SET_SPEED(0, UART_460_M, UART_460_E);
#else
  UART_SET_SPEED(0, UART_9_M, UART_9_E);
#endif

#ifdef UART0_ALTERNATIVE_2
  PERCFG |= PERCFG_U0CFG;  /* alternative port 2 = P1.5-2 */
#ifdef UART0_RTSCTS
  P1SEL |= 0x3C;    /* peripheral select for TX and RX, RTS, CTS */
  P1DIR |= 0x08;    /* RTS out */
  P1DIR &= 0x04;    /* CTS in */
#else
  P1SEL |= 0x30;    /* peripheral select for TX and RX */
  P1 &= ~0x08;      /* RTS down */
#endif
  P1DIR |= 0x20;    /* TX out */
  P1DIR &= ~0x10;   /* RX in */
#else
  PERCFG &= ~PERCFG_U0CFG; /* alternative port 1 = P0.5-2 */
#ifdef UART0_RTSCTS
  P0SEL |= 0x3C;    /* peripheral select for TX and RX, RTS, CTS */
  P0DIR |= 0x20;    /* RTS out */
  P0DIR &= 0x10;    /* CTS in */
#else
  P0SEL |= 0x0C;    /* peripheral select for TX and RX */
  P0 &= ~0x20;      /* RTS down */
#endif
  P0DIR |= 0x08;    /* TX out */
  P0DIR &= ~0x04;   /* RX in */
#endif

#ifdef UART0_RTSCTS
  U0UCR = 0x42; /* defaults: 8N1, RTS/CTS, high stop bit */
#else
  U0UCR = 0x02; /* defaults: 8N1, no flow control, high stop bit */
#endif

  U0CSR = UCSR_MODE; /* UART mode */
  U0UCR |= 0x80; /* Flush */
  UART0_RX_EN();

  UART0_RX_INT(1);
}
/*---------------------------------------------------------------------------*/
/* Write one byte over the UART. */
void
uart0_writeb(uint8_t byte)
{
  // U0CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
  U0DBUF = byte;
  // while(!(U0CSR & UCSR_TX_BYTE)); /* Wait until byte has been transmitted. */
  while(U0CSR & UCSR_ACTIVE); /* Wait until byte has been transmitted. */
  // U0CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
}
/*---------------------------------------------------------------------------*/
void
uart0_config(uint8_t config)
{
  /* select baud rate */
  if ((config & 0xC0) == UART_B2400)
    UART_SET_SPEED(0, UART_2_M, UART_2_E);
  if ((config & 0xC0) == UART_B4800)
    UART_SET_SPEED(0, UART_4_M, UART_4_E);
  if ((config & 0xC0) == UART_B9600)
    UART_SET_SPEED(0, UART_9_M, UART_9_E);
  if ((config & 0xC0) == UART_B19200)
    UART_SET_SPEED(0, UART_19_M, UART_19_E);

  /* if parity is enabled */
  if (config & 0x30) {
    U0UCR |= 0x18; /* 9-bit data width, enable parity */
    if ((config & 0x30) == UART_PARITY_ODD)
      U0UCR &= ~0x20;
    if ((config & 0x30) == UART_PARITY_EVEN)
      U0UCR |= 0x20;
  }
  else {
    U0UCR &= ~0x18; /* 8-bit data width, disable parity */
  }

  /* select stop bit */
  U0UCR &= ~0x04;
  U0UCR |= (config & 0x04);
}
/*---------------------------------------------------------------------------*/
#endif
