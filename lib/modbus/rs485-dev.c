/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: dmacro.dm,v 1.11 2006/03/18 19:43:52 nfi Exp $
 */

/**
 * \file
 *         A brief description of what this file is
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 */
#include "contiki.h"
#include "dev/hwconf.h";
#include "dev/uart0.h"
#include <stdio.h>

HWCONF_PIN(RS485_TRANSMIT, 4, 5);

/* a small input buffer that */
static int pos;
unsigned char rs485_buffer[64];

static int input_handler(unsigned char c) {
  rs485_buffer[pos++] = c;
  /* wrap buffer if it overflows... */
  if(pos >= 64) {
    pos = 0;
  }
  return 1;
}

void rs485_input_reset() {
  pos = 0;
}

int rs485_input_len() {
  return pos;
}

int
rs485_transmit(uint8_t *data, int len) {
  int i;
  RS485_TRANSMIT_SET();
  /* a few NOP's just to wait some before sending data */
  clock_delay(10);

  for(i = 0; i < len; i++) {
    uart0_writeb(data[i]);
  }

  while(uart0_active())
    clock_delay(2);

  RS485_TRANSMIT_CLEAR();
  return len;
}

void
rs485_init() {

  RS485_TRANSMIT_SELECT();
  RS485_TRANSMIT_MAKE_OUTPUT();

  printf("Starting up test RS485 \n");

  UCA0CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
  UCA0CTL1 |= UCSSEL_2;           /* CLK = SMCLK */
  /* reconfigure UART speed */
  UCA0BR0  = 0xA0;
  UCA0BR1 = 0x01;
  UCA0MCTL = UCBRS_0; 			//Changed from 0x6D and seems to work.  Liam 
  /* UCA0MCTL = 0x5B; */

  /* UCA0BR0 = 0x45;                 /\* 8MHz/115200 = 69 = 0x45 *\/ */
  /* UCA0BR0 = 0x45; /\* tested... *\/ */
  /* /\* UCA0BR0 = 9; *\/ */
  /* UCA0BR1 = 0x00; */
  /* UCA0MCTL = UCBRS_3;             /\* Modulation UCBRSx = 3 *\/ */


  /* XXX Clear pending interrupts before enable */
  UCA0IE &= ~UCRXIFG;
  UCA0IE &= ~UCTXIFG;
 

  UCA0CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
  UCA0IE |= UCRXIE;                        /* Enable UCA0 RX interrupt */
  /* set the input handler */
  
  uart0_set_input(&input_handler);
}
