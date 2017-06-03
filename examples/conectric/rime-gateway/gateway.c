/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Testing the abc layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "debug.h"

#include "dev/serial-line.h"

#include "dev/adc-sensor.h"

#include <stdio.h>

#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *gmacp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *gmacp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *gmacp = (__code unsigned char *)0xFFE8;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(gateway_abc_process, "Gateway");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &gateway_abc_process,
    &serial_in_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  static uint16_t len;
  static char * packetbuf;

  len = packetbuf_hdrlen();
  packetbuf = (char *)packetbuf_hdrptr();

  printf("> ");
  while(len--) puthex(*packetbuf++);

  len = packetbuf_datalen();
  packetbuf = (char *)packetbuf_dataptr();

  while(len--) puthex(*packetbuf++);
  printf("\n");
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gateway_abc_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static char * mydata = NULL;
  static uint8_t ext_addr[8];
  signed char i;

  PROCESS_BEGIN();

  /*
   * Read IEEE address from flash, store in ext_addr.
   * Invert endianness (from little to big endian)
   */
  for(i = 7; i >= 0; --i) {
    ext_addr[i] = *gmacp;
    gmacp++;
  }

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    //printf("Serial_RX: %s (len=%d)\n", (char*)data, strlen(data));
    mydata = (char*)data;
    if (mydata[0] == 'M') {
      if (mydata[1] == 'R') {
        for(i = 0; i <= 7; i++) printf("%02x", ext_addr[i]);
      }
      printf("\n");
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{

}
