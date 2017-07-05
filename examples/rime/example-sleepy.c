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
 *         Example for using the trickle code in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"

#include "dev/serial-line.h"

#include "dev/leds.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ConBurst");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &example_abc_process,
    &serial_in_process);
/*---------------------------------------------------------------------------*/
static const struct abc_callbacks abc_call = {NULL};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static uint8_t message[128];
  static uint8_t loop;
  static uint16_t counter = 0;
  static uint8_t len;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  /* Delay 0-60 seconds */
  etimer_set(&et, random_rand() % CLOCK_SECOND * 60);

  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  while(1) {

    /* Delay 60-61 seconds */
    etimer_set(&et, CLOCK_SECOND * 60 + random_rand() % CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    memset(message, 0, sizeof(message));
    counter++;
    message[0] = 6;         /* header len */
    message[1] = counter;   /* seqno */
    message[2] = 0;         /* hop count */
    message[3] = 0;         /* number of hops */
    message[4] = 0xFF;      /* destination addr H */
    message[5] = 0xFF;      /* destination addr L */
    message[6] = 8;         /* message length */
    message[7] = 0x01;      /* messsage type = sensor broadcast */
    message[8] = 0x10;      /* device type = RHT */
    message[9] = 0x21;      /* battery level */
    message[10] = 0xAA;     /* RH high */
    message[11] = 0xBB;     /* RH low */
    message[12] = 0xCC;     /* T high */
    message[13] = 0xDD;     /* T low */

    len = 14;
    packetbuf_copyfrom(message, len);

    loop = 5;
    while(loop--) {
      /* Delay 1-2 seconds */
      etimer_set(&et, CLOCK_SECOND + random_rand() % CLOCK_SECOND);
      packetbuf_copyfrom(message, len);
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, (uint8_t)counter);
      abc_send(&abc);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    printf("Serial_RX: %s\n", (char*)data);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
