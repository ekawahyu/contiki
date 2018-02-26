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
 *         An example of how the Conectric primitive can be used.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include "dev/serial-line.h"

#include <stdio.h>
#include <string.h>

#define MESSAGE "Hello"
#define REPLIED "Replied"

static struct conectric_conn conectric;
process_event_t serial_event_message;
/*---------------------------------------------------------------------------*/
PROCESS(example_conectric_process, "Conectric example");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(&example_conectric_process, &serial_in_process);
/*---------------------------------------------------------------------------*/
static void
sent(struct conectric_conn *c)
{
  printf("packet sent\n");
}

static void
timedout(struct conectric_conn *c)
{
  printf("packet timedout\n");
}

static void
recv(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  printf("Data received from %d.%d: %.*s (%d)\n",
	 from->u8[0], from->u8[1],
	 packetbuf_datalen(), (char *)packetbuf_dataptr(), packetbuf_datalen());

  if (*(char *)packetbuf_dataptr() == 'H') {
    packetbuf_copyfrom(REPLIED, strlen(REPLIED));
    conectric_send(&conectric, from);
  }
}

const static struct conectric_callbacks callbacks = {recv, sent, timedout};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_conectric_process, ev, data)
{
  PROCESS_EXITHANDLER(conectric_close(&conectric);)
  PROCESS_BEGIN();

  conectric_open(&conectric, 132, &callbacks);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    linkaddr_t addr;

    /* Wait for button click before sending the first message. */
//    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
//
//    printf("Button clicked\n");

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_event_message && data != NULL);
    
    packetbuf_copyfrom(MESSAGE, strlen(MESSAGE));
    addr.u8[0] = *(uint8_t*)data;
    addr.u8[1] = 0;
    conectric_send(&conectric, &addr);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
//  static uint8_t * request;
//  static uint8_t counter;
//  static uint8_t hex_string[2];
//  static uint8_t bytereq[128];

  PROCESS_BEGIN();

  serial_event_message = process_alloc_event();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    printf("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    printf("%s\n", (uint8_t *)data);

    /* Broadcast event */
    process_post(PROCESS_BROADCAST, serial_event_message, data);
    /* Wait until all processes have handled the serial line event */
    if(PROCESS_ERR_OK ==
      process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) {
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
