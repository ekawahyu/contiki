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
#include "net/rime/trickle.h"
#include "random.h"

#include "dev/button-sensor.h"
#include "dev/serial-line.h"

#include "dev/leds.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ConBurst");
PROCESS(example_trickle_process, "ConTB");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(&example_abc_process, &example_trickle_process, &serial_in_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  struct {
    clock_time_t timestamp;
    linkaddr_t src;
    uint16_t rssi;
  } msg;
  static uint16_t counter;
  static uint8_t * packet;
  static uint8_t seqno;

  memset(&msg, 0, sizeof(msg));
  linkaddr_copy(&msg.src, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  msg.rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  msg.timestamp = clock_seconds();

  printf("%d.%d: found sensor %d.%d (%d) - %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      msg.src.u8[0], msg.src.u8[1], msg.rssi, msg.timestamp);

  seqno = (uint8_t)packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  packet = (uint8_t *)packetbuf_dataptr();
  counter = (packet[1] << 8) + packet[0];

  //printf("%d, %d, %d\n", linkaddr_node_addr.u8[0], msg.src.u8[0], seqno);
  //printf("abc message received '%s'\n", (char *)packetbuf_dataptr());
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{
  struct {
    clock_time_t timestamp;
    linkaddr_t src;
    uint16_t rssi;
  } msg;

  memset(&msg, 0, sizeof(msg));
  linkaddr_copy(&msg.src, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  msg.rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  msg.timestamp = clock_seconds();

  printf("%d.%d: found neighbor %d.%d (%d) - %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      msg.src.u8[0], msg.src.u8[1], msg.rssi, msg.timestamp);

  leds_toggle(LEDS_RED);
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static uint8_t message[64];
  static uint8_t loop;
  static uint16_t counter = 0;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {

    /* Delay 10-11 seconds */
    etimer_set(&et, CLOCK_SECOND * 5 + random_rand() % CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    memset(message, 0, 2);
    counter++;
    message[0] = counter & 0xFF;
    message[1] = (counter & 0xFF00) >> 8;

    loop = 3;
    while(loop--) {
      /* Delay 1-2 seconds */
      etimer_set(&et, CLOCK_SECOND + random_rand() % CLOCK_SECOND);
      packetbuf_copyfrom(message, 2);
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, (uint8_t)counter);
      abc_send(&abc);
      printf("abc message sent %d\n", (uint8_t)packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_trickle_process, ev, data)
{
  static struct etimer et;
  static uint8_t message[64];

  PROCESS_EXITHANDLER(trickle_close(&trickle);)
  PROCESS_BEGIN();

  memset(message, 0, 64);

  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);
  //SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Delay 25-26 seconds */
    etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    //PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event &&
		//	     data == &button_sensor);

    packetbuf_copyfrom("Who has this S/N", 17);
    trickle_send(&trickle);

    /* Delay 5 seconds */
    etimer_set(&et, CLOCK_SECOND * 5);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
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
void invoke_process_before_sleep(void)
{

}
