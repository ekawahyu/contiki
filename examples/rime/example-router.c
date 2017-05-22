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

#include "dev/button-sensor.h"
#include "dev/serial-line.h"

#include "dev/leds.h"

#include <stdio.h>

static uint16_t rank = 255;
static linkaddr_t forward_addr = {1, 0};
static linkaddr_t esender_addr = {1, 0};
/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ConBurst");
PROCESS(example_trickle_process, "ConTB");
PROCESS(example_multihop_process, "ConMHop");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &example_abc_process,
    &example_trickle_process,
    &example_multihop_process,
    &serial_in_process);
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
  static uint16_t counter;
  static uint8_t * packet;
  static uint8_t seqno;

  memset(&msg, 0, sizeof(msg));
  linkaddr_copy(&msg.src, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  linkaddr_copy(&forward_addr, &msg.src);
  linkaddr_copy(&esender_addr, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  msg.rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  msg.timestamp = clock_seconds();

  seqno = packetbuf_attr(PACKETBUF_ATTR_EPACKET_ID);
  packet = (uint8_t *)packetbuf_dataptr();
  counter = (packet[1] << 8) + packet[0];

  printf("%d.%d: found neighbor %d.%d (%d) - %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      msg.src.u8[0], msg.src.u8[1],
      packetbuf_attr(PACKETBUF_ATTR_EPACKET_TYPE),
      msg.timestamp);

  rank = trickle_rank(c);
  printf("%d\n", rank);

  if (packet[0] == linkaddr_node_addr.u8[0] && packet[1] == linkaddr_node_addr.u8[1])
    process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, NULL);

  leds_toggle(LEDS_RED);
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
/*
 * This function is called at the final recepient of the message.
 */
static void
multihop_recv(struct multihop_conn *c, const linkaddr_t *sender,
     const linkaddr_t *prevhop,
     uint8_t hops)
{
  printf("%d.%d: multihop message from %d.%d - %d hops\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        packetbuf_addr(PACKETBUF_ADDR_ESENDER)->u8[0],
        packetbuf_addr(PACKETBUF_ADDR_ESENDER)->u8[1],
        packetbuf_attr(PACKETBUF_ATTR_HOPS));
}
/*
 * This function is called to forward a packet. The function picks a
 * random neighbor from the neighbor list and returns its address. The
 * multihop layer sends the packet to this address. If no neighbor is
 * found, the function returns NULL to signal to the multihop layer
 * that the packet should be dropped.
 */
static linkaddr_t *
multihop_forward(struct multihop_conn *c,
  const linkaddr_t *originator, const linkaddr_t *dest,
  const linkaddr_t *prevhop, uint8_t hops)
{
  return &forward_addr;
}
static const struct multihop_callbacks multihop_call = {multihop_recv, multihop_forward};
static struct multihop_conn multihop;
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
    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_trickle_process, ev, data)
{
  static uint8_t message[64];
  static uint16_t counter = 0;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)
  PROCESS_BEGIN();

  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);
  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event &&
			     data == &button_sensor);

    memset(message, 0, 2);
    counter++;
    message[0] = 49;
    message[1] = 0;
    packetbuf_copyfrom(message, 2);

    trickle_set_rank(1);
    rank = trickle_rank(&trickle);
    printf("%d\n", rank);
    trickle_send(&trickle);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(multihop_close(&multihop);)

  PROCESS_BEGIN();

  /* Open a multihop connection on Rime channel CHANNEL. */
  multihop_open(&multihop, 135, &multihop_call);

  /* Loop forever, send a packet when the button is pressed. */
  while(1) {
    linkaddr_t to;

    /* Wait until we get trickle message */
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);

    /* Delay 1 seconds */
    etimer_set(&et, CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Copy the "Hello" to the packet buffer. */
    packetbuf_copyfrom("Hello", 6);

    /* Set the Rime address of the final receiver of the packet to
       1.0. This is a value that happens to work nicely in a Cooja
       simulation (because the default simulation setup creates one
       node with address 1.0). */
    to.u8[0] = esender_addr.u8[0];
    to.u8[1] = esender_addr.u8[1];

    /* Send the packet. */
    multihop_send(&multihop, &to);
    printf("%d.%d: multihop sent to %d.%d - %d\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());

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
