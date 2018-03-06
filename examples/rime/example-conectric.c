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
#include "debug.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"
#include "dev/serial-line.h"

#include <stdio.h>

#include "examples/conectric/conectric-messages.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MESSAGE_CONECTRIC_RECV      7

static message_recv conectric_message_recv;

static uint8_t dump_header = 0;
static uint8_t usb_collect_is_a_sink = 0;

/*---------------------------------------------------------------------------*/
static uint8_t
packetbuf_and_attr_copyto(message_recv * message, uint8_t message_type)
{
  uint8_t packetlen, hdrlen;
  uint8_t *dataptr;

  /* Backup the previous senders and receivers */
  linkaddr_copy(&message->prev_sender, &message->sender);
  linkaddr_copy(&message->prev_esender, &message->esender);
  linkaddr_copy(&message->prev_receiver, &message->receiver);
  linkaddr_copy(&message->prev_ereceiver, &message->ereceiver);

  /* Copy the packet attributes to avoid them being overwritten or
   * cleared by an application program that uses the packet buffer for
   * its own needs
   */
  memset(message, 0, sizeof(message));
  message->timestamp = clock_seconds();
  message->message_type = message_type;
  linkaddr_copy(&message->sender, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  linkaddr_copy(&message->esender, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  linkaddr_copy(&message->ereceiver, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
  message->rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  message->hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  /* Copy the packetbuf to avoid them being overwritten or
   * cleared by an application program that uses the packet buffer for
   * its own needs
   */
  packetlen = packetbuf_copyto(message->message);

  /* Decoding payload and its length */
  hdrlen = message->message[0];
  message->seqno = message->message[1];
  message->payload = &message->message[0] + hdrlen;
  message->length = message->message[hdrlen];

  /* Update hop count */
  message->message[2] = message->hops;

  /* Replace destination with originator address */
  if (message->esender.u8[1] || message->esender.u8[0]) {
    message->message[4] = message->esender.u8[1];
    message->message[5] = message->esender.u8[0];
  }
  else {
    message->message[4] = message->sender.u8[1];
    message->message[5] = message->sender.u8[0];
  }

  /* Decoding request byte */
  dataptr = message->payload;
  message->request = *++dataptr;

  return packetlen;
}
/*---------------------------------------------------------------------------*/
void
dump_packet_buffer(uint8_t mode)
{
  dump_header = mode;
}
/*---------------------------------------------------------------------------*/
static void
dump_packetbuf(message_recv * message)
{
  uint8_t len;
  static char * packetbuf;

  putstring(">");

  if (dump_header) {
    len = packetbuf_hdrlen();
    packetbuf = (char *)packetbuf_hdrptr();
    while(len--) puthex(*packetbuf++);
  }

  len = message->message[0] + message->length;
  packetbuf = message->message;
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}
/*---------------------------------------------------------------------------*/
static struct conectric_conn conectric;
/*---------------------------------------------------------------------------*/
PROCESS(example_conectric_process, "Conectric example");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(&example_conectric_process, &serial_in_process);
/*---------------------------------------------------------------------------*/
static void
sent(struct conectric_conn *c)
{
  PRINTF("packet sent\n");
}

static void
timedout(struct conectric_conn *c)
{
  PRINTF("packet timedout\n");
}

static void
recv(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  packetbuf_and_attr_copyto(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);

  dump_packetbuf(&conectric_message_recv);

  PRINTF("%d.%d: data received from %d.%d: %.*s (%d) - %d hops\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1],
      (char *)packetbuf_dataptr(), packetbuf_datalen(), hops);
}
static void
netbroadcast(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  packetbuf_and_attr_copyto(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);

  dump_packetbuf(&conectric_message_recv);

  PRINTF("%d.%d: broadcast received from %d.%d: %.*s (%d) - %d hops\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1],
      packetbuf_datalen(), (char *)packetbuf_dataptr(),
      packetbuf_datalen(), hops);
}

const static struct conectric_callbacks callbacks = {recv, sent, timedout, netbroadcast};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_conectric_process, ev, data)
{
  static uint8_t seqno = 0;
  static uint8_t * request;
  static linkaddr_t to;

  PROCESS_EXITHANDLER(conectric_close(&conectric);)

  PROCESS_BEGIN();

  conectric_open(&conectric, 132, &callbacks);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    request = (uint8_t*)data;
    compose_request_to_packetbuf(request, seqno++, &to);
    conectric_send(&conectric, &to);

    PRINTF("%d.%d: conectric sent to %d.%d - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static uint8_t * event;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    printf("%s\n", (uint8_t *)data);

    event = command_interpreter((uint8_t *)data);

    if (event) {
      process_post(&example_conectric_process, PROCESS_EVENT_CONTINUE, event);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
