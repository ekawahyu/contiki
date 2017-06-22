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

enum {
  CONECTRIC_ATTR_NONE,
  CONECTRIC_SENSOR_BROADCAST,
  CONECTRIC_ROUTE_REQUEST,
  CONECTRIC_ROUTE_REPLY,
  CONECTRIC_TIME_SYNC,
  CONECTRIC_POLL_SENSORS,
  CONECTRIC_POLL_SENSORS_REPLY,
  CONECTRIC_POLL_NEIGHBORS,
  CONECTRIC_POLL_NEIGHBORS_REPLY,
  CONECTRIC_ATTR_MAX
};

static uint16_t rank = 255;
static linkaddr_t forward_addr = {{1, 0}};
static linkaddr_t prevhop_addr = {{1, 0}};
static linkaddr_t esender_addr = {{1, 0}};
static uint8_t sensors[128];
/*---------------------------------------------------------------------------*/
static void
dump_packetbuf(void)
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

  memset(&msg, 0, sizeof(msg));
  linkaddr_copy(&msg.src, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  msg.rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  msg.timestamp = clock_seconds();

  dump_packetbuf();

  printf("%d.%d: found sensor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      msg.src.u8[0], msg.src.u8[1], msg.rssi, msg.timestamp);
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
  uint8_t message[128];
  uint8_t packetbuf_len, header_len;
  uint8_t request;

  /* Copy the packet attributes to avoid them being overwritten or
     cleared by an application program that uses the packet buffer for
     its own needs */
  memset(&msg, 0, sizeof(msg));
  linkaddr_copy(&msg.src, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  linkaddr_copy(&forward_addr, &msg.src);
  linkaddr_copy(&esender_addr, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  msg.rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  msg.timestamp = clock_seconds();

  /*printf("%d.%d: found neighbor %d.%d (%d) - %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      msg.src.u8[0], msg.src.u8[1],
      packetbuf_attr(PACKETBUF_ATTR_EPACKET_TYPE),
      msg.timestamp);*/

  rank = trickle_rank(c);

  /* Copy packetbuf to message */
  memset(message, 0, sizeof(message));
  packetbuf_len = packetbuf_copyto(message);
  packetbuf_len = packetbuf_len; /* suppressed warning */

  header_len = message[0];
  request = message[header_len];
  if (request == CONECTRIC_ROUTE_REQUEST) {
    if (message[header_len-1] == linkaddr_node_addr.u8[0] &&
        message[header_len-2] == linkaddr_node_addr.u8[1]) {
      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, &request);
      leds_toggle(LEDS_RED);
    }
  }
  if (message[header_len] == CONECTRIC_TIME_SYNC) {
    clock_set_seconds(
        ((uint32_t)message[header_len+1] << 24) +
        ((uint32_t)message[header_len+2] << 16) +
        ((uint32_t)message[header_len+3] << 8) +
        ((uint32_t)message[header_len+4]));
    leds_toggle(LEDS_RED);
  }
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
/*
 * This function is called at the final recepient of the multihop message.
 */
static void
multihop_recv(struct multihop_conn *c, const linkaddr_t *sender,
     const linkaddr_t *prevhop,
     uint8_t hops)
{
  linkaddr_t msender, mreceiver;
  uint8_t message[128];
  uint8_t packetbuf_len, header_len;
  uint8_t mhops;
  int i;

  /* Copy the packet attributes to avoid them being overwritten or
     cleared by an application program that uses the packet buffer for
     its own needs */
  linkaddr_copy(&msender, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  linkaddr_copy(&mreceiver, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
  linkaddr_copy(&prevhop_addr, prevhop);
  mhops = packetbuf_attr(PACKETBUF_ATTR_HOPS);
  mhops = mhops; /* suppressed warning */

  dump_packetbuf();

  printf("%d.%d: multihop message from %d.%d - (%d hops) - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        packetbuf_addr(PACKETBUF_ADDR_ESENDER)->u8[0],
        packetbuf_addr(PACKETBUF_ADDR_ESENDER)->u8[1],
        packetbuf_attr(PACKETBUF_ATTR_HOPS),
        clock_seconds());

  /* Copy packetbuf to message */
  memset(message, 0, sizeof(message));
  packetbuf_len = packetbuf_copyto(message);

  header_len = message[0];
  header_len = header_len; /* suppressed warning */
}
/*---------------------------------------------------------------------------*/
/*
 * This function is called to forward a packet. It returns a forwarding address
 * determined by trickle message ranking. If no neighbor is found, the function
 * returns NULL to signal to the multihop layer to drop the packet.
 */
static linkaddr_t *
multihop_forward(struct multihop_conn *c,
  const linkaddr_t *originator, const linkaddr_t *dest,
  const linkaddr_t *prevhop, uint8_t hops)
{
  linkaddr_t msender, mreceiver;
  uint8_t message[128];
  uint8_t mhops, packetbuf_len, header_len;
  uint8_t * header;
  int i;

  /* Copy the packet attributes to avoid them being overwritten or
     cleared by an application program that uses the packet buffer for
     its own needs */
  linkaddr_copy(&msender, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  linkaddr_copy(&mreceiver, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
  mhops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  /* If I am the originator, prevhop_addr == self linkaddr */
  if (prevhop == NULL)
    linkaddr_copy(&prevhop_addr, originator);
  else
    linkaddr_copy(&prevhop_addr, prevhop);

  /*printf("%d.%d: multihop to forward from %d.%d - len=%d - %d hops\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        prevhop_addr.u8[0], prevhop_addr.u8[1], packetbuf_datalen(), mhops);*/

  memset(message, 0, sizeof(message));
  packetbuf_len = packetbuf_copyto(message);

  header_len = message[0];
  packetbuf_copyfrom(&message[header_len], packetbuf_len - header_len);
  packetbuf_hdralloc(header_len + 2);
  header = (uint8_t *)packetbuf_hdrptr();

  *header++ = header_len + 2;
  *header++ = message[1];
  *header++ = message[2] + 1;
  *header++ = message[3] + 1;
  *header++ = message[4];
  *header++ = message[5];
  *header++ = linkaddr_node_addr.u8[1];
  *header++ = linkaddr_node_addr.u8[0];
  for (i = 6; i < header_len; i++) {
    *header++ = message[i];
  }

  packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &msender);
  packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mreceiver);
  packetbuf_set_attr(PACKETBUF_ATTR_HOPS, mhops);

  return &forward_addr;
}
static const struct multihop_callbacks multihop_call = {multihop_recv, multihop_forward};
static struct multihop_conn multihop;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
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
  static linkaddr_t to;
  static uint8_t message[128];
  static uint8_t * header;
  static uint8_t counter = 0;
  static uint8_t request;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  /* Open a trickle connection */
  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    request = *(uint8_t *)data;

    memset(message, 0, sizeof(message));
    message[0] = request;
    packetbuf_copyfrom(message, 1);

    /* Set the Rime address of the final receiver */
    esender_addr.u8[1] = 0;
    esender_addr.u8[0] = 49;
    to.u8[0] = esender_addr.u8[0];
    to.u8[1] = esender_addr.u8[1];

    packetbuf_hdralloc(6);

    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = 6;          /* header len */
    *header++ = counter++;  /* seqno */
    *header++ = 0;          /* hop count */
    *header++ = 0;          /* number of hops */
    *header++ = to.u8[1];   /* destination addr H */
    *header++ = to.u8[0];   /* destination addr L */

    /* Send the rank to 1 (source of trickle) */
    trickle_set_rank(1);

    /* Send the packet */
    trickle_send(&trickle);

    /* For debugging purposes */
    if (request == CONECTRIC_ROUTE_REQUEST) {
      printf("%d.%d: route request sent to %d.%d - %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          esender_addr.u8[0], esender_addr.u8[1],
          clock_seconds());
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data)
{
  static struct etimer et;

  static linkaddr_t to;
  static uint8_t message[128];
  static uint8_t * header;
  static uint8_t counter = 0;
  static uint8_t request;
  static uint8_t reply;

  PROCESS_EXITHANDLER(multihop_close(&multihop);)

  PROCESS_BEGIN();

  /* Open a multihop connection */
  multihop_open(&multihop, 135, &multihop_call);

  /* Loop forever */
  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    request = *(uint8_t *)data;

    if (request == CONECTRIC_ROUTE_REQUEST) {
      reply = CONECTRIC_ROUTE_REPLY;
      /* TODO delay count (for now) 1 seconds for trickle to subside */
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    memset(message, 0, sizeof(message));
    message[0] = reply;
    packetbuf_copyfrom(message, 1);

    /* Set the Rime address of the final receiver */
    to.u8[0] = esender_addr.u8[0];
    to.u8[1] = esender_addr.u8[1];

    packetbuf_hdralloc(6);

    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = 6;          /* header len */
    *header++ = counter++;  /* seqno */
    *header++ = 0;          /* hop count */
    *header++ = 0;          /* number of hops */
    *header++ = to.u8[1];   /* destination addr H */
    *header++ = to.u8[0];   /* destination addr L */

    /* Send the packet. */
    multihop_send(&multihop, &to);

    /* For debugging purposes */
    if (reply == CONECTRIC_ROUTE_REPLY) {
      printf("%d.%d: replying route request to %d.%d - %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          to.u8[0], to.u8[1], clock_seconds());
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static uint8_t * request = NULL;
  static uint8_t counter;
  static uint8_t hex_string[2];
  static uint8_t hex_num;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    printf("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    request = (uint8_t *)data;

    counter = 0;

    if (request[0] == '<') {
      printf("< ");
      while(*++request != '\0') {
        if (*request == ' ') continue;

        /* do conversion here 0-9-0x30 A-F-0x37 a-f-0x57 */
        if (*request >= 0x30 && *request <= 0x39)
          *request -= 0x30;
        else if (*request >= 0x41 && *request <= 0x46)
          *request -= 0x37;
        else if (*request >= 0x61 && *request <= 0x66)
                  *request -= 0x57;

        hex_string[counter%2] = *request;

        /* do conversion here and execute */
        if (counter++ % 2) {
          hex_num = (hex_string[0] << 4) + hex_string[1];
          if (hex_num == CONECTRIC_ROUTE_REQUEST)
            process_post(&example_trickle_process, PROCESS_EVENT_CONTINUE, &hex_num);
        }
      }
    }
    else {
      /* commands for local execution */
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
