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
#include "debug.h"
#include "net/rime/rime.h"
#include "random.h"

#include "dev/serial-line.h"
#include "dev/leds.h"

#include <stdio.h>

#define DEBUG 0

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PUTSTRING(...) putstring(__VA_ARGS__)
#define PUTHEX(...) puthex(__VA_ARGS__)
#define PUTBIN(...) putbin(__VA_ARGS__)
#define PUTDEC(...) putdec(__VA_ARGS__)
#define PUTCHAR(...) putchar(__VA_ARGS__)
#else
#define PRINTF(...)
#define PUTSTRING(...)
#define PUTHEX(...)
#define PUTBIN(...)
#define PUTDEC(...)
#define PUTCHAR(...)
#endif

enum {
  CONECTRIC_ATTR_NONE,
  CONECTRIC_SENSOR_BROADCAST,
  CONECTRIC_ROUTE_REQUEST,
  CONECTRIC_ROUTE_REQUEST_BY_SN,
  CONECTRIC_ROUTE_REPLY,
  CONECTRIC_TIME_SYNC,
  CONECTRIC_POLL_SENSORS,
  CONECTRIC_POLL_SENSORS_REPLY,
  CONECTRIC_POLL_NEIGHBORS,
  CONECTRIC_POLL_NEIGHBORS_REPLY,
  CONECTRIC_MULTIHOP_PING,
  CONECTRIC_MULTIHOP_PING_REPLY,
  CONECTRIC_ATTR_MAX
};

static uint16_t rank = 255;
static uint8_t sensors[128] = {
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0x33, 0x44, 0x55
};
static uint8_t *sensors_head, *sensors_tail;

typedef struct {
  clock_time_t  timestamp;
  uint8_t       message_type;
  uint8_t       message[128];
  linkaddr_t    sender;
  linkaddr_t    prev_sender;
  linkaddr_t    receiver;
  linkaddr_t    prev_receiver;
  linkaddr_t    esender;
  linkaddr_t    prev_esender;
  linkaddr_t    ereceiver;
  linkaddr_t    prev_ereceiver;
  uint8_t       *payload;
  uint8_t       length;
  uint8_t       request;
  uint8_t       hops;
  uint16_t      rssi;
} message_recv;

#define MESSAGE_ABC_RECV      1
#define MESSAGE_TRICKLE_RECV  2
#define MESSAGE_MHOP_RECV     3
#define MESSAGE_MHOP_FWD      4

static message_recv abc_message_recv;
static message_recv trickle_message_recv;
static message_recv mhop_message_recv;

/*****************************************************************/
/* [REQLEN][REQ][DESTH][DESTL] [DATAN][DATAN-1]...[DATA1][DATA0] */
/*****************************************************************/
#define REQUEST_HEADER_LEN    4

/* for cooja serial number testing purpose */
static uint8_t serial_number[12] = {
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30
};
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

  /* Copy packetbuf to message */
  packetlen = packetbuf_copyto(message->message);

  /* Getting payload and its length */
  hdrlen = message->message[0];
  message->payload = &message->message[0] + hdrlen;
  message->length = message->message[hdrlen];

  dataptr = message->payload;
  message->request = *++dataptr;

  /* Decoding trickle ereceiver address */
  if (message_type == MESSAGE_TRICKLE_RECV) {
    dataptr--;
    message->ereceiver.u8[0] = *--dataptr;
    message->ereceiver.u8[1] = *--dataptr;
  }

  return packetlen;
}
/*---------------------------------------------------------------------------*/
static void
dump_packetbuf(void)
{
  static uint16_t len;
  static char * packetbuf;

  putstring(">");

  len = packetbuf_hdrlen();
  packetbuf = (char *)packetbuf_hdrptr();
  while(len--) puthex(*packetbuf++);

  len = packetbuf_datalen();
  packetbuf = (char *)packetbuf_dataptr();
  while(len--) puthex(*packetbuf++);

  putstring("\n");
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
  packetbuf_and_attr_copyto(&abc_message_recv, MESSAGE_ABC_RECV);
  dump_packetbuf();

  PRINTF("%d.%d: found sensor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      abc_message_recv.sender.u8[0], abc_message_recv.sender.u8[1],
      abc_message_recv.rssi, abc_message_recv.timestamp);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{
  uint8_t * payload;

  packetbuf_and_attr_copyto(&trickle_message_recv, MESSAGE_TRICKLE_RECV);
  payload = trickle_message_recv.payload;
  rank = trickle_rank(c);

  PRINTF("%d.%d: found neighbor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      trickle_message_recv.sender.u8[0], trickle_message_recv.sender.u8[1],
      trickle_message_recv.rssi, trickle_message_recv.timestamp);

  if (trickle_message_recv.request == CONECTRIC_ROUTE_REQUEST)
    if (linkaddr_cmp(&trickle_message_recv.ereceiver, &linkaddr_node_addr) == 0)
      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, payload);

  if (trickle_message_recv.request == CONECTRIC_ROUTE_REQUEST_BY_SN)
    if (trickle_message_recv.ereceiver.u8[1] == 0xFF && trickle_message_recv.ereceiver.u8[0] == 0xFF)
      if (*(payload+12) == serial_number[10] && *(payload+13) == serial_number[11])
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, payload);
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
  uint8_t * payload;

  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);
  payload = mhop_message_recv.payload;
  dump_packetbuf();

  PRINTF("%d.%d: multihop message from %d.%d - (%d hops) - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        mhop_message_recv.esender.u8[0], mhop_message_recv.esender.u8[1],
        mhop_message_recv.hops, clock_seconds());

  if (mhop_message_recv.request == CONECTRIC_MULTIHOP_PING)
    if (linkaddr_cmp(&mhop_message_recv.ereceiver, &linkaddr_node_addr) == 0)
      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, payload);

  if (mhop_message_recv.request == CONECTRIC_POLL_SENSORS)
    if (linkaddr_cmp(&mhop_message_recv.ereceiver, &linkaddr_node_addr) == 0)
      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, payload);
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
  static linkaddr_t forward_addr;
  static linkaddr_t prevhop_addr; /* only used for PRINTF */
  uint8_t mhops, hdrlen;
  uint8_t * header;
  int i;

  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);
  mhops = mhop_message_recv.hops;
  hdrlen = mhop_message_recv.message[0];

  /* If I am the originator, prevhop_addr == self linkaddr */
  if (prevhop == NULL)
    linkaddr_copy(&prevhop_addr, originator);
  else
    linkaddr_copy(&prevhop_addr, prevhop);

  PRINTF("%d.%d: multihop to forward from %d.%d - len=%d - %d hops\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        prevhop_addr.u8[0], prevhop_addr.u8[1], packetbuf_datalen(), mhops);

  /* Discard current packet header */
  packetbuf_copyfrom(mhop_message_recv.payload, mhop_message_recv.length);

  packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
  packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.ereceiver);
  packetbuf_set_attr(PACKETBUF_ATTR_HOPS, mhops);

  /* was serial request for multihop send with source routing */
  if (mhop_message_recv.request == CONECTRIC_MULTIHOP_PING) {
    /* Add new packet header */
    packetbuf_hdralloc(hdrlen);
    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = hdrlen;
    *header++ = mhop_message_recv.message[1];
    *header++ = mhops; /******** increment by one always */
    *header++ = mhop_message_recv.message[3]; /* FIXME this has to be the max hop */
    *header++ = mhop_message_recv.message[4];
    *header++ = mhop_message_recv.message[5];
    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }
    forward_addr.u8[1] = mhop_message_recv.message[4 + (mhops << 1)];
    forward_addr.u8[0] = mhop_message_recv.message[5 + (mhops << 1)];
  }

  /* was serial request for multihop send with source routing */
  if (mhop_message_recv.request == CONECTRIC_POLL_SENSORS) {
    /* Add new packet header */
    packetbuf_hdralloc(hdrlen);
    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = hdrlen;
    *header++ = mhop_message_recv.message[1];
    *header++ = mhops; /******** increment by one always */
    *header++ = mhop_message_recv.message[3]; /* FIXME this has to be the max hop */
    *header++ = mhop_message_recv.message[4];
    *header++ = mhop_message_recv.message[5];
    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }
    forward_addr.u8[1] = mhop_message_recv.message[4 + (mhops << 1)];
    forward_addr.u8[0] = mhop_message_recv.message[5 + (mhops << 1)];
  }

  /* was multihop radio request with source routing */
  if (mhop_message_recv.request == CONECTRIC_MULTIHOP_PING_REPLY) {
    /* Add new packet header */
    packetbuf_hdralloc(hdrlen);
    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = hdrlen;
    *header++ = mhop_message_recv.message[1];
    *header++ = mhops; /******** increment by one always */
    *header++ = mhop_message_recv.message[3]; /* FIXME this has to be the max hop */
    *header++ = mhop_message_recv.message[4];
    *header++ = mhop_message_recv.message[5];
    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }
    linkaddr_copy(&forward_addr, &mhop_message_recv.prev_sender);
    packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
    packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.prev_esender);
  }

  /* was multihop radio request with source routing */
  if (mhop_message_recv.request == CONECTRIC_POLL_SENSORS_REPLY) {
    /* Add new packet header */
    packetbuf_hdralloc(hdrlen);
    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = hdrlen;
    *header++ = mhop_message_recv.message[1];
    *header++ = mhops; /******** increment by one always */
    *header++ = mhop_message_recv.message[3]; /* FIXME this has to be the max hop */
    *header++ = mhop_message_recv.message[4];
    *header++ = mhop_message_recv.message[5];
    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }
    linkaddr_copy(&forward_addr, &mhop_message_recv.prev_sender);
    packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
    packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.prev_esender);
  }

  /* was trickle radio request, no source routing */
  if (mhop_message_recv.request == CONECTRIC_ROUTE_REPLY) {
    /* Add new packet header */
    packetbuf_hdralloc(hdrlen + 2);
    header = (uint8_t *)packetbuf_hdrptr();
    *header++ = hdrlen + 2;
    *header++ = mhop_message_recv.message[1];
    *header++ = mhop_message_recv.message[2] + 1; /******** increment by one always */
    *header++ = mhop_message_recv.message[3] + 1; /* FIXME this has to be the max hop */
    *header++ = mhop_message_recv.message[4];
    *header++ = mhop_message_recv.message[5];
    *header++ = linkaddr_node_addr.u8[1];
    *header++ = linkaddr_node_addr.u8[0];
    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }
    linkaddr_copy(&forward_addr, &trickle_message_recv.sender);
  }

  PRINTF("%d.%d: forwarding address is %d.%d - %d hops\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      forward_addr.u8[0], forward_addr.u8[1], mhops);

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
static void
compose_request_to_packetbuf(uint8_t * request,
    uint8_t seqno, linkaddr_t * ereceiver)
{
  static uint8_t packet_buffer[128];
  uint8_t * packet = packet_buffer;
  uint8_t * header = NULL;
  linkaddr_t dest;
  uint8_t req;
  uint8_t reqlen;
  uint8_t datalen;
  uint8_t routinglen;
  uint8_t i;

  request++; /* request from serial port, skip the '<' */

  reqlen     = *request++;
  req        = *request++;
  dest.u8[1] = *request++;
  dest.u8[0] = *request++;

  if (ereceiver) linkaddr_copy(ereceiver, &dest);

  if (req == CONECTRIC_MULTIHOP_PING ||
      req == CONECTRIC_POLL_SENSORS) {
    datalen = 0;
    routinglen = reqlen - REQUEST_HEADER_LEN;
  }
  else {
    routinglen = 0;
    datalen = reqlen - REQUEST_HEADER_LEN;
  }

  memset(packet_buffer, 0, sizeof(packet_buffer));
  *packet++ = datalen + 2;
  *packet++ = req;

  i = datalen;

  /* Composing payload */
//  if (req == CONECTRIC_ROUTE_REQUEST) {
//    /* do nothing, it's been populated above */
//  }
  if (req == CONECTRIC_ROUTE_REQUEST_BY_SN) {
    while (i--) *packet++ = *request++;
  }
//  if (req == CONECTRIC_MULTIHOP_PING) {
//    /* do nothing, it's been populated above */
//  }
//  if (req == CONECTRIC_POLL_SENSORS) {
//    /* do nothing, it's been populated above */
//  }

  packetbuf_copyfrom(packet_buffer, datalen+2);

  packetbuf_hdralloc(6 + routinglen);

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = 6 + routinglen;   /* header len */
  *header++ = seqno;            /* seqno */
  *header++ = 0;                /* hop count */
  *header++ = 0;                /* number of hops */
  *header++ = dest.u8[1];       /* destination addr H */
  *header++ = dest.u8[0];       /* destination addr L */
  while(routinglen--)
        *header++ = *request++; /* routing table */
}
/*---------------------------------------------------------------------------*/
static void
compose_response_to_packetbuf_from(uint8_t * radio_request,
    uint8_t seqno, linkaddr_t * ereceiver)
{
  static uint8_t packet_buffer[128];
  uint8_t * packet = packet_buffer;
  uint8_t * header = NULL;
  uint8_t req;
  uint8_t reqlen;
  uint8_t datalen;
  uint8_t routinglen;
  uint8_t i;
  uint8_t response;

  reqlen = *radio_request++;
  req    = *radio_request++;

  /* Composing payload */
  if (req == CONECTRIC_ROUTE_REQUEST) {
    response = CONECTRIC_ROUTE_REPLY;
    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
  }
  if (req == CONECTRIC_ROUTE_REQUEST_BY_SN) {
    response = CONECTRIC_ROUTE_REPLY;
    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
  }
  if (req == CONECTRIC_MULTIHOP_PING) {
    response = CONECTRIC_MULTIHOP_PING_REPLY;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
  if (req == CONECTRIC_POLL_SENSORS) {
    response = CONECTRIC_POLL_SENSORS_REPLY;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }

  memset(packet_buffer, 0, sizeof(packet_buffer));
  *packet++ = 2;
  *packet++ = response;

  packetbuf_copyfrom(packet_buffer, 2);

  packetbuf_hdralloc(6);

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = 6;   /* header len */
  *header++ = seqno;            /* seqno */
  *header++ = 0;                /* hop count */
  *header++ = 0;                /* number of hops */
  *header++ = ereceiver->u8[1]; /* destination addr H */
  *header++ = ereceiver->u8[0]; /* destination addr L */
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_trickle_process, ev, data)
{
  static uint8_t counter = 0;
  static uint8_t * request;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  /* Open a trickle connection */
  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    /* Compose packetbuf based on the received request */
    request = (uint8_t *)data;
    compose_request_to_packetbuf(request, counter++, NULL);

    /* Send the rank to 1 (source of trickle) */
    trickle_set_rank(1);

    /* Send the packet */
    trickle_send(&trickle);

#if DEBUG
    request++; request++;

    if (*request == CONECTRIC_ROUTE_REQUEST) {
      PRINTF("%d.%d: route request sent to %d.%d - %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          *(request+2), *(request+1), clock_seconds());
    }
    if (*request == CONECTRIC_ROUTE_REQUEST_BY_SN) {
      PRINTF("%d.%d: route request by S/N sent to %d.%d - %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          *(request+2), *(request+1), clock_seconds());
    }
#endif

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data)
{
  static struct etimer et;
  static uint8_t reply;
  static linkaddr_t to;
  static uint8_t counter = 0;
  static uint8_t * request;

  PROCESS_EXITHANDLER(multihop_close(&multihop);)

  PROCESS_BEGIN();

  /* Open a multihop connection */
  multihop_open(&multihop, 135, &multihop_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    /* Compose packetbuf based on the received request */
    request = (uint8_t *)data;
    /* Request comes from serial port */
    if (*request == '<')
      compose_request_to_packetbuf(request, counter++, &to);
    /* Request receives over the air */
    else
      compose_response_to_packetbuf_from(request, counter++, &to);

    /* TODO delay count (for now) 1 seconds for (local) trickle to subside */
    //etimer_set(&et, CLOCK_SECOND);
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send the packet */
    multihop_send(&multihop, &to);

    request++; request++;

    PRINTF("%d.%d: multihop send to %d.%d - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static uint8_t * request;
  static uint8_t counter;
  static uint8_t hex_string[2];
  static uint8_t bytereq[128];

  PROCESS_BEGIN();

  /* for cooja serial number testing purpose */
  serial_number[10] = ((linkaddr_node_addr.u8[0] & 0xF0) >> 4);
  if (serial_number[10] > 9)
    serial_number[10] += 0x37;
  else
    serial_number[10] += 0x30;

  serial_number[11] = (linkaddr_node_addr.u8[0] & 0x0F);
  if (serial_number[11] > 9)
    serial_number[11] += 0x37;
  else
    serial_number[11] += 0x30;

  printf("Meter S/N=%s\n", serial_number);
  /*******************************************/

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    printf("%s\n", (uint8_t *)data, strlen(data));
    request = (uint8_t *)data;

    counter = 2;

    memset(bytereq, 0, sizeof(bytereq));

    if (request[0] == '<') {

      bytereq[0] = '<';

      /* do conversion from hex string to hex bytes */
      while(*++request != '\0') {

        if (*request == ' ') continue;

        /* single digit hex string 0-9, A-F, a-f adjustment */
        if (*request >= 0x30 && *request <= 0x39)
          *request -= 0x30;
        else if (*request >= 0x41 && *request <= 0x46)
          *request -= 0x37;
        else if (*request >= 0x61 && *request <= 0x66)
                  *request -= 0x57;
        else /* skip all input other than hex number */
          continue;

        hex_string[counter % 2] = *request;

        /* combinining two digits hex bytes into one and store it */
        if (counter++ % 2)
          bytereq[(counter >> 1)-1] = (hex_string[0] << 4) + hex_string[1];
      }

      /* interpreting request bytes */
      if (bytereq[2] == CONECTRIC_ROUTE_REQUEST ||
          bytereq[2] == CONECTRIC_ROUTE_REQUEST_BY_SN)
        process_post(&example_trickle_process, PROCESS_EVENT_CONTINUE, bytereq);
      else if (
          bytereq[2] == CONECTRIC_MULTIHOP_PING ||
          bytereq[2] == CONECTRIC_POLL_SENSORS)
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, bytereq);
      else
        /* unknown request */
        PRINTF("%d.%d: Unknown request - %lu\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            clock_seconds());
    }
    else {
      /* commands for local execution */
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
