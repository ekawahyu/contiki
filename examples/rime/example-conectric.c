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

#ifndef CONECTRIC_VERSION_STRING
#define CONECTRIC_VERSION_STRING "Contiki-unknown"
#endif
#ifndef CONECTRIC_PROJECT_STRING
#define CONECTRIC_PROJECT_STRING "unknown"
#endif

/* definition of command line request */
#define CONECTRIC_REQUEST_HEADER_LEN    4

/* definition of network message request */
#define CONECTRIC_MESSAGE_HEADER_LEN    6
#define CONECTRIC_MESSAGE_LEN           2

/* Conectric Network Message Types */
/*   Commissioning */
#define CONECTRIC_ROUTE_REQUEST                 0x01
#define CONECTRIC_ROUTE_REQUEST_BY_SN           0x02
#define CONECTRIC_ROUTE_REPLY                   0x03
#define CONECTRIC_IMG_UPDATE_BCST               0x10
#define CONECTRIC_IMG_UPDATE_DIR                0x11
#define CONECTRIC_IMG_ACK                       0x12
#define CONECTRIC_IMG_COMPLETE                  0x13
#define CONECTRIC_MULTIHOP_PING                 0x14
#define CONECTRIC_MULTIHOP_PING_REPLY           0x15
#define CONECTRIC_REBOOT_REQUEST                0x16
#define CONECTRIC_REBOOT_REPLY                  0x17
#define CONECTRIC_SET_LONG_MAC                  0x18
#define CONECTRIC_SET_LONG_MAC_REPLY            0x19
#define CONECTRIC_GET_LONG_MAC                  0x1A
#define CONECTRIC_GET_LONG_MAC_REPLY            0x1B

/*   Reporting */
#define CONECTRIC_SENSOR_BROADCAST_RHT          0x30
#define CONECTRIC_SENSOR_BROADCAST_SW           0x31
#define CONECTRIC_SENSOR_BROADCAST_OC           0x32
#define CONECTRIC_SUPERVISORY_REPORT            0x33
#define CONECTRIC_SENSOR_UPDATE                 0x34
#define CONECTRIC_SET_STATE                     0x35
#define CONECTRIC_POLL_RS485                    0x36
#define CONECTRIC_POLL_RS485_REPLY              0x37
#define CONECTRIC_POLL_RS485_CHUNK              0x38
#define CONECTRIC_POLL_RS485_CHUNK_REPLY        0x39
#define CONECTRIC_POLL_WI                       0x3A
#define CONECTRIC_POLL_WI_REPLY                 0x3B
#define CONECTRIC_POLL_SENSORS                  0x3C
#define CONECTRIC_POLL_SENSORS_REPLY            0x3D
#define CONECTRIC_POLL_NEIGHBORS                0x3E
#define CONECTRIC_POLL_NEIGHBORS_REPLY          0x3F
#define CONECTRIC_SENSOR_BROADCAST_PLS          0x40
#define CONECTRIC_SENSOR_BROADCAST_USB          0x41

/*   Reporting boot status */
#define CONECTRIC_DEVICE_BROADCAST_BOOT_STATUS  0x60


/* Any message with MSB bit set to 1 needs immediate attention */

/*   Reporting low battery */
#define CONECTRIC_DEVICE_BROADCAST_LOW_BATTERY  0x80


/* Message buffer for incoming and outgoing sensor broadcasts */
#define CONECTRIC_MESSAGE_LENGTH                40


/* Conectric Message Structure */
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
  uint8_t       seqno;
  uint8_t       hops;
  uint8_t       maxhops;
  uint16_t      rssi;
} message_recv;

#define RUN_ON_COOJA_SIMULATION   1

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BUFFER_PAYLOAD        0
#define BUFFER_ALL            1

#if RUN_ON_COOJA_SIMULATION
volatile unsigned char gmacp_sim[8] = {0};
volatile unsigned char *gmacp = gmacp_sim;
#else
#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *gmacp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *gmacp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *gmacp = (__code unsigned char *)0xFFE8;
#endif
#endif

#define MESSAGE_CONECTRIC_RECV      7

static message_recv conectric_message_recv;

static uint8_t dump_header = 0;

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
void
hexstring_to_bytereq(uint8_t * hexstring, uint8_t * bytereq)
{
  uint8_t counter;
  uint8_t hex_string[2];
  int i;

  counter = 0;
  hexstring--;

  /* do conversion from hex string to hex bytes */
  while(*++hexstring != '\0') {

    /* remove space */
    if (*hexstring == ' ') continue;

    /* single digit hex string 0-9, A-F, a-f adjustment */
    if (*hexstring >= 0x30 && *hexstring <= 0x39)
      *hexstring -= 0x30;
    else if (*hexstring >= 0x41 && *hexstring <= 0x46)
      *hexstring -= 0x37;
    else if (*hexstring >= 0x61 && *hexstring <= 0x66)
              *hexstring -= 0x57;
    else /* skip all input other than hex number */
      continue;

    hex_string[counter % 2] = *hexstring;

    /* combinining two digits hex bytes into one and store it */
    if (counter++ % 2)
      bytereq[(counter >> 1)-1] = (hex_string[0] << 4) + hex_string[1];
  }
}
/*---------------------------------------------------------------------------*/
uint8_t *
command_respond(uint8_t * bytereq)
{
  static uint8_t hexstring[128];
  uint8_t request;
  int8_t i;

  if (bytereq == NULL) return 0;

  /*******************************************************/
  /***** INTERPRETING REQUEST BYTES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * [<][Len][Req][DestH][DestL][RLen][R1H][R1L]...[RnH][RnL][Data0][Data1]...[Datan]
   *
   * [Len]   = request byte length including [Len], but excluding [<]
   * [Req]   = request type
   * [DestH] = destination address H (can be one or more hops with/without routing table)
   * [DestL] = destination address L (can be one or more hops with/without routing table)
   * [RLen]  = routing table length including [RLen] itself
   * [RnH]   = the last hop address H ---> [DestH]
   * [RnL]   = the last hop address L ---> [DestL]
   * [Data0] = data sequence starts
   * [Datan] = the last data sequence
   *
   */
  if (bytereq[0] == '<') {

    request = bytereq[2];

    /* List of commands a device must respond to */
    if (request == CONECTRIC_ROUTE_REQUEST ||
        request == CONECTRIC_ROUTE_REQUEST_BY_SN ||
        request == CONECTRIC_MULTIHOP_PING ||
        request == CONECTRIC_REBOOT_REQUEST ||
        request == CONECTRIC_POLL_RS485  ||
        request == CONECTRIC_POLL_RS485_CHUNK  ||
        request == CONECTRIC_POLL_SENSORS  ||
        request == CONECTRIC_GET_LONG_MAC) {
      /* return the pointer of the request bytes */
      return bytereq;
    }

    /* Unknown request */
    else {
      putstring(": Unknown request - 0x");
      puthex(request);
      putstring("\n");
    }

  }

  /*******************************************************/
  /***** INTERPRETING COMMAND LINES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * - It starts with any char, but '<'
   * - It tests only capitalized letters
   *
   */
  else {

    if (bytereq[0] == 'M' && bytereq[1] == 'R') { /* MR = Mac address Read */
#if RUN_ON_COOJA_SIMULATION
      gmacp[0] = linkaddr_node_addr.u8[0];
      gmacp[1] = linkaddr_node_addr.u8[1];
      gmacp = gmacp_sim;
#else
      gmacp = &X_IEEE_ADDR;
#endif
      putstring("MR:");
      for(i = 7; i >= 0; i--) puthex(gmacp[i]);
      putstring("\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'P') { /* DP = Dump Payload only */
      dump_packet_buffer(BUFFER_PAYLOAD);
      putstring("DP:Ok\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'B') { /* DB = Dump everything from Buffer */
      dump_packet_buffer(BUFFER_ALL);
      putstring("DB:Ok\n");
    }

    else if (bytereq[0] == 'V' && bytereq[1] == 'E' && bytereq[2] == 'R') {
      putstring("VER:");
      putstring(CONTIKI_VERSION_STRING "\n");
      putstring("VER:");
      putstring(CONECTRIC_PROJECT_STRING "\n");
    }

    else if (bytereq[0] == '@' && bytereq[1] == 'B' && bytereq[2] == 'O' && bytereq[3] == 'O' && bytereq[4] == 'T') {
      putstring("@BOOT:Rebooting...\n");
      while(1);
    }

    /***** SEND PACKET *****/
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'S') {
      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "0714010001");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);
      return bytereq;
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == '0') {
      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "0714FFFF01");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);
      return bytereq;
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == '1') {
      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "0714280001");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);
      return bytereq;
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == '2') {
      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "0714180001");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);
      return bytereq;
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == '3') {
      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "07140C0001");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);
      return bytereq;
    }

    /***** SHOW ROUTING TABLE ****/
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == 'T') {
      for (i=0; i < route_num(); i++) {
        putstring("RT:");
        putdec(i);
        putstring(":");
        putdec(route_get(i)->dest.u8[0]);
        putstring(".");
        putdec(route_get(i)->dest.u8[1]);
        putstring("->");
        putdec(route_get(i)->nexthop.u8[0]);
        putstring(".");
        putdec(route_get(i)->nexthop.u8[1]);
        putstring("(C:");
        putdec(route_get(i)->cost);
        putstring(":LT:");
        putdec(route_get(i)->time);
        putstring(")\n");
      }
    }

    /***** REMOVE ROUTING TABLE ****/
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '0') {
      putstring("RT:");
      putdec(route_get(0)->dest.u8[0]);
      putstring(".");
      putdec(route_get(0)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(0));
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '1') {
      putstring("RT:");
      putdec(route_get(1)->dest.u8[0]);
      putstring(".");
      putdec(route_get(1)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(1));
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '2') {
      putstring("RT:");
      putdec(route_get(2)->dest.u8[0]);
      putstring(".");
      putdec(route_get(2)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(2));
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '3') {
      putstring("RT:");
      putdec(route_get(3)->dest.u8[0]);
      putstring(".");
      putdec(route_get(3)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(3));
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '4') {
      putstring("RT:");
      putdec(route_get(4)->dest.u8[0]);
      putstring(".");
      putdec(route_get(4)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(4));
    }
    /* temporary command to test routing table reliability */
    else if (bytereq[0] == 'R' && bytereq[1] == '5') {
      putstring("RT:");
      putdec(route_get(5)->dest.u8[0]);
      putstring(".");
      putdec(route_get(5)->dest.u8[1]);
      putstring("->");
      putdec(route_get(0)->nexthop.u8[0]);
      putstring(".");
      putdec(route_get(0)->nexthop.u8[1]);
      putstring(" removed\n");
      route_remove(route_get(5));
    }


    /* Unknown command */
    else {
      if (strlen((const char*)bytereq) != 0) {
        putstring((char*)bytereq);
        putstring(":Bad command!\n");
      }
    }

  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
uint8_t *
command_interpreter(uint8_t * command_line)
{
  static uint8_t bytereq[128];
  uint8_t * request;
  uint8_t counter;

  request = command_line;
  memset(bytereq, 0, sizeof(bytereq));

  if (request[0] == '<') {

    bytereq[0] = '<';
    hexstring_to_bytereq(&request[1], &bytereq[1]);
    return command_respond(bytereq);

  }
  else {

    counter = 0;

    /* pass through until end of line found */
    while(*request != '\0') {
      /* remove space */
      if (*request == ' ') {
        request++;
        continue;
      }
      /* capitalize letter */
      if (*request >= 0x61 && *request <= 0x7A)
        *request -= 0x20;
      bytereq[counter++] = *request++;
    }

    return command_respond(bytereq);

  }
}
/*---------------------------------------------------------------------------*/
void
compose_request_to_packetbuf(uint8_t * request, uint8_t seqno, linkaddr_t * ereceiver)
{
  static uint8_t packet_buffer[128];
  uint8_t * packet = packet_buffer;
  uint8_t * header = NULL;
  uint8_t * route = NULL;
  linkaddr_t dest;
  uint8_t req;
  uint8_t reqlen;
  uint8_t datalen;
  uint8_t routelen;
  uint8_t i;

  /*****************************************************/
  /***** NETWORK MESSAGE REQUEST/RESPONSE PROTOCOL *****/
  /*****************************************************/
  /*
   * [HdrLen][Seq][HopCnt][MaxHop][DestH][DestL][R1H][R1L]...[RnH][RnL][DLen][Data0][Data1]...[Datan]
   *
   * [HdrLen] = header + routing table length including the length byte itself
   * [Seq]    = sequence number
   * [HopCnt] = hop count
   * [MaxHop] = maximum hops before it gets dropped
   * [DestH]  = destination address H
   * [DestL]  = destination address L
   * [R1H]    = the first hop address H
   * [R1L]    = the first hop address L
   * [RnH]    = the last hop address H ---> [DestH]
   * [RnL]    = the last hop address L ---> [DestL]
   * [DLen]   = payload length
   * [Data0]  = data sequence starts
   * [Datan]  = the last data sequence
   *
   */

  if (*request == '<') request++; /* skip the '<' */

  reqlen     = *request++;
  req        = *request++;
  dest.u8[0] = *request++;
  dest.u8[1] = *request++;
  routelen   = *request++;

  if (ereceiver) linkaddr_copy(ereceiver, &dest);

  /* Filling in packetbuf with request byte and data.
   * Minimum length of data = 2 ---> [DLen][Req], the rest of data will
   * follow if there is any.
   */
  memset(packet_buffer, 0, sizeof(packet_buffer));
  datalen = reqlen - routelen - CONECTRIC_REQUEST_HEADER_LEN;
  *packet++ = datalen + CONECTRIC_MESSAGE_LEN;
  *packet++ = req;
  route = request;
  request += (routelen - 1);
  i = datalen;
  while (i--) *packet++ = *request++;

  packetbuf_copyfrom(packet_buffer, datalen + CONECTRIC_MESSAGE_LEN);

  routelen--; /* get rid of the length byte */

  packetbuf_hdralloc(CONECTRIC_MESSAGE_HEADER_LEN + routelen); /* allocate some space for header */

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = CONECTRIC_MESSAGE_HEADER_LEN + routelen;   /* header length */
  *header++ = seqno;          /* seqno */
  *header++ = 0;              /* hop count */
  *header++ = 0;              /* number of hops */
  *header++ = dest.u8[0];     /* destination addr H */
  *header++ = dest.u8[1];     /* destination addr L */
  while(routelen--)
        *header++ = *route++; /* routing table */

  /* The packetbuf is filled and ready to be sent */
}
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
