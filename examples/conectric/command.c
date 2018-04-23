/*
 * command.c
 *
 * Created on: Feb 26, 2018
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2018, Conectric Network, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the copyright holder.
 *
 */

#include "contiki.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"

#include "command.h"
#include "sernum.h"
#if defined __IAR_SYSTEMS_ICC__
#include "conectric-version.h"
#endif

#ifdef CONTIKI_VERSION_STRING
#undef CONTIKI_VERSION_STRING
#define CONTIKI_VERSION_STRING CONECTRIC_VERSION_STRING
#endif

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

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

/* definition of command line request */
#define CONECTRIC_REQUEST_HEADER_LEN    4

/* definition of network message request */
#define CONECTRIC_MESSAGE_HEADER_LEN    6
#define CONECTRIC_MESSAGE_LEN           3

extern struct conectric_conn conectric;

/*---------------------------------------------------------------------------*/
void
hexstring_to_bytereq(uint8_t * hexstring, uint8_t * bytereq)
{
  uint8_t counter;
  uint8_t hex_string[2];

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
  uint8_t snlen;
  uint8_t sn[12];
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
        request == CONECTRIC_RS485_POLL  ||
        request == CONECTRIC_RS485_POLL_CHUNK  ||
        request == CONECTRIC_RS485_CONFIG  ||
        request == CONECTRIC_POLL_SENSORS  ||
        request == CONECTRIC_GET_LONG_MAC ||
        request == CONECTRIC_TEXT_MESSAGE) {
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

    else if (bytereq[0] == 'S' && bytereq[1] == 'S') {
      conectric_set_sink(&conectric, 60 * CLOCK_SECOND, 1);
      putstring("SS:Ok\n");
    }

    else if (bytereq[0] == 'S' && bytereq[1] == 'R') {
      conectric_set_sink(&conectric, 60 * CLOCK_SECOND, 0);
      putstring("SR:Ok\n");
    }

    else if (bytereq[0] == 'C' && bytereq[1] == 'S') {
      conectric_set_collect(&conectric, 1);
      putstring("SS:Ok\n");
    }

    else if (bytereq[0] == 'C' && bytereq[1] == 'R') {
      conectric_set_collect(&conectric, 0);
      putstring("SR:Ok\n");
    }

    else if (bytereq[0] == 'V' && bytereq[1] == 'E' && bytereq[2] == 'R') {
      putstring("VER:");
      putstring(CONTIKI_VERSION_STRING "\n");
      putstring("VER:");
      putstring(CONECTRIC_PROJECT_STRING "\n");
    }

    else if (bytereq[0] == 'S' && bytereq[1] == 'N' && bytereq[2] == 'R') {
      putstring("SNR:");
      snlen = sernum_read(sn);
      while (snlen--) puthex(sn[snlen]);
      putstring("\n");
    }

    else if (bytereq[0] == 'S' && bytereq[1] == 'N' && bytereq[2] == 'W') {
      if (strlen((const char*)bytereq) == 27) {
        memset(sn, 0, sizeof(sn));
        snlen = sizeof(sn);
        hexstring_to_bytereq(&bytereq[3], sn);
        putstring("SNW:");
        while(snlen) puthex(sn[sizeof(sn)-snlen--]);
        putstring("\n");
        if (sernum_write(sn))
          putstring("SNW:Ok\n");
        else
          putstring("SNW:Err:already assigned\n");
        putstring("\n");
      }
      else {
        putstring("SNW:Err:incorrect length\n");
      }
    }

    else if (bytereq[0] == '@' && bytereq[1] == 'B' && bytereq[2] == 'O' && bytereq[3] == 'O' && bytereq[4] == 'T') {
      putstring("@BOOT:Rebooting...\n");
      while(1);
    }

    else if (bytereq[0] == 'R' && bytereq[1] == 'T') {
      for (i=0; i < route_num(); i++) {
        putstring("RT:");
        putdec(i);
        putstring(":");
        puthex(route_get(i)->dest.u8[0]);
        putstring(".");
        puthex(route_get(i)->dest.u8[1]);
        putstring("->");
        puthex(route_get(i)->nexthop.u8[0]);
        putstring(".");
        puthex(route_get(i)->nexthop.u8[1]);
        putstring("(C:");
        putdec(route_get(i)->cost);
        putstring(":LT:");
        putdec(route_get(i)->time);
        putstring(")\n");
      }
    }

    else if (bytereq[0] == 'S' && bytereq[1] == 'T') {
      for (i=0; i < sink_num(); i++) {
        putstring("ST:");
        putdec(i);
        putstring(":");
        puthex(sink_get(i)->addr.u8[0]);
        putstring(".");
        puthex(sink_get(i)->addr.u8[1]);
        putstring("(C:");
        putdec(sink_get(i)->cost);
        putstring(":LT:");
        putdec(sink_get(i)->time);
        putstring(")\n");
      }
    }

    else if (bytereq[0] == 'R' && bytereq[1] == 'F') {
        putstring("RF:routing table flushed\n");
        route_flush_all();
    }

    /* Unknown command */
    else {
      if (strlen((const char*)bytereq) != 0) {
        putstring((char*)bytereq);
        putstring(":Bad command\n");
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
compose_request_to_packetbuf(uint8_t * request, uint8_t seqno, uint8_t batt, linkaddr_t * ereceiver)
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

  if (*request == '<') request++; /* skip the '<' */

  reqlen     = *request++;
  req        = *request++;
  dest.u8[0] = *request++;
  dest.u8[1] = *request++;
  routelen   = *request++;

  if (ereceiver) linkaddr_copy(ereceiver, &dest);

  /* Filling in packetbuf with request byte and data.
   * Minimum length of data = 3 ---> [DLen][Req][Batt], the rest of data will
   * follow if there is any.
   */
  memset(packet_buffer, 0, sizeof(packet_buffer));
  datalen = reqlen - routelen - CONECTRIC_REQUEST_HEADER_LEN;
  *packet++ = datalen + CONECTRIC_MESSAGE_LEN;
  *packet++ = req;
  *packet++ = batt;
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
void
compose_request_line_to_packetbuf(request_line * line, uint8_t seqno, uint8_t batt, linkaddr_t * ereceiver)
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

  reqlen     = line->reqlen;
  req        = line->req;
  dest.u8[0] = line->dest.u8[0];
  dest.u8[1] = line->dest.u8[1];
  routelen   = line->hdr_next;

  if (ereceiver) linkaddr_copy(ereceiver, &line->dest);

  /* Filling in packetbuf with request byte and data.
   * Minimum length of data = 3 ---> [DLen][Req][Batt], the rest of data will
   * follow if there is any.
   */
  memset(packet_buffer, 0, sizeof(packet_buffer));
  datalen = reqlen - routelen - CONECTRIC_REQUEST_HEADER_LEN;
  *packet++ = datalen + CONECTRIC_MESSAGE_LEN;
  *packet++ = req;
  *packet++ = batt;
  i = datalen;
  while (i--) *packet++ = *line->payload++;

  packetbuf_copyfrom(packet_buffer, datalen + CONECTRIC_MESSAGE_LEN);

  routelen--; /* get rid of the length byte */

  packetbuf_hdralloc(CONECTRIC_MESSAGE_HEADER_LEN + routelen); /* allocate some space for header */

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = CONECTRIC_MESSAGE_HEADER_LEN;   /* header length */
  *header++ = seqno;          /* seqno */
  *header++ = 0;              /* hop count */
  *header++ = 0;              /* number of hops */
  *header++ = dest.u8[0];     /* destination addr H */
  *header++ = dest.u8[1];     /* destination addr L */

  /* The packetbuf is filled and ready to be sent */
}
/*---------------------------------------------------------------------------*/
void
request_line_create(request_line * line, uint8_t request, linkaddr_t * dest, uint8_t * data, uint8_t datalen)
{
  line->reqlen = CONECTRIC_REQUEST_HEADER_LEN + 1 + datalen;
  line->req = request;
  line->dest.u8[0] = dest->u8[0];
  line->dest.u8[1] = dest->u8[1];
  line->hdr_next = 0x01; /* reserved, always 0x01 */
  line->payload = data;
}
/*---------------------------------------------------------------------------*/
