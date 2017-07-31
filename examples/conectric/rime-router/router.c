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
 *         with ranking and multihop
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Ekawahyu Susilo <ekawahyu@yahoo.com>
 */

#include "contiki.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "random.h"
#include "flash-logging.h"

#include "dev/button-sensor.h"
#include "dev/rs485-arch.h"
#include "dev/serial-line.h"
#include "dev/modbus-line.h"
#include "dev/modbus-crc16.h"

#include "dev/uart-arch.h"
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
  /* abc messages */
  CONECTRIC_SENSOR_BROADCAST,
  /* trickle messages */
  CONECTRIC_ROUTE_REQUEST,
  CONECTRIC_ROUTE_REQUEST_BY_SN,
  CONECTRIC_ROUTE_REPLY,
  CONECTRIC_TIME_SYNC,
  /* multihop messages */
  CONECTRIC_SET_LONG_MAC,
  CONECTRIC_SET_LONG_MAC_REPLY,
  CONECTRIC_GET_LONG_MAC,
  CONECTRIC_GET_LONG_MAC_REPLY,
  CONECTRIC_POLL_RS485,
  CONECTRIC_POLL_RS485_REPLY,
  CONECTRIC_POLL_RS485_CHUNK,
  CONECTRIC_POLL_RS485_CHUNK_REPLY,
  CONECTRIC_POLL_SENSORS,
  CONECTRIC_POLL_SENSORS_REPLY,
  CONECTRIC_POLL_NEIGHBORS,
  CONECTRIC_POLL_NEIGHBORS_REPLY,
  CONECTRIC_MULTIHOP_PING,
  CONECTRIC_MULTIHOP_PING_REPLY,
  CONECTRIC_ATTR_MAX
};

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

static void compose_request_to_packetbuf(
    uint8_t * request, uint8_t seqno, linkaddr_t * ereceiver);
static void compose_response_to_packetbuf(
    uint8_t * request, uint8_t seqno, linkaddr_t * ereceiver);
static linkaddr_t * call_decision_maker(void * incoming, uint8_t type);

#define REQUEST_HEADER_LEN    4

#define MESSAGE_BYTEREQ       1
#define MESSAGE_BYTECMD       2
#define MESSAGE_ABC_RECV      3
#define MESSAGE_TRICKLE_RECV  4
#define MESSAGE_MHOP_RECV     5 /* uses mhop_message_recv to store message */
#define MESSAGE_MHOP_FWD      6 /* uses mhop_message_recv to store message */

static message_recv abc_message_recv;
static message_recv trickle_message_recv;
static message_recv mhop_message_recv;

extern volatile uint16_t deep_sleep_requested;

/* Flash Logging */
static uint8_t logData[4]= {0x00, 0x00, 0x00, 0x00};

#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))

static uint16_t rank = 255;
static uint8_t sensors[128];
static uint8_t *sensors_head, *sensors_tail;

#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *gmacp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *gmacp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *gmacp = (__code unsigned char *)0xFFE8;
#endif

/* RS485 */
#define BUFSIZE 256
static uint16_t rs485_in_pos;
static uint8_t rs485_buffer[BUFSIZE];

/* EKM Messaging */
#define RS485_DATA_MAX_SIZE 20
static uint8_t rs485_data_request;
static linkaddr_t rs485_data_recv;
static uint8_t rs485_data_payload[RS485_DATA_MAX_SIZE];

static uint8_t dump_buffer = 0;
/*---------------------------------------------------------------------------*/
/*
 * TODO add pre-built command line to collect git information on IAR
 *
 */
#ifndef CONECTRIC_VERSION_STRING
#define CONECTRIC_VERSION_STRING "Contiki-unknown"
#endif
#ifndef CONECTRIC_PROJECT_STRING
#define CONECTRIC_PROJECT_STRING "unknow"
#endif
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

  /* Decoding request byte */
  dataptr = message->payload;
  message->request = *++dataptr;

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
static void
dump_payload(void)
{
  static uint16_t len;
  static char * packetbuf;

  putstring(">");

  len = packetbuf_datalen();
  packetbuf = (char *)packetbuf_dataptr();
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}
/*---------------------------------------------------------------------------*/
static uint8_t
shortaddr_cmp(linkaddr_t * addr1, linkaddr_t * addr2)
{
  return (addr1->u8[0] == addr2->u8[0] && addr1->u8[1] == addr2->u8[1]);
}
/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ConBurst");
PROCESS(example_trickle_process, "ConTB");
PROCESS(example_multihop_process, "ConMHop");
PROCESS(serial_in_process, "SerialIn");
PROCESS(modbus_in_process, "ModbusIn");
PROCESS(modbus_out_process, "ModbusOut");
PROCESS(modbus_wi_test, "WITest");
PROCESS(flash_log_process, "FlashLog");
#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "ButtonTest");
AUTOSTART_PROCESSES(
    &example_abc_process,
    &example_trickle_process,
    &example_multihop_process,
    &serial_in_process,
    &modbus_in_process,
    &modbus_out_process,
    &modbus_wi_test,
    &flash_log_process,
    &buttons_test_process);
#else
AUTOSTART_PROCESSES(
    &example_abc_process,
    &example_trickle_process,
    &example_multihop_process,
    &serial_in_process,
    &modbus_in_process,
    &modbus_out_process,
    &modbus_wi_test,
    &flash_log_process);
#endif

/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  packetbuf_and_attr_copyto(&abc_message_recv, MESSAGE_ABC_RECV);

  /* TODO only the sink should dump packetbuf,
   * but routers have to store sensors data
   */
  if (dump_buffer)
    dump_packetbuf();
  else
    dump_payload();

  PRINTF("%d.%d: found sensor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      abc_message_recv.sender.u8[0], abc_message_recv.sender.u8[1],
      abc_message_recv.rssi, abc_message_recv.timestamp);

  call_decision_maker(&abc_message_recv, MESSAGE_ABC_RECV);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{
  uint8_t * dataptr;

  packetbuf_and_attr_copyto(&trickle_message_recv, MESSAGE_TRICKLE_RECV);

  /* Decoding ereceiver address from message, no built-in trickle attribute */
  dataptr = trickle_message_recv.payload;
  trickle_message_recv.ereceiver.u8[1] = *--dataptr;
  trickle_message_recv.ereceiver.u8[0] = *--dataptr;

  /* Get the rank to the sink */
  rank = trickle_rank(c);

  /* TODO store neighbors as a list here */

  PRINTF("%d.%d: found neighbor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      trickle_message_recv.sender.u8[0], trickle_message_recv.sender.u8[1],
      trickle_message_recv.rssi, trickle_message_recv.timestamp);

  call_decision_maker(&trickle_message_recv, MESSAGE_TRICKLE_RECV);
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
  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);

  /* TODO only the sink should dump packetbuf */
  if (dump_buffer)
    dump_packetbuf();
  else
    dump_payload();

  PRINTF("%d.%d: multihop message from %d.%d - (%d hops) - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        mhop_message_recv.esender.u8[0], mhop_message_recv.esender.u8[1],
        mhop_message_recv.hops, clock_seconds());

  call_decision_maker(&mhop_message_recv, MESSAGE_MHOP_RECV);
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
  static linkaddr_t * forward_addr;

  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);

  forward_addr = call_decision_maker(&mhop_message_recv, MESSAGE_MHOP_FWD);

  PRINTF("%d.%d: multihop forwarding address is %d.%d - %d hops\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      forward_addr->u8[0], forward_addr->u8[1], mhop_message_recv.hops);

  return forward_addr;
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
  static uint8_t counter = 0;
  static uint8_t * request;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  /* Open a trickle connection */
  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    /* Compose packetbuf for request comes from serial port with '<' marking
     * or request comes from radio with no '<' marking
     */
    request = (uint8_t *)data;
    if (*request == '<')
      compose_request_to_packetbuf(request, counter++, &to);
    else
      compose_response_to_packetbuf(request, counter++, &to);

    /* Send the rank to 1 (source of trickle) */
    trickle_set_rank(1);

    /* Send the packet */
    trickle_send(&trickle);

    PRINTF("%d.%d: route request sent to %d.%d - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data)
{
  //static struct etimer et;
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

    /* Compose packetbuf for request comes from serial port with '<' marking
     * or request comes from radio with no '<' marking
     */
    request = (uint8_t *)data;
    if (*request == '<')
      compose_request_to_packetbuf(request, counter++, &to);
    else
      compose_response_to_packetbuf(request, counter++, &to);

    /* TODO delay count (for now) 1 seconds for (local) trickle to subside */
    //etimer_set(&et, CLOCK_SECOND);
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send the packet */
    multihop_send(&multihop, &to);

    PRINTF("%d.%d: multihop sent to %d.%d - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_test_process, ev, data)
{
  struct sensors_sensor *sensor;
  static uint8_t counter;
  static uint8_t button;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_1_sensor) {
      button = 0x71;
      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
    }
    if(sensor == &button_2_sensor) {
      button = 0x72;
      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
    }
  }

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static uint8_t * request;
  static uint8_t counter;
  static uint8_t hex_string[2];
  static uint8_t bytereq[128];

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    printf("%s\n", (uint8_t *)data);

    request = (uint8_t *)data;
    memset(bytereq, 0, sizeof(bytereq));

    if (request[0] == '<') {

      bytereq[0] = '<';
      counter = 2;

      /* do conversion from hex string to hex bytes */
      while(*++request != '\0') {

        /* remove space */
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

      call_decision_maker(bytereq, MESSAGE_BYTEREQ);

    }
    else {

      counter = 0;

      /* passthrough until end of line found */
      while(*request != '\0') {
        /* remove space */
        if (*request == ' ') {
          request++;
          continue;
        }
        if (*request >= 0x61 && *request <= 0x7A)
          *request -= 0x20;
        bytereq[counter++] = *request++;
      }

      call_decision_maker(bytereq, MESSAGE_BYTECMD);

    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_in_process, ev, data)
{
  static uint8_t datasize;
  static uint8_t* dataptr;
  static uint16_t crc;
  uint8_t cnt;

  PROCESS_BEGIN();

  /* FIXME workaround to send at least one character out so that
   * PROCESS_WAIT_EVENT() can receive modbus_line_event_message
   */
  uart_arch_writeb(0);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);
    dataptr = data;
    printf("got modbus data (%d)\n", *dataptr);
    dataptr++;
    while (*dataptr != 0) {
      puthex(*dataptr++);
    }
    putstring("\n");
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_out_process, ev, data)
{
  static struct etimer et;
  static message_recv * message;
  static uint8_t * serial_data;
  static uint8_t reqlen;
  static uint8_t req;
  static uint8_t len;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    message = (message_recv *)data;
    serial_data = message->payload;
    reqlen = *serial_data++;
    req = *serial_data++;

    len = reqlen - 2;

    /* reset modbus input index */
    rs485_in_pos = 0;

    /* modbus write */
    putstring("modbus_out_process: ");
    while(len--) {
      puthex(*serial_data);
      uart_arch_writeb(*serial_data++);
    }
    putstring("\n");
    
    // store message information from last S/N query for transmission later (don't assume the message structure is still valid)
    rs485_data_request = message->request;
    linkaddr_copy(&rs485_data_recv, &message->ereceiver);
    memcpy(rs485_data_payload, message->payload, message->length);
  }

  PROCESS_END();
}
void
fill_modbus_payload(uint8_t * payload, uint8_t id, uint8_t fc, uint16_t word1, uint16_t word2)
{
  static uint16_t crc16_result;
  static uint8_t * modbus_payload;

  *payload++ = 10; /* length, not part of modbus */
  *payload++ = 0;  /* request, not part of modbus */
  modbus_payload = payload;
  *payload++ = id; /* slave id */
  *payload++ = fc; /* function code */
  *payload++ = (word1 & 0xFF00) >> 8; /* register starting address H */
  *payload++ = word1 & 0x00FF;        /* register starting address L */
  *payload++ = (word2 & 0xFF00) >> 8; /* value/total number of registers requested H */
  *payload++ = word2 & 0x00FF;        /* value/total number of registers requested L */
  crc16_result = modbus_crc16_calc(modbus_payload, 6);
  *payload++ = crc16_result & 0x00FF;
  *payload++ = (crc16_result & 0xFF00) >> 8;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_wi_test, ev, data)
{
  static struct etimer et;
  static uint8_t payload[20];
  static message_recv message;
  static uint16_t crc16_result;
  static uint16_t counter;

  PROCESS_BEGIN();

  while(1) {

    /*** read register holding from 1-20 one at a time ***/
    printf("modbus_wi_test: ID=1, FC=0x03, read 20 times\n");
    counter = 1;
    while (counter <= 20) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x03, counter++, 0x0001);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    /*** read register holding from 1-20 all at once ***/
    printf("modbus_wi_test: ID=1, FC=0x03, read all at once\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x03, 0x0001, 0x0014);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* read register holding from 1-20 one at a time FROM DEVICE ID=2,
     * WI should not respond to any of these read request
     */
    printf("modbus_wi_test: ID=2, FC=0x03, read 20 times\n");
    counter = 1;
    while (counter <= 20) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x02, 0x03, counter++, 0x0001);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, backlight dim up\n");
    counter = 5;
    while (counter <= 10) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x0008, counter++);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, backlight dim down\n");
    counter = 10;
    while (counter >= 5) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x0008, counter--);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, backlight delay off 10 seconds\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0009, 0x000a);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, power off\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000a, 0x0000);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, power on\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000a, 0x0001);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, home icon 0-9\n");
    counter = 0x0030;
    while (counter <= 0x0039) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x000b, counter++);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, home icon A-F\n");
    counter = 0x0041;
    while (counter <= 0x0046) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x000b, counter++);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, home icon L\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000b, 'L');
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, home icon H\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000b, 'H');
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, home icon P\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000b, 'P');
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, home icon U\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000b, 'U');
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, home icon off\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000b, 0x0000);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, days icon 1-7 then off\n");
    counter = 0x0001;
    while (counter <= 0x0100) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x000c, counter & 0x00FF);
      counter = counter << 1;
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, playing buzzer from 5-1\n");
    counter = 0x0005;
    while (counter > 0) {
      message.payload = payload;
      fill_modbus_payload(payload, 0x01, 0x06, 0x000d, counter--);
      process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

    printf("modbus_wi_test: ID=1, FC=0x06, turning off buzzer\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000d, 0);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, clock showing 00:00\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000e, 0x0000);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, clock showing 99:99\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000e, 0x6363);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temperature to 10.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000f, 100);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temperature to 10.5 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000f, 105);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temperature to 11.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000f, 110);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temperature to 11.5 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x000f, 115);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set low temperature to 10.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0010, 100);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set low temperature to 20.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0010, 200);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set high temperature to 20.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0011, 200);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set high temperature to 32.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0011, 320);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temp compensation to +5.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0012, 50);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temp compensation to +4.5 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0012, 45);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temp compensation to -5.0 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0012, 0x00CE);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, set temp compensation to -4.5 deg C\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0012, 0x00D3);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, show RH data\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0013, 0);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x06, show WI ID\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x06, 0x0013, 1);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("modbus_wi_test: ID=1, FC=0x03, read status bytes\n");
    message.payload = payload;
    fill_modbus_payload(payload, 0x01, 0x03, 0x0014, 0x0001);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_log_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  flashlogging_init();

  while (1)
  {
    etimer_set(&et, LOGGING_REF_TIME_PD);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, 0);
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
  uint8_t * route = NULL;
  linkaddr_t dest;
  uint8_t req;
  uint8_t reqlen;
  uint8_t datalen;
  uint8_t routelen;
  uint8_t i;

  /* Request from serial port, skip the '<' */
  request++;

  reqlen     = *request++;
  req        = *request++;
  dest.u8[0] = *request++;
  dest.u8[1] = *request++;
  routelen   = *request++;

  if (ereceiver) linkaddr_copy(ereceiver, &dest);

  /* Filling in packetbuf with data and skip routing table */
  memset(packet_buffer, 0, sizeof(packet_buffer));
  datalen = reqlen - routelen - REQUEST_HEADER_LEN;
  *packet++ = datalen + 2;
  *packet++ = req;
  route = request;
  request += (routelen - 1);
  i = datalen;
  while (i--) *packet++ = *request++;

  packetbuf_copyfrom(packet_buffer, datalen+2);

  routelen--; /* get rid of the length byte */

  packetbuf_hdralloc(6 + routelen);

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = 6 + routelen;   /* header len */
  *header++ = seqno;          /* seqno */
  *header++ = 0;              /* hop count */
  *header++ = 0;              /* number of hops */
  *header++ = dest.u8[0];     /* destination addr H */
  *header++ = dest.u8[1];     /* destination addr L */
  while(routelen--)
        *header++ = *route++; /* routing table */
}
/*---------------------------------------------------------------------------*/
static void
compose_response_to_packetbuf(uint8_t * radio_request,
    uint8_t seqno, linkaddr_t * ereceiver)
{
  static uint8_t packet_buffer[128];
  uint8_t * packet = packet_buffer;
  uint8_t * header = NULL;
  uint8_t req;
  uint8_t reqlen;
  uint8_t response = 0;
  uint8_t responselen;
  uint8_t chunk_number = 0;
  uint8_t chunk_size = 0;
  uint8_t i;

  reqlen = *radio_request++;
  req    = *radio_request++;

  responselen = 2;

  /* Responses to trickle requests */
  if (req == CONECTRIC_ROUTE_REQUEST) {
    response = CONECTRIC_ROUTE_REPLY;
    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
  }
  if (req == CONECTRIC_ROUTE_REQUEST_BY_SN) {
    response = CONECTRIC_ROUTE_REPLY;
    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
  }

  /* Responses to multihop requests */
  if (req == CONECTRIC_MULTIHOP_PING) {
    response = CONECTRIC_MULTIHOP_PING_REPLY;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
  if (req == CONECTRIC_POLL_RS485) {
    response = CONECTRIC_POLL_RS485_REPLY;
    responselen += 2;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
  if (req == CONECTRIC_POLL_RS485_CHUNK) {
    response = CONECTRIC_POLL_RS485_CHUNK_REPLY;
    chunk_number = *radio_request++;
    chunk_size   = *radio_request++;
    responselen += chunk_size;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
  if (req == CONECTRIC_POLL_SENSORS) {
    response = CONECTRIC_POLL_SENSORS_REPLY;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
  if (req == CONECTRIC_GET_LONG_MAC) {
    response = CONECTRIC_GET_LONG_MAC_REPLY;
    responselen += 8;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }

  memset(packet_buffer, 0, sizeof(packet_buffer));
  *packet++ = responselen;
  *packet++ = response;

  i = responselen-2;

  if (req == CONECTRIC_POLL_RS485) {
    /* FIXME this has to be calculated from RS485 reply length */
    *packet++ = 0x04; /* number of chunks available to poll */
    *packet++ = 0x40; /* chunk size */
  }

  if (req == CONECTRIC_POLL_RS485_CHUNK) {
    for (i = 0; i < chunk_size; i++)
      *packet++ = rs485_buffer[(chunk_size*chunk_number) + i];
  }

  if (req == CONECTRIC_GET_LONG_MAC) {
    gmacp = &X_IEEE_ADDR;
    while (i--) {
      *packet++ = gmacp[i];
      puthex(gmacp[i]);
    }
    putstring("\n");
  }

  packetbuf_copyfrom(packet_buffer, responselen);

  packetbuf_hdralloc(6);

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = 6;                /* header len */
  *header++ = seqno;            /* seqno */
  *header++ = 0;                /* hop count */
  *header++ = 0;                /* number of hops */
  *header++ = ereceiver->u8[0]; /* destination addr H */
  *header++ = ereceiver->u8[1]; /* destination addr L */
}
/*---------------------------------------------------------------------------*/
static linkaddr_t *
call_decision_maker(void * incoming, uint8_t type)
{
  static linkaddr_t forward_addr;
  message_recv * message = (message_recv *)incoming;
  uint8_t * bytereq = (uint8_t *)incoming;
  uint8_t request;
  uint8_t seqno, mhops, hdrlen;
  uint8_t * header;
  int i;

  /*******************************************************/
  /***** INTERPRETING COMMAND LINES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * BYTECMD Protocol:
   * - It starts with any char, but '<'
   * - Non-capital letter inputs get capitalized automatically
   *
   */
  if (type == MESSAGE_BYTECMD) {

    /* Command line interpreter */
    if (bytereq[0] == 'M' && bytereq[1] == 'R') {
      gmacp = &X_IEEE_ADDR;
      for(i = 7; i >= 0; i--) puthex(gmacp[i]);
      putstring("\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'P') {
      dump_buffer = 0;
      putstring("Ok DP\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'B') {
      dump_buffer = 1;
      putstring("Ok DB\n");
    }

    else if (bytereq[0] == 'V' && bytereq[1] == 'E' && bytereq[2] == 'R') {
      putstring(CONTIKI_VERSION_STRING "\n");
      putstring(CONECTRIC_PROJECT_STRING "\n");
    }

    /* Unknown command */
    else {
      puthex(linkaddr_node_addr.u8[0]);
      putstring(".");
      puthex(linkaddr_node_addr.u8[1]);
      putstring(": Bad command!\n");
    }

  }

  /*******************************************************/
  /***** INTERPRETING REQUEST BYTES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * BYTEREQ Protocol:
   * -----------------
   * [<][Len][Req][DestH][DestL][RLen][R1H][R1L]...[RnH][RnL][Data0][Data1]...
   *
   * [Len]  = request byte length including [Len], but excluding [<]
   * [RLen] = routing table length including [RLen] itself
   * [RnH]  = the last hop address H ---> [DestH]
   * [RnL]  = the last hop address L ---> [DestL]
   *
   */
  else if (type == MESSAGE_BYTEREQ) {

    request = bytereq[2];

    /* Request bytes to be sent as trickle */
    if (request == CONECTRIC_ROUTE_REQUEST ||
        request == CONECTRIC_ROUTE_REQUEST_BY_SN)
      process_post(&example_trickle_process, PROCESS_EVENT_CONTINUE, bytereq);

    /* Request bytes to be sent as multihop */
    else if (
        request == CONECTRIC_MULTIHOP_PING ||
        request == CONECTRIC_POLL_RS485  ||
        request == CONECTRIC_POLL_RS485_CHUNK  ||
        request == CONECTRIC_POLL_SENSORS  ||
        request == CONECTRIC_GET_LONG_MAC)
      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, bytereq);

    /* Unknown request */
    else {
      puthex(linkaddr_node_addr.u8[0]);
      putstring(".");
      puthex(linkaddr_node_addr.u8[0]);
      putstring(": Unknown request - 0x");
      puthex(request);
      putstring("\n");
    }

  }

  /*******************************************************/
  /***** RULES TO ASSIGN MULTIHOP FORWARDING ADDRESS *****/
  /*******************************************************/
  else if (type == MESSAGE_MHOP_FWD) {

    /* multihop message received but need to be forwarded */
    seqno = mhop_message_recv.seqno;
    mhops = mhop_message_recv.hops;
    hdrlen = mhop_message_recv.message[0];

    /* Discard current packet header */
    packetbuf_copyfrom(mhop_message_recv.payload, mhop_message_recv.length);

    packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
    packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.ereceiver);
    packetbuf_set_attr(PACKETBUF_ATTR_HOPS, mhops);

    /* Compose header */
    if (mhop_message_recv.request == CONECTRIC_ROUTE_REPLY) {
      packetbuf_hdralloc(hdrlen+2);
      header = (uint8_t *)packetbuf_hdrptr();
      *header++ = hdrlen+2;
    }
    else {
      packetbuf_hdralloc(hdrlen);
      header = (uint8_t *)packetbuf_hdrptr();
      *header++ = hdrlen;
    }

    *header++ = seqno;
    *header++ = mhops;
    *header++ = mhop_message_recv.message[3]; /* TODO assign max hops here */
    *header++ = mhop_message_recv.ereceiver.u8[0];
    *header++ = mhop_message_recv.ereceiver.u8[1];

    /* multihop request with built-in routing table */
    if (mhop_message_recv.request == CONECTRIC_MULTIHOP_PING ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485  ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_CHUNK  ||
        mhop_message_recv.request == CONECTRIC_POLL_SENSORS  ||
        mhop_message_recv.request == CONECTRIC_GET_LONG_MAC) {
      forward_addr.u8[0] = mhop_message_recv.message[4 + (mhops << 1)];
      forward_addr.u8[1] = mhop_message_recv.message[5 + (mhops << 1)];
    }
    /* multihop reply, no routing table */
    if (mhop_message_recv.request == CONECTRIC_MULTIHOP_PING_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_CHUNK_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_SENSORS_REPLY ||
        mhop_message_recv.request == CONECTRIC_GET_LONG_MAC_REPLY) {
      linkaddr_copy(&forward_addr, &mhop_message_recv.prev_sender);
      packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
      packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.prev_esender);
    }
    /* multihop reply, update routing table on every hop */
    if (mhop_message_recv.request == CONECTRIC_ROUTE_REPLY) {
      *header++ = linkaddr_node_addr.u8[0];
      *header++ = linkaddr_node_addr.u8[1];
      linkaddr_copy(&forward_addr, &trickle_message_recv.sender);
    }

    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }

    return &forward_addr;
  }

  /*******************************************************/
  /***** ALL THE REST ARE HANDLED HERE *******************/
  /*******************************************************/
  else {

    /* abc message received */
    /* TODO store sensors data as a ring buffer with timestamp */

    /* trickle message received */
    if (message->request == CONECTRIC_ROUTE_REQUEST)
      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE,
            message->payload);

    if (message->request == CONECTRIC_ROUTE_REQUEST_BY_SN)
      if (message->ereceiver.u8[0] == 0xFF && message->ereceiver.u8[1] == 0xFF)
        process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE,
            message);

    /* multihop message received */
    if (message->request == CONECTRIC_MULTIHOP_PING ||
        message->request == CONECTRIC_POLL_RS485_CHUNK  ||
        message->request == CONECTRIC_POLL_SENSORS  ||
        message->request == CONECTRIC_GET_LONG_MAC)
      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE,
            message->payload);

    if (message->request == CONECTRIC_POLL_RS485)
      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
        process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE,
            message);

  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
