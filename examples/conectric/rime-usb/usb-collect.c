/*
 * usb-collect.c
 *
 * Created on: Feb 23, 2018
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

/* General */
#include <stdio.h>

/* Contiki */
#include "contiki.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"
#include "net/netstack.h"
#include "random.h"

/* Conectric Device */
#if RUN_ON_COOJA_SIMULATION
#else
#include "flash-logging.h"
#include "dev/adc-sensor.h"
#endif
#include "dev/serial-line.h"
#include "command.h"

/* Conectric Network */
#include "examples/conectric/conectric-messages.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* PLS Network Parameters */
#define USB_SUP_TIMEOUT         180 /* minutes */
#define USB_PERIODIC_TIMEOUT    1   /* minutes */
#define USB_HEADER_SIZE         2
#define USB_PAYLOAD_SIZE        4
static uint8_t message[CONECTRIC_MESSAGE_LENGTH];
extern volatile uint16_t deep_sleep_requested;

/* PLS Device Parameters */
#define USB_PULSE_PERIODIC       0x92
#define USB_PULSE_NOEVT          0x00
#define USB_SUP_EVT              0xBB
#define USB_SUP_NOEVT            0x00

/* Flash Logging */
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
enum
{
  USB_RESERVED = 0x00,    // reserved
  USB_SEND     = 0x01,    // send data event
};

static message_recv abc_message_recv;
static message_recv conectric_message_recv;

#define MESSAGE_ABC_RECV      3
#define MESSAGE_TRICKLE_RECV  4
#define MESSAGE_MHOP_RECV     5 /* uses mhop_message_recv to store message */
#define MESSAGE_MHOP_FWD      6 /* uses mhop_message_recv to store message */
#define MESSAGE_CONECTRIC_RECV      7

static uint8_t dump_buffer = 0;

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
void
dump_packet_buffer(uint8_t mode)
{
  dump_buffer = mode;
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
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static struct conectric_conn conectric;
/*---------------------------------------------------------------------------*/
PROCESS(usb_abc_process, "USB Collect");
PROCESS(usb_supervisory_process, "USB Supervisory");
PROCESS(example_conectric_process, "Conectric Example");
//PROCESS(flash_log_process, "Flash Log");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &usb_abc_process,
    &usb_supervisory_process,
    &example_conectric_process,
//    &flash_log_process,
    &serial_in_process
);
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

  // call_decision_maker(&abc_message_recv, MESSAGE_ABC_RECV);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(usb_abc_process, ev, data)
{
  static unsigned int batt;
  static uint8_t seqno = 0;
  static float sane;
  static int dec;
  static float frac;
  static uint8_t *sensor_data;
  static struct etimer et;
  static uint8_t loop;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  /* Composing boot status message */
  memset(message, 0, sizeof(message));
  message[0] = 2;
  message[1] = seqno++;
  message[2] = 4;
  message[3] = CONECTRIC_DEVICE_BROADCAST_BOOT_STATUS;
#if RUN_ON_COOJA_SIMULATION
  batt = 0;
#else
  batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
#endif
  sane = batt * 3 * 1.15 / 2047;
  dec = sane;
  frac = sane - dec;
  message[4] = (char)(dec*10)+(char)(frac*10);
  message[5] = clock_reset_cause();

  loop = CONECTRIC_BURST_NUMBER;

  while(loop--) {

    packetbuf_copyfrom(message, 2 + 4);
    NETSTACK_MAC.on();
    abc_send(&abc);

    PROCESS_WAIT_EVENT();

#if LPM_CONF_MODE
    if (loop)
      deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
    else
      deep_sleep_requested = 60 * CLOCK_SECOND;
#else
    if (loop) {
      etimer_set(&et, 1 + random_rand() % (CLOCK_SECOND / 8));
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
#endif

  }

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

#if RUN_ON_COOJA_SIMULATION
    batt = 0;
#else
    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
#endif
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;

    sensor_data = (uint8_t*)data;

    if(*sensor_data == USB_PULSE_PERIODIC)
    {
      memset(message, 0, sizeof(message));
      message[0] = USB_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = USB_PAYLOAD_SIZE;
      message[3] = CONECTRIC_SENSOR_BROADCAST_USB;
      message[4] = (char)(dec*10)+(char)(frac*10);
      message[5] = (char)USB_PULSE_PERIODIC;

      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        abc_send(&abc);

        PROCESS_WAIT_EVENT();

#if LPM_CONF_MODE
        if (loop)
          deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
        else
          deep_sleep_requested = 60 * CLOCK_SECOND;
#else
        if (loop) {
          etimer_set(&et, 1 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
#endif

      }
    }
    else if(*sensor_data == USB_SUP_EVT)
    {
      uint16_t time = clock_seconds();

      memset(message, 0, sizeof(message));
      message[0] = USB_HEADER_SIZE;                      // Header Length
      message[1] = seqno++;                             // Sequence number
      message[2] = USB_PAYLOAD_SIZE;                     // Payload Length
      message[3] = CONECTRIC_SUPERVISORY_REPORT;        // Payload Type
      message[4] = (char)(dec*10)+(char)(frac*10);      // battery
      message[5] = (char)(time >> 6);                   // time (rough min)

      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        abc_send(&abc);

        PROCESS_WAIT_EVENT();

#if LPM_CONF_MODE
        if (loop)
          deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
        else
          deep_sleep_requested = 60 * CLOCK_SECOND;
#else
        if (loop) {
          etimer_set(&et, 1 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
#endif

      }
    }
    else if(*sensor_data == USB_PULSE_NOEVT || *sensor_data == USB_SUP_NOEVT) {
#if LPM_CONF_MODE
      deep_sleep_requested = 60 * CLOCK_SECOND;
#endif
    }
  }

  PROCESS_END();
}
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

  /* TODO only the sink should dump packetbuf,
   * but routers have to store sensors data
   */
  if (dump_buffer)
    dump_packetbuf();
  else
    dump_payload();

  PRINTF("Data received from %d.%d: %.*s (%d)\n",
   from->u8[0], from->u8[1],
   packetbuf_datalen(), (char *)packetbuf_dataptr(), packetbuf_datalen());

  // call_decision_maker(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);
}

const static struct conectric_callbacks callbacks = {recv, sent, timedout};
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
PROCESS_THREAD(usb_supervisory_process, ev, data)
{
  static struct etimer et;
  static uint8_t event;
  static int16_t supervisory_counter;
  static int16_t periodic_counter;

  PROCESS_BEGIN();

  supervisory_counter = USB_SUP_TIMEOUT;
  periodic_counter = USB_PERIODIC_TIMEOUT;

  while (1)
  {
    etimer_set(&et, 60 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send supervisory message */
    if (supervisory_counter > 1) {
      supervisory_counter--;
      event = USB_SUP_NOEVT;
      process_post(&usb_abc_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      supervisory_counter = USB_SUP_TIMEOUT;
      event = USB_SUP_EVT;
      process_post(&usb_abc_process, PROCESS_EVENT_CONTINUE, &event);
    }

//    /* Send periodic message */
//    if (periodic_counter > 1) {
//      PRINTF("usb_periodic: no event\n");
//      periodic_counter--;
//      event = USB_PULSE_NOEVT;
//      process_post(&example_conectric_process, PROCESS_EVENT_CONTINUE, &event);
//    }
//    else {
//      PRINTF("usb_periodic: periodic event\n");
//      periodic_counter = USB_PERIODIC_TIMEOUT;
//      event = USB_PULSE_PERIODIC;
//      process_post(&example_conectric_process, PROCESS_EVENT_CONTINUE, &event);
//    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
//PROCESS_THREAD(flash_log_process, ev, data)
//{
//  static struct etimer et;
//
//  PROCESS_BEGIN();
//
//  flashlogging_init();
//
//  while (1)
//  {
//    etimer_set(&et, LOGGING_REF_TIME_PD);
//    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
//
//    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, 0);
//  }
//
//  PROCESS_END();
//}
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
void
invoke_process_before_sleep(void)
{
  process_post_synch(&usb_abc_process, PROCESS_EVENT_CONTINUE, NULL);
}
/*---------------------------------------------------------------------------*/