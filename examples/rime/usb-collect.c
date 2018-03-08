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
#include "dev/button-sensor.h"
#else
#include "flash-logging.h"
#include "dev/adc-sensor.h"
#endif
#include "dev/serial-line.h"
#include "command.h"

/* Conectric Network */
#include "examples/conectric/conectric-messages.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* USB Network Parameters */
#define USB_SUP_TIMEOUT         180 /* minutes */
#define USB_PERIODIC_TIMEOUT    1   /* minutes */
#define USB_HEADER_SIZE         6
#define USB_BOOT_PAYLOAD_SIZE   4
#define USB_PAYLOAD_SIZE        4
static uint8_t message[CONECTRIC_MESSAGE_LENGTH];
extern volatile uint16_t deep_sleep_requested;

/* USB Device Parameters */
#define USB_COLLECT_PERIODIC     0x92
#define USB_COLLECT_NOEVT        0x00
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

#define MESSAGE_BROADCAST_RECV    3
#define MESSAGE_CONECTRIC_RECV    7

static message_recv broadcast_message_recv;
static message_recv conectric_message_recv;

static uint8_t dump_header = 0;

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
  packetbuf = (char *)message->message;
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}
/*---------------------------------------------------------------------------*/
struct conectric_conn conectric;
/*---------------------------------------------------------------------------*/
PROCESS(usb_broadcast_process, "USB Collect");
PROCESS(usb_supervisory_process, "USB Supervisory");
PROCESS(usb_conectric_process, "USB Conectric");
//PROCESS(flash_log_process, "Flash Log");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &usb_broadcast_process,
    &usb_supervisory_process,
    &usb_conectric_process,
//    &flash_log_process,
    &serial_in_process
);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  packetbuf_and_attr_copyto(&broadcast_message_recv, MESSAGE_BROADCAST_RECV);

  dump_packetbuf(&broadcast_message_recv);

  PRINTF("%d.%d: local broadcast from %d.%d rssi %d ts %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1],
      broadcast_message_recv.rssi, broadcast_message_recv.timestamp);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(usb_broadcast_process, ev, data)
{
  static unsigned int batt;
  static uint8_t seqno = 0;
  static float sane;
  static int dec;
  static float frac;
  static uint8_t *sensor_data;
  static struct etimer et;
  static uint8_t loop;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

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

    if(*sensor_data == USB_COLLECT_PERIODIC)
    {
      memset(message, 0, sizeof(message));
      message[0] = USB_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0xFF;
      message[5] = 0xFF;
      message[6] = USB_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SENSOR_BROADCAST_USB;
      message[8] = (char)(dec*10)+(char)(frac*10);
      message[9] = (char)USB_COLLECT_PERIODIC;

      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        /* commented out because it execute on event that is not sent to it
         * need an immediate fix
         */
        // broadcast_send(&broadcast);

#if RUN_ON_COOJA_SIMULATION
#else
        PROCESS_WAIT_EVENT();
#endif

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
      message[0] = USB_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0xFF;
      message[5] = 0xFF;
      message[6] = USB_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SUPERVISORY_REPORT;
      message[8] = (char)(dec*10)+(char)(frac*10);
      message[9] = (char)(time >> 6);

      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        broadcast_send(&broadcast);

#if RUN_ON_COOJA_SIMULATION
#else
        PROCESS_WAIT_EVENT();
#endif

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
    else if(*sensor_data == USB_COLLECT_NOEVT || *sensor_data == USB_SUP_NOEVT) {
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

  dump_packetbuf(&conectric_message_recv);

  PRINTF("%d.%d: data from %d.%d len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

static void
netbroadcast(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  packetbuf_and_attr_copyto(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);

  dump_packetbuf(&conectric_message_recv);

  PRINTF("%d.%d: broadcast from %d.%d: len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

static void
sink(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  PRINTF("%d.%d: sink from %d.%d len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

const static struct conectric_callbacks callbacks = {recv, sent, timedout, netbroadcast, sink};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(usb_conectric_process, ev, data)
{
  static uint8_t hexstring[20];
  static uint8_t bytereq[20];

  static unsigned int batt;
  static uint8_t seqno = 0;
  static float sane;
  static int dec;
  static float frac;
  static struct etimer et;
  static uint8_t loop;

  static uint8_t * request;
  static linkaddr_t to;

  PROCESS_EXITHANDLER(conectric_close(&conectric);)

  PROCESS_BEGIN();

  conectric_open(&conectric, 132, &callbacks);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

#if RUN_ON_COOJA_SIMULATION
  SENSORS_ACTIVATE(button_sensor);
#endif

  /* Composing boot status message */
  memset(message, 0, sizeof(message));
  message[0] = USB_HEADER_SIZE;
  message[1] = seqno++;
  message[2] = 0;
  message[3] = 0;
  message[4] = 0xFF;
  message[5] = 0xFF;
  message[6] = USB_BOOT_PAYLOAD_SIZE;
  message[7] = CONECTRIC_DEVICE_BROADCAST_BOOT_STATUS;
#if RUN_ON_COOJA_SIMULATION
  batt = 0;
  sane = batt * 3 * 1.15 / 2047;
  dec = sane;
  frac = sane - dec;
  message[8] = (char)(dec*10)+(char)(frac*10);
  message[9] = 0;
#else
  batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
  sane = batt * 3 * 1.15 / 2047;
  dec = sane;
  frac = sane - dec;
  message[8] = (char)(dec*10)+(char)(frac*10);
  message[9] = clock_reset_cause();
#endif

  loop = CONECTRIC_BURST_NUMBER;

  while(loop--) {

    packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_BOOT_PAYLOAD_SIZE);
    NETSTACK_MAC.on();
    conectric_send_to_sink(&conectric);

    PROCESS_PAUSE();

    if (loop) {
      etimer_set(&et, 1 + random_rand() % (CLOCK_SECOND / 8));
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

  }

  while(1) {

#if RUN_ON_COOJA_SIMULATION
    PROCESS_WAIT_EVENT_UNTIL((ev == PROCESS_EVENT_CONTINUE || ev == sensors_event) && data != NULL);
#else
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
#endif

    request = (uint8_t *)data;

#if RUN_ON_COOJA_SIMULATION
    if (request == &button_sensor) while(1); /* loop here until watchdog reboots */
#endif

    /* temporary workaround to test sending to sink */
    /*                                              */

    if (*request == USB_COLLECT_PERIODIC) {

      memset(hexstring, 0, sizeof(hexstring));
      strcpy(hexstring, "051A000001");
      bytereq[0] = '<';
      hexstring_to_bytereq(hexstring, &bytereq[1]);

      compose_request_to_packetbuf(bytereq, seqno++, &to);
      conectric_send_to_sink(&conectric);

      /*                                              */
      /* temporary workaround to test sending to sink */
    }
    else {

    if (*request == '<')
      compose_request_to_packetbuf(request, seqno++, &to);
    else
      compose_response_to_packetbuf(request, seqno++, &to);

    conectric_send(&conectric, &to);

    }

    PRINTF("%d.%d: conectric sent to %d.%d ts %lu\n",
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
      process_post(&usb_broadcast_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      supervisory_counter = USB_SUP_TIMEOUT;
      event = USB_SUP_EVT;
      process_post(&usb_broadcast_process, PROCESS_EVENT_CONTINUE, &event);
    }

    /* Send periodic message */
    if (periodic_counter > 1) {
      periodic_counter--;
      event = USB_COLLECT_NOEVT;
      process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      periodic_counter = USB_PERIODIC_TIMEOUT;
      event = USB_COLLECT_PERIODIC;
      process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, &event);
    }
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
      process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, event);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if RUN_ON_COOJA_SIMULATION
#else
void
invoke_process_before_sleep(void)
{
  process_post_synch(&usb_broadcast_process, PROCESS_EVENT_CONTINUE, NULL);
}
#endif
/*---------------------------------------------------------------------------*/
