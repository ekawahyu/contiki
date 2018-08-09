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
#include "../command.h"

/* Conectric Network */
#include "../conectric-messages.h"

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
#define USB_COLLECT_SENSOR_BROADCAST  0x91
#define USB_COLLECT_PERIODIC          0x92
#define USB_COLLECT_NOEVT             0x00
#define USB_SUP_EVT                   0xBB
#define USB_SUP_NOEVT                 0x00

/* Flash Logging */
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
enum
{
  USB_RESERVED = 0x00,    // reserved
  USB_SEND     = 0x01,    // send data event
};

#define MESSAGE_LOCALBC_RECV      3
#define MESSAGE_NETBC_RECV        4
#define MESSAGE_CONECTRIC_RECV    7

static message_recv localbc_message_recv;
static message_recv netbc_message_recv;
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
  message->message_len = packetlen;

  /* Decoding payload and its length */
  hdrlen = message->message[0];
  message->seqno = message->message[1];
  message->payload = &message->message[0] + hdrlen;
  message->length = message->message[hdrlen];

  /* Update hop count */
  message->message[2] = message->hops;

  /* If destination is provided */
  if (message->message[4] || message->message[5]) {
    /* do nothing */
  }
  else {
    /* Replace destination with originator address */
    if (message->esender.u8[0] || message->esender.u8[1]) {
      message->message[4] = message->esender.u8[0];
      message->message[5] = message->esender.u8[1];
    }
    else {
      message->message[4] = message->sender.u8[0];
      message->message[5] = message->sender.u8[1];
    }
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

  len = packetbuf_datalen();
  packetbuf = (char *)message->message;
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}
/*---------------------------------------------------------------------------*/
struct conectric_conn conectric;
/*---------------------------------------------------------------------------*/
PROCESS(usb_periodic_process, "USB Periodic");
PROCESS(usb_conectric_process, "USB Conectric");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &usb_periodic_process,
    &usb_conectric_process,
//    &flash_log_process,
    &serial_in_process
);
/*---------------------------------------------------------------------------*/
static void
sent(struct conectric_conn *c)
{
  PRINTF("packet sent\n");
}

static void
recv(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  packetbuf_and_attr_copyto(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);

  dump_packetbuf(&conectric_message_recv);

  if (conectric_message_recv.request == CONECTRIC_REBOOT_REQUEST) {
    /* Halt the system right here until watchdog kicks in */
    while(1);
  }

  PRINTF("%d.%d: data from %d.%d len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

static void
localbroadcast(struct conectric_conn *c, const linkaddr_t *from)
{
  static uint8_t event;

  packetbuf_and_attr_copyto(&localbc_message_recv, MESSAGE_LOCALBC_RECV);

  /* Sensor broadcast received is dumped to serial console */
  if (conectric_is_sink()) {
    dump_packetbuf(&localbc_message_recv);
  }
  else {
    /* Sensor broadcast received is forwarded to a sink */
    if (conectric_is_collect()) {
      event = USB_COLLECT_SENSOR_BROADCAST;
      process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, &event);
    }
  }

  PRINTF("%d.%d: localbc from %d.%d: len %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen());
}

static void
netbroadcast(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  packetbuf_and_attr_copyto(&netbc_message_recv, MESSAGE_NETBC_RECV);

  dump_packetbuf(&netbc_message_recv);

  if (netbc_message_recv.request == CONECTRIC_REBOOT_REQUEST) {
    /* Halt the system right here until watchdog kicks in */
    while(1);
  }

  PRINTF("%d.%d: netbc from %d.%d: len %d hops %d\n",
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

const static struct conectric_callbacks callbacks = {
    recv, sent, localbroadcast, netbroadcast, sink
};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(usb_conectric_process, ev, data)
{
  static uint8_t hexstring[20];
  static uint8_t bytereq[20];

  static unsigned int vdd;
  static uint8_t batt;
  static uint8_t seqno = 0;
  static float sane;
  static int dec;
  static float frac;
  static struct etimer et;
  static uint8_t loop;

  static uint8_t * request;
  static linkaddr_t to;
  static linkaddr_t * which_sink;

  static request_line line;
  static uint8_t payload[20];

  PROCESS_EXITHANDLER(conectric_close(&conectric);)

  PROCESS_BEGIN();

  conectric_open(&conectric, 132, &callbacks);
  conectric_set_collect(&conectric, 1);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

//  /* Temporary request line test */
//  payload[0] = 'H';
//  payload[1] = 'e';
//  payload[2] = 'l';
//  payload[3] = 'l';
//  payload[4] = 'o';
//  to.u8[0] = 0;
//  to.u8[1] = 0;
//  request_line_create(&line, CONECTRIC_TEXT_MESSAGE, &to, payload, 5);
//  compose_request_line_to_packetbuf(&line, seqno++, batt, &to);
//  conectric_send(&conectric, &to);
//  putstring("Hello sent\n");
//
//  payload[0] = 2;
//  payload[1] = 2;
//  payload[2] = 0;
//  to.u8[0] = 0;
//  to.u8[1] = 0;
//  request_line_create(&line, CONECTRIC_RS485_CONFIG, &to, payload, 3);
//  compose_request_line_to_packetbuf(&line, seqno++, batt, &to);
//  conectric_send(&conectric, &to);
//  putstring("RS485 config sent\n");

#if RUN_ON_COOJA_SIMULATION
  SENSORS_ACTIVATE(button_sensor);
#endif

  while(1) {

#if RUN_ON_COOJA_SIMULATION
    PROCESS_WAIT_EVENT_UNTIL((ev == PROCESS_EVENT_CONTINUE || ev == sensors_event) && data != NULL);
    vdd = 0;
#else
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
    vdd = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
#endif
    sane = vdd * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;
    batt = (uint8_t)(dec*10)+(uint8_t)(frac*10);

    request = (uint8_t *)data;

    if(*request == USB_COLLECT_SENSOR_BROADCAST)
    {
      memset(message, 0, sizeof(message));
      message[0] = USB_HEADER_SIZE;
      message[1] = localbc_message_recv.seqno;
      message[2] = 0;
      message[3] = 0;
      message[4] = localbc_message_recv.sender.u8[0];
      message[5] = localbc_message_recv.sender.u8[1];
      for (loop = 0;loop < localbc_message_recv.length; loop++) {
        message[6+loop] = localbc_message_recv.payload[loop];
      }

      etimer_set(&et, CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8));
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      packetbuf_copyfrom(message, USB_HEADER_SIZE + localbc_message_recv.length);
      NETSTACK_MAC.on();
      which_sink = conectric_send_to_sink(&conectric);
      if (which_sink == NULL) PRINTF("%d.%d: which_sink returns NULL\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
      PRINTF("%d.%d: sensor broadcast sent to sink ts %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          clock_seconds());
      // PROCESS_PAUSE();
    }

    else if(*request == USB_SUP_EVT)
    {
      uint16_t time = clock_seconds();
      memset(message, 0, sizeof(message));
      message[0] = USB_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = USB_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SUPERVISORY_REPORT;
      message[8] = batt;
      message[9] = (uint8_t)(time >> 6);
      loop = CONECTRIC_BURST_NUMBER;
      while(loop--) {
        packetbuf_copyfrom(message, USB_HEADER_SIZE + USB_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        which_sink = conectric_send_to_sink(&conectric);
        PRINTF("%d.%d: conectric sent to sink ts %lu\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            clock_seconds());
        // PROCESS_PAUSE();
        if (loop) {
          etimer_set(&et, CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
      }
    }

    else if(*request == USB_COLLECT_NOEVT || *request == USB_SUP_NOEVT)
    {
      /* do nothing, periodic counter updates */
    }

#if RUN_ON_COOJA_SIMULATION
    else if (request == &button_sensor)
      while(1); /* loop here until watchdog reboots */
#endif

    else if (*request == '<')
    {
      compose_request_to_packetbuf(request, seqno++, batt, &to);
      conectric_send(&conectric, &to);

      PRINTF("%d.%d: conectric sent to %d.%d ts %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          to.u8[0], to.u8[1], clock_seconds());
    }

    else
    {
      /* everything else just silently fails for now */
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(usb_periodic_process, ev, data)
{
  static struct etimer et;
  static uint8_t event;
  static int16_t supervisory_counter;
  static int16_t periodic_counter;

  PROCESS_BEGIN();

  supervisory_counter = USB_SUP_TIMEOUT;

  while (1)
  {
    etimer_set(&et, 58 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send supervisory message */
    if (supervisory_counter > 1) {
      supervisory_counter--;
      event = USB_SUP_NOEVT;
    }
    else {
      supervisory_counter = USB_SUP_TIMEOUT;
      event = USB_SUP_EVT;
    }
    process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, &event);

    etimer_set(&et, 2 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send periodic message */
    if (periodic_counter > 1) {
      periodic_counter--;
      event = USB_COLLECT_NOEVT;
    }
    else {
      periodic_counter = USB_PERIODIC_TIMEOUT;
      event = USB_COLLECT_PERIODIC;
    }
    process_post(&usb_conectric_process, PROCESS_EVENT_CONTINUE, &event);
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
      PRINTF("Command: %i\n", event[2]);
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

}
#endif
/*---------------------------------------------------------------------------*/
