/*
 * rs485-router.c
 *
 * Created on: Apr 02, 2018
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
// #include "flash-logging.h"
#include "flash-state.h"
#include "ota.h"
#include "dev/adc-sensor.h"
#include "dev/rs485-arch.h"
#include "dev/uart-arch.h"
#include "dev/modbus-line.h"
#include "dev/modbus-crc16.h"
#include "dev/serial-line.h"

/* Conectric Network */
#include "../command.h"
#include "../conectric-messages.h"

#include "config.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DEBUG_CRC16 0

#define CONECTRIC_DEVICE_TYPE_RS485 0x90
uint16_t ota_img_version = OTA_NO_IMG;
uint16_t rs485_img_version = 0;

/* RS485 Network Parameters */
#define RS485_SUP_TIMEOUT         180 /* minutes */
#define RS485_PERIODIC_TIMEOUT    1   /* minutes */
#define RS485_HEADER_SIZE         6
#define RS485_BOOT_PAYLOAD_SIZE   4
#define RS485_PAYLOAD_SIZE        4
static uint8_t message[CONECTRIC_MESSAGE_LENGTH];
extern volatile uint16_t deep_sleep_requested;

/* RS485 Device Parameters */
#define RS485_COLLECT_SENSOR_BROADCAST  0x91
#define RS485_COLLECT_PERIODIC          0x92
#define RS485_INCOMING_RESPONSE         0x93
#define RS485_POLL_CHUNK                0x94
#define RS485_COLLECT_NOEVT             0x00
#define RS485_SUP_EVT                   0xBB
#define RS485_SUP_NOEVT                 0x00

/* Flash Logging */
//static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
//#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
//enum
//{
//  RS485_RESERVED = 0x00,    // reserved
//  RS485_SEND     = 0x01,    // send data event
//};

#define MESSAGE_LOCALBC_RECV      3
#define MESSAGE_NETBC_RECV        4
#define MESSAGE_CONECTRIC_RECV    7

static message_recv localbc_message_recv;
static message_recv netbc_message_recv;
static message_recv conectric_message_recv;

static uint8_t dump_header = 0;

/* RS485 Buffer */
#define BUFSIZE 256
static uint16_t rs485_in_pos;
static uint8_t rs485_data[BUFSIZE];

/* RS485 Messaging */
#define RS485_DATA_MAX_SIZE 20

/*---------------------------------------------------------------------------*/
static void
ota_img_version_restore()
{
  uint8_t *data;
  uint8_t size;
  // size = flashstate_read(FLASH_STATE_OTA_IMG_VERSION, &data);
  if(size == 2)
  {
    memcpy(&ota_img_version, data, size);
    ota_restore_map();
  }
}
/*---------------------------------------------------------------------------*/
static void
process_img_update(uint8_t * payload)
{
   static uint8_t img_segment[OTA_SEGMENT_MAX];
   uint16_t img_version;
   uint16_t addr_offset;
   uint8_t dev_type;
   uint16_t crc16, crc16_result;
   uint8_t data_len;

   img_version = (payload[3] << 8) + payload[4];
   dev_type = payload[5];
   crc16 = (payload[6] << 8) + payload[7];
   addr_offset = (payload[8] << 8) + payload[9];
   data_len = payload[10];

   // exit if not for me
   if(dev_type != CONECTRIC_DEVICE_TYPE_RS485)
     return;

   // exit if old version
   if(img_version <= rs485_img_version)
     return;

   // BMB: Not implemented
   // crc16_result = crc16_data(&payload[13], data_len, crc16_result);

   // validate checksum (including data_len): BMB - set to FF for test, need to implement checksum
   if(crc16 != 0xFFFF || data_len > OTA_SEGMENT_MAX)
     return;

   if((ota_img_version == OTA_NO_IMG && img_version > rs485_img_version) || (img_version > ota_img_version))
   { // first image segment of a new image

     // clear flash
     ota_clear();
     // save img version in state variables
     // flashstate_write(FLASH_STATE_OTA_IMG_VERSION, (uint8_t*)&img_version, sizeof(img_version));
     ota_img_version = img_version;
   }
   else if(img_version != ota_img_version)
   {
     // old image
     return;
   }

   // copy data out of payload in case it gets overwritten by another process
   memcpy(img_segment, &payload[11], data_len);
   // write segment of current image
   ota_flashwrite(addr_offset, data_len, img_segment);
   // update ota image map
   ota_update_map(addr_offset);
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
PROCESS(rs485_periodic_process, "RS485 Periodic");
PROCESS(rs485_conectric_process, "RS485 Conectric");
PROCESS(modbus_in_process, "ModbusIn");
PROCESS(modbus_out_process, "ModbusOut");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(
    &rs485_periodic_process,
    &rs485_conectric_process,
//    &flash_log_process,
    &modbus_in_process,
    &modbus_out_process,
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
  static uint8_t event;
  uint8_t rs485p[CONFIG_RS485_PARAMS_LENGTH];

  packetbuf_and_attr_copyto(&conectric_message_recv, MESSAGE_CONECTRIC_RECV);

#if DEBUG
  dump_packetbuf(&conectric_message_recv);
#endif

  if (conectric_message_recv.request == CONECTRIC_RS485_POLL_CHUNK) {
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &conectric_message_recv);
  }

  if (conectric_message_recv.request == CONECTRIC_RS485_POLL) {
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &conectric_message_recv);
  }

  if (conectric_message_recv.request == CONECTRIC_RS485_CONFIG) {
    rs485p[3] = conectric_message_recv.payload[3];
    rs485p[2] = conectric_message_recv.payload[4];
    rs485p[1] = conectric_message_recv.payload[5];
    rs485p[0] = conectric_message_recv.payload[6];
    config_update(CONFIG_RS485_PARAMS, rs485p, CONFIG_RS485_PARAMS_LENGTH);
    uart_arch_config(rs485p[3] << 6 | rs485p[2] << 4 | rs485p[1] << 2 | rs485p[0] << 0);
    event = CONECTRIC_RS485_CONFIG;
    process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);
  }

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
      event = RS485_COLLECT_SENSOR_BROADCAST;
      process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);
    }
  }

  PRINTF("%d.%d: localbc from %d.%d: len %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen());
}

static void
netbroadcast(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  uint8_t rs485p[CONFIG_RS485_PARAMS_LENGTH];

  packetbuf_and_attr_copyto(&netbc_message_recv, MESSAGE_NETBC_RECV);

#if DEBUG
  dump_packetbuf(&netbc_message_recv);
#endif

  if (netbc_message_recv.request == CONECTRIC_RS485_POLL_CHUNK) {
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &netbc_message_recv);
  }

  if (netbc_message_recv.request == CONECTRIC_RS485_POLL) {
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &netbc_message_recv);
  }

  if (netbc_message_recv.request == CONECTRIC_IMG_UPDATE_BCST) {
    process_img_update(netbc_message_recv.payload);
  }

  if (netbc_message_recv.request == CONECTRIC_RS485_CONFIG) {
    rs485p[3] = netbc_message_recv.payload[3];
    rs485p[2] = netbc_message_recv.payload[4];
    rs485p[1] = netbc_message_recv.payload[5];
    rs485p[0] = netbc_message_recv.payload[6];
    config_update(CONFIG_RS485_PARAMS, rs485p, CONFIG_RS485_PARAMS_LENGTH);
    uart_arch_config(rs485p[3] << 6 | rs485p[2] << 4 | rs485p[1] << 2 | rs485p[0] << 0);
  }

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
static void
configure_rs485_from_flash(void)
{
  uint8_t p[CONFIG_RS485_PARAMS_LENGTH];

  config_rs485_params_read(p);

  uart_arch_config(p[3] << 6 | p[2] << 4 | p[1] << 2 | p[0] << 0);

  putstring("RS485:");
  if (p[3] << 6 == UART_B2400) putstring("2400");
  if (p[3] << 6 == UART_B4800) putstring("4800");
  if (p[3] << 6 == UART_B9600) putstring("9600");
  if (p[3] << 6 == UART_B19200) putstring("19200");
  putstring(":8");
  if (p[2] << 4 == UART_PARITY_NONE) putstring("N");
  if (p[2] << 4 == UART_PARITY_ODD) putstring("O");
  if (p[2] << 4 == UART_PARITY_EVEN) putstring("E");
  if (p[1] << 2 == UART_STOP_BIT_1) putstring("1");
  if (p[1] << 2 == UART_STOP_BIT_2) putstring("2");
  putstring(":");
  if (p[0] << 0 == UART_BITMASK_8BIT) putstring("8");
  if (p[0] << 0 == UART_BITMASK_7BIT) putstring("7");
  putstring("BMASK\n");
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rs485_conectric_process, ev, data)
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

  uint8_t chunk_number;
  uint8_t chunk_size;
  uint8_t chunk_offset;

  PROCESS_EXITHANDLER(conectric_close(&conectric);)

  PROCESS_BEGIN();

  // flashstate_init();
  ota_init();
  ota_img_version_restore();

  conectric_init();
  conectric_open(&conectric, 132, &callbacks);
  conectric_set_collect(&conectric, 1);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  /* Load RS485 parameters from the flash */
  configure_rs485_from_flash();

  /* workaround to make modbus_in_process() to start receiving messages */
  uart_arch_writeb(0);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
    vdd = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = vdd * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;
    batt = (uint8_t)(dec*10)+(uint8_t)(frac*10);

    request = (uint8_t *)data;

    if (*request == RS485_INCOMING_RESPONSE)
    {
      memset(message, 0, sizeof(message));
      message[0] = RS485_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      if (rs485_in_pos > 64) {
        message[6] = 2 + 3;
        message[7] = CONECTRIC_RS485_POLL_REPLY_IN_CHUNK;
        message[8] = batt;
        message[9] = (rs485_in_pos >> 6) + ((rs485_in_pos && 63) ? 1 : 0);
        message[10] = 0x40;
      }
      else {
        message[6] = rs485_in_pos + 3;
        message[7] = CONECTRIC_RS485_POLL_REPLY;
        message[8] = batt;
        for (loop = 0;loop < rs485_in_pos; loop++) {
          message[9+loop] = rs485_data[loop];
        }
      }

      loop = CONECTRIC_BURST_NUMBER;
      while(loop--) {
        if (rs485_in_pos > 64) {
          packetbuf_copyfrom(message, RS485_HEADER_SIZE + 2 + 3);
        }
        else {
          packetbuf_copyfrom(message, RS485_HEADER_SIZE + rs485_in_pos + 3);
        }
        NETSTACK_MAC.on();
        which_sink = conectric_send_to_sink(&conectric);
        if (which_sink == NULL) PRINTF("%d.%d: which_sink returns NULL\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        PRINTF("%d.%d: rs485 response sent to sink ts %lu\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            clock_seconds());
        PRINTF("%d.%d: esender=%d.%d\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            netbc_message_recv.esender.u8[0], netbc_message_recv.esender.u8[1]);
        // PROCESS_PAUSE();
        if (loop) {
          etimer_set(&et, CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
      }
    }

    else if (*request == RS485_POLL_CHUNK)
    {
      chunk_number = *++request;
      chunk_size = *++request;
      chunk_offset = chunk_number * chunk_size;
      memset(message, 0, sizeof(message));
      message[0] = RS485_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = chunk_size + 3;
      message[7] = CONECTRIC_RS485_POLL_CHUNK_REPLY;
      message[8] = batt;
      for (loop = 0;loop < (chunk_size); loop++) {
        message[9+loop] = rs485_data[chunk_offset+loop];
      }

      loop = CONECTRIC_BURST_NUMBER;
      while(loop--) {
        packetbuf_copyfrom(message, RS485_HEADER_SIZE + chunk_size + 3);
        NETSTACK_MAC.on();
        which_sink = conectric_send_to_sink(&conectric);
        if (which_sink == NULL) PRINTF("%d.%d: which_sink returns NULL\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        PRINTF("%d.%d: rs485 response sent to sink ts %lu\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            clock_seconds());
        PRINTF("%d.%d: esender=%d.%d\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            netbc_message_recv.esender.u8[0], netbc_message_recv.esender.u8[1]);
        // PROCESS_PAUSE();
        if (loop) {
          etimer_set(&et, CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
      }
    }

    else if (*request == CONECTRIC_RS485_CONFIG)
    {
      memset(message, 0, sizeof(message));
      message[0] = RS485_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = 3;
      message[7] = CONECTRIC_RS485_CONFIG_ACK;
      message[8] = batt;

      loop = CONECTRIC_BURST_NUMBER;
      while(loop--) {
        packetbuf_copyfrom(message, RS485_HEADER_SIZE + 3);
        NETSTACK_MAC.on();
        which_sink = conectric_send_to_sink(&conectric);
        if (which_sink == NULL) PRINTF("%d.%d: which_sink returns NULL\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        PRINTF("%d.%d: rs485 response sent to sink ts %lu\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            clock_seconds());
        PRINTF("%d.%d: esender=%d.%d\n",
            linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
            netbc_message_recv.esender.u8[0], netbc_message_recv.esender.u8[1]);
        // PROCESS_PAUSE();
        if (loop) {
          etimer_set(&et, CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8));
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        }
      }
    }

    else if(*request == RS485_COLLECT_SENSOR_BROADCAST)
    {
      memset(message, 0, sizeof(message));
      message[0] = RS485_HEADER_SIZE;
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

      packetbuf_copyfrom(message, RS485_HEADER_SIZE + localbc_message_recv.length);
      NETSTACK_MAC.on();
      which_sink = conectric_send_to_sink(&conectric);
      if (which_sink == NULL) PRINTF("%d.%d: which_sink returns NULL\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
      PRINTF("%d.%d: sensor broadcast sent to sink ts %lu\n",
          linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
          clock_seconds());
      // PROCESS_PAUSE();
    }

    else if(*request == RS485_SUP_EVT)
    {
      uint16_t time = clock_seconds();
      memset(message, 0, sizeof(message));
      message[0] = RS485_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = RS485_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SUPERVISORY_REPORT;
      message[8] = batt;
      message[9] = (uint8_t)(time >> 6);
      loop = CONECTRIC_BURST_NUMBER;
      while(loop--) {
        packetbuf_copyfrom(message, RS485_HEADER_SIZE + RS485_PAYLOAD_SIZE);
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

    else if(*request == RS485_COLLECT_NOEVT || *request == RS485_SUP_NOEVT)
    {
      /* do nothing, periodic counter updates */
    }

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
PROCESS_THREAD(rs485_periodic_process, ev, data)
{
  static struct etimer et;
  static uint8_t event;
  static int16_t supervisory_counter;
  static int16_t periodic_counter;

  PROCESS_BEGIN();

  supervisory_counter = RS485_SUP_TIMEOUT;

  while (1)
  {
    etimer_set(&et, 58 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send supervisory message */
    if (supervisory_counter > 1) {
      supervisory_counter--;
      event = RS485_SUP_NOEVT;
    }
    else {
      supervisory_counter = RS485_SUP_TIMEOUT;
      event = RS485_SUP_EVT;
    }
    process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);

    etimer_set(&et, 2 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send periodic message */
    if (periodic_counter > 1) {
      periodic_counter--;
      event = RS485_COLLECT_NOEVT;
    }
    else {
      periodic_counter = RS485_PERIODIC_TIMEOUT;
      event = RS485_COLLECT_PERIODIC;
    }
    process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);
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
      process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, event);
      PRINTF("Command: %i\n", event[2]);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_in_process, ev, data)
{
  static uint8_t event;
  static uint8_t datasize;
  static uint8_t* dataptr;
  static uint16_t crc16;
  uint8_t cnt;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);

    dataptr = &((uint8_t *)data)[1];
    datasize = ((uint8_t *)data)[0];

    /* reset modbus input index */
    rs485_in_pos = 0;

    /* Copy data into RS485 buffer */
    for(cnt = 0; cnt < datasize; cnt++)
    {
#if DEBUG
      puthex(dataptr[cnt]);
#endif
      rs485_data[rs485_in_pos++] = dataptr[cnt];
    }
#if DEBUG
    putstring("\n");
#endif

#if DEBUG_CRC16
    /* This CRC16 calculation is only used to debug the incoming modbus */
    crc16 = modbus_crc16_calc(rs485_data, datasize-2);
    putstring("crc16rep=");
    puthex(crc16 & 0x00FF);
    puthex((crc16 & 0xFF00) >> 8);
    putstring("\n");
#endif

    if (rs485_in_pos) {
      event = RS485_INCOMING_RESPONSE;
      process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_out_process, ev, data)
{
  static uint8_t event[3];
  static struct etimer et;
  static message_recv * message;
  static uint8_t * serial_data;
  static uint8_t reqlen;
  static uint8_t req;
  static uint8_t batt;
  static uint8_t len;
  uint16_t crc16;
  uint8_t chunk_number;
  uint8_t chunk_size;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    message = (message_recv *)data;
    serial_data = message->payload;
    reqlen = *serial_data++;
    req = *serial_data++;
    batt = *serial_data++;

#if DEBUG_CRC16
    /* This CRC16 calculation is only used to debug the outgoing modbus */
    crc16 = modbus_crc16_calc(serial_data, len-2);
    putstring("crc16req=");
    puthex(crc16 & 0x00FF);
    puthex((crc16 & 0xFF00) >> 8);
    putstring("\n");
#endif

    if (req == CONECTRIC_RS485_POLL) {
      /* reset modbus input index */
      rs485_in_pos = 0;

      /* modbus write */
      len = reqlen - 3;
      serial_data = message->payload + 3;
      while(len--) uart_arch_writeb(*serial_data++);

#if DEBUG
      len = reqlen - 3;
      serial_data = message->payload + 3;
      while(len--) puthex(*serial_data++);
      putstring("\n");
#endif
    }
    else if (req == CONECTRIC_RS485_POLL_CHUNK) {
      chunk_number = *serial_data++;
      chunk_size = *serial_data++;
      if (rs485_in_pos) {
        event[0] = RS485_POLL_CHUNK;
        event[1] = chunk_number;
        event[2] = chunk_size;
        process_post(&rs485_conectric_process, PROCESS_EVENT_CONTINUE, &event);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{

}
/*---------------------------------------------------------------------------*/
