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
 *         Conectric Motion Detection Sensor (initially taken from broadcast example)
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Ekawahyu Susilo <ekawahyu.susilo@conectric.com>
 */

/* General */
#include <stdio.h>

/* Contiki */
#include "contiki.h"
#include "net/rime/rime.h"
#include "net/netstack.h"
#include "random.h"

/* Conectric Device */
//#include "flash-logging.h"
#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/leds.h"

/* Conectric Network */
#include "../conectric-messages.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* OC Network Parameters */
#define OC_SUP_TIMEOUT         180 /* minutes */
#define OC_HEADER_SIZE         6
#define OC_BOOT_PAYLOAD_SIZE   4
#define OC_PAYLOAD_SIZE        4
static uint8_t message[CONECTRIC_MESSAGE_LENGTH];
extern volatile uint16_t deep_sleep_requested;

/* OC Device Parameters */
#define OC_EVT            0x81
#define OC_SUP_EVT        0xBB
#define OC_SUP_NOEVT      0x00

/* Flash Logging */
//static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
//#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
//enum
//{
//  OC_RESERVED = 0x00,    // reserved
//  OC_SEND     = 0x01,    // send data event
//};

/*---------------------------------------------------------------------------*/
PROCESS(oc_broadcast_process, "OC Sensor");
PROCESS(oc_supervisory_process, "OC Supervisory");
//PROCESS(flash_log_process, "Flash Log");
#if BUTTON_SENSOR_ON
PROCESS(oc_interrupt_process, "OC Interrupt");
AUTOSTART_PROCESSES(&oc_broadcast_process, &oc_supervisory_process, &oc_interrupt_process/*, &flash_log_process*/);
#else
AUTOSTART_PROCESSES(&oc_broadcast_process, &oc_supervisory_process/*, &flash_log_process*/);
#endif
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  PRINTF("broadcast received from %d.%d (%d) '%s'\n",
      from->u8[0], from->u8[1], strlen(message), message);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(oc_broadcast_process, ev, data)
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

  broadcast_open(&broadcast, 132, &broadcast_call);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  /* Composing boot status message */
  memset(message, 0, sizeof(message));
  message[0] = OC_HEADER_SIZE;
  message[1] = seqno++;
  message[2] = 0;
  message[3] = 0;
  message[4] = 0;
  message[5] = 0;
  message[6] = OC_BOOT_PAYLOAD_SIZE;
  message[7] = CONECTRIC_DEVICE_BROADCAST_BOOT_STATUS;
  batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
  sane = batt * 3 * 1.15 / 2047;
  dec = sane;
  frac = sane - dec;
  message[8] = (char)(dec*10)+(char)(frac*10);
  message[9] = clock_reset_cause();

  loop = CONECTRIC_BURST_NUMBER;

  while(loop--) {

    packetbuf_copyfrom(message, OC_HEADER_SIZE + OC_BOOT_PAYLOAD_SIZE);
    NETSTACK_MAC.on();
    broadcast_send(&broadcast);

    PROCESS_WAIT_EVENT();

    if (loop)
      deep_sleep_requested = CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8);
    else
      deep_sleep_requested = 60 * CLOCK_SECOND;
  }

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
    NETSTACK_MAC.off(0);
    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;

    sensor_data = (uint8_t*)data;

    if(*sensor_data == OC_EVT)
    {
      /* Composing OC sensor message */
      memset(message, 0, sizeof(message));
      message[0] = OC_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = OC_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SENSOR_BROADCAST_OC;
      message[8] = (char)(dec*10)+(char)(frac*10);
      message[9] = *sensor_data;

//      /* Log data that will be sent out over the air */
//      logData[0] = (char)(dec*10)+(char)(frac*10);
//      logData[1] = *sensor_data;
//      logData[2] = 0x00;
//      logData[3] = 0x00;
//      flashlogging_write4(RIME_OC_CMP_ID, OC_SEND, logData);
    
      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, OC_HEADER_SIZE + OC_PAYLOAD_SIZE);

        NETSTACK_MAC.on();
        leds_on(LEDS_RED);
        broadcast_send(&broadcast);
        leds_off(LEDS_RED);

        PROCESS_WAIT_EVENT();

        if (loop)
          deep_sleep_requested = CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8);
        else
          deep_sleep_requested = 60 * CLOCK_SECOND;
      }
    }
    else if(*sensor_data == OC_SUP_EVT)
    {
      uint16_t time = clock_seconds();
      
      memset(message, 0, sizeof(message));
      message[0] = OC_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = 0;
      message[3] = 0;
      message[4] = 0;
      message[5] = 0;
      message[6] = OC_PAYLOAD_SIZE;
      message[7] = CONECTRIC_SUPERVISORY_REPORT;
      message[8] = (char)(dec*10)+(char)(frac*10);
      message[9] = (char)(time >> 6);
    
      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, OC_HEADER_SIZE + OC_PAYLOAD_SIZE);
        NETSTACK_MAC.on();
        broadcast_send(&broadcast);

        PROCESS_WAIT_EVENT();

        if (loop)
          deep_sleep_requested = CLOCK_SECOND / 8 + random_rand() % (CLOCK_SECOND / 8);
        else
          deep_sleep_requested = 60 * CLOCK_SECOND;
      }
    }
    else if(*sensor_data == OC_SUP_NOEVT) {
      deep_sleep_requested = 60 * CLOCK_SECOND;
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(oc_interrupt_process, ev, data)
{
  struct sensors_sensor *sensor;
  static uint8_t counter;
  static uint8_t button;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_1_sensor) {
      button = OC_EVT;
      process_post(&oc_broadcast_process, PROCESS_EVENT_CONTINUE, &button);
    }
    if(sensor == &button_2_sensor) {
      button = OC_EVT;
      process_post(&oc_broadcast_process, PROCESS_EVENT_CONTINUE, &button);
    }
  }

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(oc_supervisory_process, ev, data)
{
  static struct etimer et;
  static uint8_t event;
  static int16_t supervisory_counter;

  PROCESS_BEGIN();

  supervisory_counter = OC_SUP_TIMEOUT;

  while (1)
  {
    etimer_set(&et, 60 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send supervisory message */
    if (supervisory_counter > 1) {
      supervisory_counter--;
      event = OC_SUP_NOEVT;
      process_post(&oc_broadcast_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      supervisory_counter = OC_SUP_TIMEOUT;
      event = OC_SUP_EVT;
      process_post(&oc_broadcast_process, PROCESS_EVENT_CONTINUE, &event);
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
void
invoke_process_before_sleep(void)
{
  process_post_synch(&oc_broadcast_process, PROCESS_EVENT_CONTINUE, NULL);
}
/*---------------------------------------------------------------------------*/
