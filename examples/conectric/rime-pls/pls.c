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
 *         Conectric Pulse Sensor (initially taken from abc example)
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
#include "flash-logging.h"
#include "dev/button-sensor.h"
#include "dev/sht21/sht21-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/rs485-arch.h"

/* Conectric Network */
#include "examples/conectric/conectric-messages.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* PLS Network Parameters */
#define PLS_SUP_TIMEOUT         180 /* minutes */
#define PLS_PERIODIC_TIMEOUT    1   /* minutes */
#define PLS_HEADER_SIZE         2
#define PLS_PAYLOAD_SIZE        4
static uint8_t message[CONECTRIC_MESSAGE_LENGTH];
extern volatile uint16_t deep_sleep_requested;

/* PLS Device Parameters */
#define PLS_PULSE_DETECTED       0x91
#define PLS_PULSE_PERIODIC       0x92
#define PLS_PULSE_TOTAL          0x93
#define PLS_PULSE_NOEVT          0x00
#define PLS_SUP_EVT              0xBB
#define PLS_SUP_NOEVT            0x00

static uint32_t pls_pulse_counter;

/* Flash Logging */
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
enum
{
  PLS_RESERVED = 0x00,    // reserved
  PLS_SEND     = 0x01,    // send data event
};

/*---------------------------------------------------------------------------*/
PROCESS(pls_abc_process, "PLS Sensor");
PROCESS(pls_supervisory_process, "PLS Supervisory");
//PROCESS(flash_log_process, "Flash Log");
#if BUTTON_SENSOR_ON
PROCESS(pls_interrupt_process, "PLS Interrupt");
AUTOSTART_PROCESSES(&pls_abc_process, &pls_supervisory_process, &pls_interrupt_process/*, &flash_log_process*/);
#else
AUTOSTART_PROCESSES(&pls_abc_process, &pls_supervisory_process/*, &flash_log_process*/);
#endif
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  memset(message, 0, sizeof(message));
  memcpy(message, (char *)packetbuf_dataptr(), packetbuf_datalen());
  PRINTF("abc message received (%d) '%s'\n", strlen(message), message);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pls_abc_process, ev, data)
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
  batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
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

  BUTTON_SENSOR_DEACTIVATE(1);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;

    sensor_data = (uint8_t*)data;

    if(*sensor_data == PLS_PULSE_DETECTED)
    {
      pls_pulse_counter++;
#if LPM_CONF_MODE
      deep_sleep_requested = 60 * CLOCK_SECOND;
#endif
    }
    else if(*sensor_data == PLS_PULSE_PERIODIC)
    {
      memset(message, 0, sizeof(message));
      message[0] = PLS_HEADER_SIZE;
      message[1] = seqno++;
      message[2] = PLS_PAYLOAD_SIZE + 4;
      message[3] = CONECTRIC_SENSOR_BROADCAST_PLS;
      message[4] = (char)(dec*10)+(char)(frac*10);
      message[5] = (char)PLS_PULSE_PERIODIC;
      message[6] = (char)(pls_pulse_counter >> 24);
      message[7] = (char)((pls_pulse_counter & 0x00FFFFFF) >> 16);
      message[8] = (char)((pls_pulse_counter & 0x0000FFFF) >>  8);
      message[9] = (char) (pls_pulse_counter & 0x000000FF);

      pls_pulse_counter = 0;

      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, PLS_HEADER_SIZE + PLS_PAYLOAD_SIZE + 4);
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
    else if(*sensor_data == PLS_SUP_EVT)
    {
      uint16_t time = clock_seconds();
      
      memset(message, 0, sizeof(message));
      message[0] = PLS_HEADER_SIZE;                      // Header Length
      message[1] = seqno++;                             // Sequence number
      message[2] = PLS_PAYLOAD_SIZE;                     // Payload Length
      message[3] = CONECTRIC_SUPERVISORY_REPORT;        // Payload Type                    
      message[4] = (char)(dec*10)+(char)(frac*10);      // battery
      message[5] = (char)(time >> 6);                   // time (rough min)
    
      loop = CONECTRIC_BURST_NUMBER;

      while(loop--) {
        packetbuf_copyfrom(message, PLS_HEADER_SIZE + PLS_PAYLOAD_SIZE);
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
    else if(*sensor_data == PLS_PULSE_NOEVT || *sensor_data == PLS_SUP_NOEVT) {
#if LPM_CONF_MODE
      deep_sleep_requested = 60 * CLOCK_SECOND;
#endif
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(pls_interrupt_process, ev, data)
{
  struct sensors_sensor *sensor;
  static uint8_t counter;
  static uint8_t button;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_1_sensor) {
      button = PLS_PULSE_DETECTED;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &button);
    }
    if(sensor == &button_2_sensor) {
      button = PLS_PULSE_DETECTED;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &button);
    }
  }

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pls_supervisory_process, ev, data)
{
  static struct etimer et;
  static uint8_t event;
  static int16_t supervisory_counter;
  static int16_t periodic_counter;

  PROCESS_BEGIN();

  supervisory_counter = PLS_SUP_TIMEOUT;
  periodic_counter = PLS_PERIODIC_TIMEOUT;

  while (1)
  {
    etimer_set(&et, 60 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send supervisory message */
    if (supervisory_counter > 1) {
      supervisory_counter--;
      event = PLS_SUP_NOEVT;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      supervisory_counter = PLS_SUP_TIMEOUT;
      event = PLS_SUP_EVT;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &event);
    }

    /* Send periodic message */
    if (periodic_counter > 1) {
      periodic_counter--;
      event = PLS_PULSE_NOEVT;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &event);
    }
    else {
      periodic_counter = PLS_PERIODIC_TIMEOUT;
      event = PLS_PULSE_PERIODIC;
      process_post(&pls_abc_process, PROCESS_EVENT_CONTINUE, &event);
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
  process_post_synch(&pls_abc_process, PROCESS_EVENT_CONTINUE, NULL);
}
/*---------------------------------------------------------------------------*/