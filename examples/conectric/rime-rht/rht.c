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
 *         Conectric SHT21/20 Sensor Node (initially taken from abc example)
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

/* Conectric Network */
#include "examples/conectric/conectric-messages.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* RHT Network Parameters */
#define RHT_REPORTING_PERIOD    (59U * CLOCK_SECOND)
#define RHT_HEADER_SIZE         2
#define RHT_PAYLOAD_SIZE        7
static uint8_t message[RHT_HEADER_SIZE + RHT_PAYLOAD_SIZE];
extern volatile uint16_t deep_sleep_requested;

/* Flash Logging */
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Logging reference time every 12 hours */
#define LOGGING_REF_TIME_PD ((clock_time_t)(12U * CLOCK_SECOND * 60U * 60U))
enum
{
  RHT_RESERVED = 0x00,    // reserved
  RHT_SEND     = 0x01,    // send data event
};

/*---------------------------------------------------------------------------*/
PROCESS(rht_abc_process, "RHT Sensor");
//PROCESS(flash_log_process, "Flash Log");
AUTOSTART_PROCESSES(&rht_abc_process/*, &flash_log_process*/);
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
PROCESS_THREAD(rht_abc_process, ev, data)
{
  static uint8_t * header = NULL;
  static uint8_t seqno = 0;
  static unsigned int batt, temp, humid;
  static unsigned int prev_temp, prev_humid;
  static uint8_t counter;
  static float sane;
  static int dec;
  static float frac;
  static struct etimer et;
  static uint8_t loop;

  static clock_time_t clock_now;
  static unsigned long clock_sec;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(sht21_sensor);

  abc_open(&abc, 128, &abc_call);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  /* Composing boot status message */
  memset(message, 0, 2 + 4);
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

    if (loop)
      deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
    else
      deep_sleep_requested = CLOCK_SECOND;
  }

  while(1) {

    PROCESS_WAIT_EVENT();

    /* Battery and temperature acquisition */
    NETSTACK_MAC.off(0);
    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;
    temp = sht21_sensor.value(SHT21_SENSOR_TEMP_ACQ);
    deep_sleep_requested = CLOCK_SECOND / 32;

    PROCESS_WAIT_EVENT();

    /* Humidity acquisition */
    NETSTACK_MAC.off(0);
    prev_temp = temp;
    temp = sht21_sensor.value(SHT21_SENSOR_TEMP_RESULT);
    humid = sht21_sensor.value(SHT21_SENSOR_HUMIDITY_ACQ);
    deep_sleep_requested = CLOCK_SECOND / 32;

    PROCESS_WAIT_EVENT();

    clock_now = clock_time();
    clock_sec = clock_seconds();

    NETSTACK_MAC.off(0);
    prev_humid = humid;
    humid = sht21_sensor.value(SHT21_SENSOR_HUMIDITY_RESULT);

    /* Composing RHT sensor message */
    memset(message, 0, RHT_HEADER_SIZE + RHT_PAYLOAD_SIZE);
    message[0] = RHT_HEADER_SIZE;
    message[1] = seqno++;
    message[2] = RHT_PAYLOAD_SIZE;
    message[3] = CONECTRIC_SENSOR_BROADCAST_RHT;
    message[4] = (char)(dec*10)+(char)(frac*10);
    message[5] = (char)(temp >> 8);
    message[6] = (char)(temp & 0xFC);
    message[7] = (char)(humid >> 8);
    message[8] = (char)(humid & 0xFC);

    loop = CONECTRIC_BURST_NUMBER;

//    /* Log data that will be sent out over the air */
//    logData[0] = (char)(humid >> 8);
//    logData[1] = (char)(humid & 0xFC);
//    logData[2] = (char)(temp >> 8);
//    logData[3] = (char)(temp & 0xFC);
//    flashlogging_write4(RIME_RHT_CMP_ID, RHT_SEND, logData);

    while(loop--) {
      
      packetbuf_copyfrom(message, RHT_HEADER_SIZE + RHT_PAYLOAD_SIZE);
              
      NETSTACK_MAC.on();
      abc_send(&abc);

      PROCESS_WAIT_EVENT();

      if (loop)
        deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
      else
        deep_sleep_requested = RHT_REPORTING_PERIOD;
    }

  }

  SENSORS_DEACTIVATE(sht21_sensor);

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
  process_post_synch(&rht_abc_process, PROCESS_EVENT_CONTINUE, NULL);
}
/*---------------------------------------------------------------------------*/
