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
 *         Testing the abc layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "sys/clock.h"
#include "dev/serial-line.h"
#include "dev/lsm330dlc-sensor.h"
#include "dev/button-sensor.h"
#include "dev/gpio.h"
#include "dev/leds.h"
#include "dev/lpm.h"

#include <stdio.h>

#define DEBUG 1

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

#define MESSAGE_LEN   30
static char message[MESSAGE_LEN];
static char button_pressed = 0;
static char command_received = 0;

#define FIRE_COILS    1
#define RELOAD_COILS  2

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
PROCESS(serial_in_process, "ABC example");
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS(buttons_process, "Button Process");
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process, &buttons_process);
#else
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process);
#endif
/*---------------------------------------------------------------------------*/
static void
abc_recv_cb(struct abc_conn *c)
{
  static clock_time_t curr_time;
  static clock_time_t time_diff;
  static unsigned int repeat;

  memset(message, 0, MESSAGE_LEN);
  memcpy(message, (char *)packetbuf_dataptr(), packetbuf_datalen());
  PRINTF("abc message received '%s'\n", message);
#if (LPM_MODE == 0)
  if (memcmp(message,"I am awake", 10) == 0) {
    time_diff = clock_time() - curr_time;
    curr_time = clock_time();
    process_post_synch(&example_abc_process, PROCESS_EVENT_CONTINUE, &time_diff);
  }
#else
  if (memcmp(message,"fire coils", 10) == 0) {
    PRINTF("Firing now...\n");
    gpio_clear(GPIO2 | GPIO4);
    gpio_set(GPIO1 | GPIO3 | GPIO5);
    repeat = 13;
    while (repeat--) clock_delay_usec(65000);
    gpio_clear(GPIO2 | GPIO4);
    gpio_clear(GPIO1 | GPIO3 | GPIO5);
    PRINTF("Firing done!\n");
  }
  if (memcmp(message,"reload coils", 12) == 0) {
    PRINTF("Reloading now...\n");
    gpio_clear(GPIO1 | GPIO3);
    gpio_set(GPIO2 | GPIO4 | GPIO5);
    repeat = 13;
    while (repeat--) clock_delay_usec(65000);
    gpio_clear(GPIO2 | GPIO4);
    gpio_clear(GPIO1 | GPIO3 | GPIO5);
    PRINTF("Reloading done!\n");
  }
#endif /* (LPM_MODE == 0) */
}
static void
abc_sent_cb(struct abc_conn *c, int status, int num_tx)
{
  PRINTF("abc message sent\n");
#if LPM_MODE
  /* Listening delay for any incoming message */
  clock_delay_usec(10000);
#endif /* LPM_MODE */
}
static const struct abc_callbacks abc_call = {abc_recv_cb, abc_sent_cb};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static int counter = 0;
  static const struct sensors_sensor *sensor;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {
#if LPM_MODE
    /* Delay 1 second */
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    memset(message, 0, MESSAGE_LEN);
    sprintf(message, "I am awake:%i", counter++);
    packetbuf_copyfrom(message, strlen(message));
    abc_send(&abc);
#else
    /* example of bursting abc messages to sleepy u-mote */
    /*PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_CONTINUE);
    printf("time_diff %u\n", *(int*)data);

    etimer_set(&et, *(int*)data - 10);
    counter = 10;
    while (counter) {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      etimer_set(&et, 2);

      memset(message, 0, MESSAGE_LEN);
      sprintf(message, "Hello from EKA:%i", counter--);
      packetbuf_copyfrom(message, strlen(message));
      abc_send(&abc);
    }*/
    /* example of direct reply to sleepy u-mote */
    PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_CONTINUE);

    memset(message, 0, MESSAGE_LEN);

    if (button_pressed || command_received == FIRE_COILS) sprintf(message, "fire coils:%i", 1);
    if (command_received == RELOAD_COILS) sprintf(message, "reload coils:%i", 1);

    packetbuf_copyfrom(message, strlen(message));

    if (button_pressed || command_received) {
      button_pressed = 0;
      command_received = 0;
      abc_send(&abc);
    }
#endif /* LPM_MODE */

    /*sensor = sensors_find(LSM330DLC_SENSOR);

    if (sensor) {
      printf("LSM330DLC Status: 0x%X\n",
          sensor->value(LSM330DLC_SENSOR_TYPE_GYRO_STATUS));

      printf("LSM330DLC ID: 0x%X\n",
          sensor->value(LSM330DLC_SENSOR_TYPE_ID));

      printf("LSM330DLC temp: 0x%X\n",
          sensor->value(LSM330DLC_SENSOR_TYPE_GYRO_TEMP));

      printf("LSM330DLC GX: %i GY: %i GZ: %i\n",
          sensor->value(LSM330DLC_SENSOR_TYPE_GYRO_X),
          sensor->value(LSM330DLC_SENSOR_TYPE_GYRO_Y),
          sensor->value(LSM330DLC_SENSOR_TYPE_GYRO_Z));

      printf("LSM330DLC AX: %i AY: %i AZ: %i\n",
          sensor->value(LSM330DLC_SENSOR_TYPE_ACCL_X),
          sensor->value(LSM330DLC_SENSOR_TYPE_ACCL_Y),
          sensor->value(LSM330DLC_SENSOR_TYPE_ACCL_Z));
    }*/
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_process, ev, data)
{
  struct sensors_sensor *sensor;
  static struct etimer et;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    /* If we woke up after a sensor event, inform what happened */
    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_sensor) {
      PRINTF("Button Press\n");
      leds_toggle(LEDS_GREEN);
      button_pressed = 1;
    }
  }

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s\n", (char*)data);
    if (memcmp(data,"fire coils", 10) == 0) command_received = FIRE_COILS;
    if (memcmp(data,"reload coils", 12) == 0) command_received = RELOAD_COILS;
  }

  PROCESS_END();
}
