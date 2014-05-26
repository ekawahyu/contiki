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
#include "dev/ad7689-sensor.h"
#include "dev/lpm.h"

#include <stdio.h>

#define DEBUG 1

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

static char message[MESSAGE_LEN];
static char button_pressed = 0;
static char command_received = 0;

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
PROCESS(serial_in_process, "ABC example");
/*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process);
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
}
static void
abc_sent_cb(struct abc_conn *c, int status, int num_tx)
{
  PRINTF("abc message sent\n");
}
static const struct abc_callbacks abc_call = {abc_recv_cb, abc_sent_cb};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static const struct sensors_sensor *sensor;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {
    /* Delay ~1 second */
    etimer_set(&et, CLOCK_SECOND/128);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    sensor = sensors_find(LSM330DLC_SENSOR);

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
    }

    /*sensor = sensors_find(AD7689_SENSOR);

    if (sensor) {
      printf("AD7689 Channel0: %u\n",
                sensor->value(AD7689_SENSOR_CHANNEL0));
    }
    else printf("no sensor detected\n");*/
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s\n", (char*)data);
  }

  PROCESS_END();
}
