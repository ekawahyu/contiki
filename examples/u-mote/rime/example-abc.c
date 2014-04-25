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

#include "dev/lsm330dlc-sensor.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
AUTOSTART_PROCESSES(&example_abc_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  printf("abc message received '%s'\n", (char *)packetbuf_dataptr());
  //process_post_synch(&example_abc_process, PROCESS_EVENT_CONTINUE, "wakeup");
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static int counter;
  static const struct sensors_sensor *sensor;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {

    /* Delay 2-4 seconds */
    //etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));
    etimer_set(&et, 4);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    //PROCESS_WAIT_EVENT_UNTIL(data=="wakeup");

    packetbuf_copyfrom("Hello from EKA:A", 17);
    abc_send(&abc);
    printf("abc message sent (%i)\n", counter++);

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
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
