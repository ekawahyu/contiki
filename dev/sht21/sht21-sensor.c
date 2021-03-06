/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 *
 * Author  : Ekawahyu Susilo <ekawahyu@yahoo.com>
 * Modified: 2017-02-01
 * Origin  : sht11-sensor.c
 *
 */

#include <stdlib.h>

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/sht21/sht21.h"
#include "dev/sht21/sht21-sensor.h"

const struct sensors_sensor sht21_sensor;

enum {
  ON, OFF
};
static uint8_t state = OFF;

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  switch(type) {

  case SHT21_SENSOR_TEMP_ACQ:
    return sht21_temp_acq();

  case SHT21_SENSOR_TEMP_RESULT:
    return sht21_temp_result();

  case SHT21_SENSOR_HUMIDITY_ACQ:
    return sht21_humidity_acq();

  case SHT21_SENSOR_HUMIDITY_RESULT:
    return sht21_humidity_result();

  case SHT21_SENSOR_BATTERY_INDICATOR:
    //return sht21_sreg() & 0x40? 1: 0;
    return 0;
}
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return (state == ON);
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  switch(type) {
  case SENSORS_ACTIVE:
    if(c) {
      if(!status(SENSORS_ACTIVE)) {
        rtimer_clock_t t0;
        sht21_init();
        state = ON;

        /* For for about 15 ms before the SHT21 can be used. */
        t0 = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + RTIMER_SECOND / 65));
        sht21_user_reg();
      }
    } else {
      sht21_off();
      state = OFF;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(sht21_sensor, "sht21",
	       value, configure, status);
