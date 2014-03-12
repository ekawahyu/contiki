/*
 * lsm330dlc-sensor.c
 *
 * Created on: Mar 11, 2014
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2014, Chongqing Aisenke Electronic Technology Co., Ltd.
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

#include "dev/lsm330dlc-sensor.h"

static int
value(int type)
{
  switch(type) {
  case LSM330DLC_SENSOR_TYPE_ACCL_X:
    return 1;
  case LSM330DLC_SENSOR_TYPE_ACCL_Y:
    return 1;
  case LSM330DLC_SENSOR_TYPE_ACCL_Z:
    return 1;
  case LSM330DLC_SENSOR_TYPE_GYRO_X:
    return 1;
  case LSM330DLC_SENSOR_TYPE_GYRO_Y:
    return 1;
  case LSM330DLC_SENSOR_TYPE_GYRO_Z:
    return 1;
  }
  return 0;
}

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    /* init sensor here */
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      /* active sensor */
    }
    else {
      /* disable/put into sleep */
    }
    return 1;
  }
  return 0;
}

static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return 1 /* return sensor active flag */;
  }
  return 0;
}

SENSORS_SENSOR(lsm330dlc_sensor, LSM330DLC_SENSOR, value, configure, status);
