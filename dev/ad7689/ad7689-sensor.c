/*
 * ad7689-sensor.c
 *
 * Created on: May 22, 2014
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

#include "dev/spi.h"
#include "ad7689-sensor.h"

static int ad7689_sensor_status;

static int
value(int type)
{
  unsigned char read_byte_high;
  unsigned char read_byte_low;
  int sensor_value;
  uint16_t config;

  switch(type) {
  case AD7689_SENSOR_CHANNEL0:

    config = (AD7689_CFG_UPDATE | INPUT_UNIPOLAR_GND |
          INPUT_SELECT_IN7 | FULL_BANDWIDTH |
          EXT_REF_NO_TEMPERATURE | SCAN_IN0_INX |
          DO_NOT_READBACK_CONFIG);

    spi_select(AD7689_CS);
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    config = (INPUT_UNIPOLAR_GND | INPUT_SELECT_IN7 |
          FULL_BANDWIDTH | EXT_REF_NO_TEMPERATURE |
          SCAN_IN0_INX | DO_NOT_READBACK_CONFIG);

    spi_select(AD7689_CS);// throw away
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//0
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//1
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//2
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//3
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//4
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//5
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//6
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    spi_select(AD7689_CS);//7
    read_byte_high = spi_read_write((config & 0xFF00) >> 8);
    read_byte_low = spi_read_write(config & 0x00FF);
    spi_deselect(AD7689_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;

  case AD7689_SENSOR_CHANNEL1:

    /*spi_select(AD7689_CS);
    spi_write(WHO_AM_I_G);
    read_byte_low = spi_read();
    spi_deselect(AD7689_CS);

    return read_byte_low;*/
  }
  return 0;
}

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    /* sensor status init */
    ad7689_sensor_status = 0;
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      /* no init required */
      ad7689_sensor_status = 1;
    }
    else {
      /* disable/put into sleep */
      /* TODO: how to put AD7689 into sleep? */
      ad7689_sensor_status = 0;
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
    /* return sensor status */;
    return ad7689_sensor_status;
  }
  return 0;
}

SENSORS_SENSOR(ad7689_sensor, AD7689_SENSOR, value, configure, status);
