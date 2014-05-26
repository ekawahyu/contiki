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

#include "dev/spi.h"
#include "dev/lsm330dlc-sensor.h"

static int lsm330dlc_sensor_status;

static int
value(int type)
{
  unsigned char read_byte_high;
  unsigned char read_byte_low;
  int sensor_value;

  switch(type) {
  case LSM330DLC_SENSOR_TYPE_ACCL_X:

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_X_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_X_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_ACCL_Y:

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_Y_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_Y_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_ACCL_Z:

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_Z_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    spi_select(LSM330DLC_ACCL_CS);
    spi_write(ACC_REG_OUT_Z_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_ACCL_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_GYRO_X:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_X_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_X_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_GYRO_Y:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_Y_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_Y_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_GYRO_Z:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_Z_L);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(GYR_REG_OUT_Z_H);
    read_byte_high = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    sensor_value = read_byte_high;
    sensor_value = ((sensor_value << 8) & 0xFF00) | read_byte_low;
    return sensor_value;
  case LSM330DLC_SENSOR_TYPE_GYRO_TEMP:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(OUT_TEMP_G);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    return read_byte_low;
  case LSM330DLC_SENSOR_TYPE_GYRO_STATUS:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(STATUS_REG_G);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    return read_byte_low;
  case LSM330DLC_SENSOR_TYPE_ID:

    spi_select(LSM330DLC_GYRO_CS);
    spi_write(WHO_AM_I_G);
    read_byte_low = spi_read();
    spi_deselect(LSM330DLC_GYRO_CS);

    return read_byte_low;
  }
  return 0;
}

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    /* sensor status init */
    lsm330dlc_sensor_status = 0;
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      /* initialize gyro */
      spi_select(LSM330DLC_GYRO_CS);
      spi_write(CTRL_REG1_G);
      spi_write((DRBW_1000 | LPen_G | xyz_en_G));
      spi_deselect(LSM330DLC_GYRO_CS);

      spi_select(LSM330DLC_GYRO_CS);
      spi_write(CTRL_REG4_G);
      spi_write(0x00);
      spi_deselect(LSM330DLC_GYRO_CS);

      /* initialize accelerometer */
      spi_select(LSM330DLC_ACCL_CS);
      spi_write(CTRL_REG1_A);
      spi_write(ACC_400_Hz_A | xyz_en_A);
      spi_deselect(LSM330DLC_ACCL_CS);

      spi_select(LSM330DLC_ACCL_CS);
      spi_write(CTRL_REG4_G);
      spi_write(ACC_2G_A | HR_A);
      spi_deselect(LSM330DLC_ACCL_CS);

      lsm330dlc_sensor_status = 1;
    }
    else {
      /* disable/put into sleep */
      /* TODO: how to put LSM330DLC into sleep? */
      lsm330dlc_sensor_status = 0;
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
    return lsm330dlc_sensor_status;
  }
  return 0;
}

SENSORS_SENSOR(lsm330dlc_sensor, LSM330DLC_SENSOR, value, configure, status);
