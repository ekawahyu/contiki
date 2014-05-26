/*
 * ad7689-sensor.h
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

#ifndef AD7689_SENSOR_H_
#define AD7689_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cc253x.h"
#include "contiki-conf.h"
#include "lib/sensors.h"

#define AD7689_CS              SPI_CS0

#define AD7689_CFG_UPDATE         0x8000
#define AD7689_CFG_KEEP           0x0000

#define INPUT_BIPOLAR_DIFF_VREFDIV2   0x0000
#define INPUT_BIPOLAR_COM             0x2000
#define INPUT_TEMPERATURE             0x3000
#define INPUT_UNIPOLAR_DIFF_GND       0x4000
#define INPUT_UNIPOLAR_COM            0x6000
#define INPUT_UNIPOLAR_GND            0x7000

#define INPUT_SELECT_IN0        0x0000
#define INPUT_SELECT_IN1        0x0200
#define INPUT_SELECT_IN2        0x0400
#define INPUT_SELECT_IN3        0x0600
#define INPUT_SELECT_IN4        0x0800
#define INPUT_SELECT_IN5        0x0A00
#define INPUT_SELECT_IN6        0x0C00
#define INPUT_SELECT_IN7        0x0E00

#define FULL_BANDWIDTH          0x0100

#define INT_REF25_WITH_TEMPERATURE        0x0000
#define INT_REF4096_WITH_TEMPERATURE      0x0020
#define EXT_REF_WITH_TEMPERATURE          0x0040
#define EXT_REF_INT_BUFF_WITH_TEMPERATURE 0x0060
#define EXT_REF_NO_TEMPERATURE            0x00C0
#define EXT_REF_INT_BUFF_NO_TEMPERATURE   0x00E0

#define DISABLE_SEQUENCER               0x0000
#define UPDATE_SEQUENCER                0x0008
#define SCAN_IN0_INX_TEMPERATURE        0x0010
#define SCAN_IN0_INX                    0x0018

#define READBACK_CONFIG                 0x0000
#define DO_NOT_READBACK_CONFIG          0x0004

/* Sensor types */
#define AD7689_SENSOR "AD7689"

#define AD7689_SENSOR_CHANNEL0      0
#define AD7689_SENSOR_CHANNEL1      1
#define AD7689_SENSOR_CHANNEL2      2
#define AD7689_SENSOR_CHANNEL3      3
#define AD7689_SENSOR_CHANNEL4      4
#define AD7689_SENSOR_CHANNEL5      5
#define AD7689_SENSOR_CHANNEL6      6
#define AD7689_SENSOR_CHANNEL7      7

#ifdef AD7689_SENSOR_CONF_ON
#define AD7689_SENSOR_ON AD7689_SENSOR_CONF_ON
#endif

#if AD7689_SENSOR_ON
extern const struct sensors_sensor ad7689_sensor;
#define   AD7689_SENSOR_ACTIVATE() ad7689_sensor.configure(SENSORS_ACTIVE, 1)
#else
#define   AD7689_SENSOR_ACTIVATE()
#endif

#ifdef __cplusplus
}
#endif

#endif /* AD7689_SENSOR_H_ */
