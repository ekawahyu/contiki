/*
 * lsm330dlc-sensor.h
 *
 * Created on : Sept 26, 2013
 *     Author : Marco Beccani
 *
 * Modified on: Feb 25, 2014
 *     Author : Ekawahyu Susilo
 *
 * Copyright (c) 2014, STORM Lab Vanderbilt University.
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

#ifndef LSM330DLC_SENSOR_H_
#define LSM330DLC_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cc253x.h"
#include "contiki-conf.h"
#include "lib/sensors.h"


#define LSM330DLC_ACCL_CS     SPI_CS4
#define LSM330DLC_GYRO_CS     SPI_CS0

#ifndef DUMMY
#define DUMMY     0x00
#endif


/******************************************************************************
ACCELEROMETER
******************************************************************************/
// Internal registers mapping
#define CTRL_REG1_A 0x20
// CTRL_REG1_A Configuration words
#define POWER_DOWN_MODE_A      0x00
#define ACC_1_Hz_A             0x10
#define ACC_10_Hz_A            0x20
#define ACC_25_Hz_A            0x30
#define ACC_50_Hz_A            0x40
#define ACC_100_Hz_A           0x50
#define ACC_200_Hz_A           0x60
#define ACC_400_Hz_A           0x70
#define ACC_1620_Hz_A          0x80 // only low power mode
#define ACC_1344_Hz_A          0x90 // low power mode 5376 Hz

#define LPen_A             0x08 // low power mode enable

#define x_en_A             0x01
#define y_en_A             0x02
#define xy_en_A            0x03
#define z_en_A             0x04
#define xz_en_A            0x05
#define yz_en_A            0x06
#define xyz_en_A           0x07

#define CTRL_REG4_A        0x23
// CTRL_REG4_A Configuration words
#define BLE                 0x70
#define ACC_16G_A           0x30
#define ACC_8G_A            0x20
#define ACC_4G_A            0x10
#define ACC_2G_A            0x00
#define HR_A                0x08
#define ACC_3WIRE_A         0x01

#define STATUS_REG_A   (0x27| 0x80)  // Read Access

// Axes accelerometer
#define ACC_REG_OUT_X_L (0x28 | 0x80)
#define ACC_REG_OUT_X_H (0x29 | 0x80)
#define ACC_REG_OUT_Y_L (0x2A | 0x80)
#define ACC_REG_OUT_Y_H (0x2B | 0x80)
#define ACC_REG_OUT_Z_L (0x2C | 0x80)
#define ACC_REG_OUT_Z_H (0x2D | 0x80)

#define XYZ_A  (0x28 | 0xC0)   // Read All Axis


/******************************************************************************
GYROSCOPE
******************************************************************************/
#define WHO_AM_I_G  (0x0F | 0x80)

#define CTRL_REG1_G 0x20
// CTRL_REG1_G Configuration words

#define DRBW_1111  0xF0 // DRBW_1111
#define DRBW_1110  0xE0 // DRBW_1110
#define DRBW_1101  0xD0 // DRBW_1101
#define DRBW_1100  0xC0 // DRBW_1100
#define DRBW_1011  0xB0 // DRBW_1011
#define DRBW_1010  0xA0 // DRBW_1010
#define DRBW_1001  0x90 // DRBW_1001
#define DRBW_1000  0x80 // DRBW_1000
#define DRBW_0111  0x70 // DRBW_0111
#define DRBW_0110  0x60 // DRBW_0110
#define DRBW_0101  0x50 // DRBW_0101
#define DRBW_0100  0x40 // DRBW_0100
#define DRBW_0011  0x30 // DRBW_0011
#define DRBW_0010  0x20 // DRBW_0011
#define DRBW_0001  0x10 // DRBW_0001
#define DRBW_0000  0x00 // DRBW_0000

#define LPen_G            0x08 // low power mode enable

#define x_en_G            0x01
#define y_en_G            0x02
#define xy_en_G           0x03
#define z_en_G            0x04
#define xz_en_G           0x05
#define yz_en_G           0x06
#define xyz_en_G          0x07

#define CTRL_REG4_G       0x23

// CTRL_REG4_G Configuration words
#define BDU                 0x80

#ifndef BLE
#define BLE                 0x70
#endif

#define GYR_2000_dps_2 0x30
#define GYR_2000_dps   0x20
#define GYR_500_dps    0x10
#define GYR_250_dps    0x00
#define GYR_3WIRE_G    0x01

#define OUT_TEMP_G      (0x26 | 0x80)
#define STATUS_REG_G    (0x27 | 0x80)   // Read Access

#define GYR_REG_OUT_X_L (0x28 | 0x80)
#define GYR_REG_OUT_X_H (0x29 | 0x80)
#define GYR_REG_OUT_Y_L (0x2A | 0x80)
#define GYR_REG_OUT_Y_H (0x2B | 0x80)
#define GYR_REG_OUT_Z_L (0x2C | 0x80)
#define GYR_REG_OUT_Z_H (0x2D | 0x80)

#define XYZ_G  (0x28 | 0xC0)   // Read All Axis

/* Sensor types */
#define LSM330DLC_SENSOR "LSM330DLC"

#define LSM330DLC_SENSOR_TYPE_ACCL_X      0
#define LSM330DLC_SENSOR_TYPE_ACCL_Y      1
#define LSM330DLC_SENSOR_TYPE_ACCL_Z      3
#define LSM330DLC_SENSOR_TYPE_GYRO_X      4
#define LSM330DLC_SENSOR_TYPE_GYRO_Y      5
#define LSM330DLC_SENSOR_TYPE_GYRO_Z      6
#define LSM330DLC_SENSOR_TYPE_GYRO_TEMP   7
#define LSM330DLC_SENSOR_TYPE_GYRO_STATUS 8
#define LSM330DLC_SENSOR_TYPE_ID          9

#ifdef LSM330DLC_SENSOR_CONF_ON
#define LSM330DLC_SENSOR_ON LSM330DLC_SENSOR_CONF_ON
#endif

#if LSM330DLC_SENSOR_ON
extern const struct sensors_sensor lsm330dlc_sensor;
#define   LSM330DLC_SENSOR_ACTIVATE() lsm330dlc_sensor.configure(SENSORS_ACTIVE, 1)
#else
#define   LSM330DLC_SENSOR_ACTIVATE()
#endif

#ifdef __cplusplus
}
#endif

#endif /* LSM330DLC_SENSOR_H_ */
