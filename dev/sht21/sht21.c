/*
 * sht21.c
 *
 * Created on: Feb 1, 2017
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2017, Chongqing Aisenke Electronic Technology Co., Ltd.
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

/*
 * Device driver for the Sensirion SHT2x family of humidity and
 * temperature sensors.
 */

#include "contiki.h"
#include <stdio.h>
#include "dev/sht21/sht21.h"
#include "sht21-arch.h"

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef SDA_0
#define SDA_OUT   SHT21_PxDIR |= BV(SHT21_ARCH_SDA)
#define SDA_IN    SHT21_PxDIR &= ~(BV(SHT21_ARCH_SDA))
#define SCL_OUT   SHT21_PxDIR |= BV(SHT21_ARCH_SCL)
#define SCL_IN    SHT21_PxDIR &= ~(BV(SHT21_ARCH_SCL))
#define SDA(x)    SHT21_SDA = x
#define SCL(x)    SHT21_SCL = x
#define SDA_R     SHT21_SDA
#define SCL_R     SHT21_SCL
#endif

#define SHT2x_TRIG_T_MEASUREMENT_HM       0xE3
#define SHT2x_TRIG_RH_MEASUREMENT_HM      0xE5
#define SHT2x_TRIG_T_MEASUREMENT_POLL     0xF3
#define SHT2x_TRIG_RH_MEASUREMENT_POLL    0xF5
#define SHT2x_USER_REG_W                  0xE6
#define SHT2x_USER_REG_R                  0xE7
#define SHT2x_SOFT_RESET                  0xFE

#define SHT2x_RES_12_14BIT    0x00
#define SHT2x_RES_8_12BIT     0x01
#define SHT2x_RES_10_13BIT    0x80
#define SHT2x_RES_11_11BIT    0x81
#define SHT2x_RES_MASK        0x81

#define SHT2x_I2C_ADR_W       0x80
#define SHT2x_I2C_ADR_R       0x81

/*---------------------------------------------------------------------------*/
static void
delay_sht21(void)
{
  clock_delay_usec(1);
}
/*---------------------------------------------------------------------------*/
static void
sstart(void)
{
  SDA(1);
  SCL(1);
  SDA_OUT;
  SCL_OUT;
  delay_sht21();
  SDA(0);
  delay_sht21();
  SCL(0);
  delay_sht21();
}
/*---------------------------------------------------------------------------*/
static void
sstop(void)
{
  SCL(0);
  SDA(0);
  SCL_OUT;
  SDA_OUT;
  delay_sht21();
  SCL(1);
  delay_sht21();
  SDA(1);
  delay_sht21();
}
/*---------------------------------------------------------------------------*/
static void
sreset(void)
{

}
/*---------------------------------------------------------------------------*/
/*
 * Return true if we received an ACK.
 */
static int
swrite(unsigned _c)
{
  unsigned char c = _c;
  int i;
  int ret;

  SDA(1);
  SDA_OUT;
  SCL_OUT;
  for(i = 0; i < 8; i++, c <<= 1) {
    if(c & 0x80) {
      SDA(1);
    } else {
      SDA(0);
    }
    delay_sht21();
    SCL(1);
    delay_sht21();
    SCL(0);
    delay_sht21();
  }
  SDA_IN;
  delay_sht21();
  SCL(1);
  ret = SDA_R;
  delay_sht21();
  SCL(0);
  delay_sht21();

  return ret;
}
/*---------------------------------------------------------------------------*/
static unsigned
sread(int send_ack)
{
  int i;
  unsigned char c = 0x00;

  SDA_IN;
  SCL_OUT;
  for(i = 0; i < 8; i++) {
    c <<= 1;
    SCL(1);
    delay_sht21();
    if(SDA_R) {
      c |= 0x1;
    }
    SCL(0);
    delay_sht21();
  }

  SDA_OUT;
  if(send_ack) {
    SDA(0);
  }
  else {
    SDA(1);
  }
  SCL(1);
  delay_sht21();
  SCL(0);
  delay_sht21();

  return c;
}
/*---------------------------------------------------------------------------*/
#define CRC_CHECK
#ifdef CRC_CHECK
static unsigned char
rev8bits(unsigned char v)
{
  unsigned char r = v;
  int s = 7;

  for (v >>= 1; v; v >>= 1) {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s;		    /* Shift when v's highest bits are zero */
  return r;
}
/*---------------------------------------------------------------------------*/
/* BEWARE: Bit reversed CRC8 using polynomial ^8 + ^5 + ^4 + 1 */
static unsigned
crc8_add(unsigned acc, unsigned byte)
{
  int i;
  acc ^= byte;
  for(i = 0; i < 8; i++) {
    if(acc & 0x80) {
      acc = (acc << 1) ^ 0x31;
    } else {
      acc <<= 1;
    }
  }
  return acc & 0xff;
}
#endif /* CRC_CHECK */
/*---------------------------------------------------------------------------*/
/*
 * Power up the device. The device can be used after an additional
 * 11ms waiting time.
 */
void
sht21_init(void)
{
#ifdef SHT21_INIT
  SHT21_INIT();
#else
  /* As this driver is bit-bang based, disable the I2C first
     This assumes the SDA/SCL pins passed in the -arch.h file are 
     actually the same used for I2C operation, else comment out the following
  */
  SHT21_PxSEL &= ~(BV(SHT21_ARCH_SDA) | BV(SHT21_ARCH_SCL));
  SDA_OUT;
  SCL_OUT;
  SDA(1);
  SCL(1);
#endif
}
/*---------------------------------------------------------------------------*/
/*
 * Power of device.
 */
void
sht21_off(void)
{
#ifdef SHT21_OFF
  SHT21_OFF();
#else
  SDA_IN;
  SCL_IN;
  SDA(1);
  SCL(1);
#endif
}
/*---------------------------------------------------------------------------*/
static unsigned int
scmd(unsigned cmd)
{
  /*if(cmd != SHT2x_TRIG_T_MEASUREMENT_POLL && cmd != SHT2x_TRIG_RH_MEASUREMENT_POLL) {
    PRINTF("Illegal command: %d\n", cmd);
    return -1;
  }*/

  sstart();

  if(swrite(SHT2x_I2C_ADR_W)) {
    PRINTF("SHT21: scmd - swrite failed 1\n");
    return -1;
  }

  if(swrite(cmd)) {
    PRINTF("SHT21: scmd - swrite failed 2\n");
    return -1;
  }

  sstop();

  return 0;

/*  for(n = 0; n < 20000; n++) {
    if(!SDA_IS_1) {
      unsigned t0, t1, rcrc;
      t0 = sread(1);
      t1 = sread(1);
      rcrc = sread(0);
      PRINTF("SHT21: scmd - read %d, %d\n", t0, t1);
#ifdef CRC_CHECK
      {
	unsigned crc;
	crc = crc8_add(0x0, cmd);
	crc = crc8_add(crc, t0);
	crc = crc8_add(crc, t1);
	if(crc != rev8bits(rcrc)) {
	  PRINTF("SHT21: scmd - crc check failed %d vs %d\n",
		 crc, rev8bits(rcrc));
	  goto fail;
	}
      }
#endif
      return (t0 << 8) | t1;
    }
    // short wait before next loop
    clock_wait(1);
  }*/
}
/*---------------------------------------------------------------------------*/
unsigned int
sht21_temp_acq(void)
{
  return scmd(SHT2x_TRIG_T_MEASUREMENT_POLL);
}
/*---------------------------------------------------------------------------*/
unsigned int
sht21_temp_result(void)
{
  unsigned t0, t1, rcrc;

  sstart();
  swrite(SHT2x_I2C_ADR_R);
  t0 = sread(1);
  t1 = sread(1);
  rcrc = sread(0);
  sstop();
  if (t1 & 0x02)
    PRINTF("SHT21: scmd - read RH %d, %d\n", t0, t1);
  else
    PRINTF("SHT21: scmd - read T %d, %d\n", t0, t1);
  return (t0 << 8) | t1;
}
/*---------------------------------------------------------------------------*/
unsigned int
sht21_humidity_acq(void)
{
  return scmd(SHT2x_TRIG_RH_MEASUREMENT_POLL);
}
/*---------------------------------------------------------------------------*/
unsigned int
sht21_humidity_result(void)
{
  unsigned t0, t1, rcrc;

  sstart();
  swrite(SHT2x_I2C_ADR_R);
  t0 = sread(1);
  t1 = sread(1);
  rcrc = sread(0);
  sstop();
  if (t1 & 0x02)
    PRINTF("SHT21: scmd - read RH %d, %d\n", t0, t1);
  else
    PRINTF("SHT21: scmd - read T %d, %d\n", t0, t1);
  return (t0 << 8) | t1;
}
/*---------------------------------------------------------------------------*/
unsigned int
sht21_user_reg(void)
{
  unsigned user_reg;

  sstart();
  swrite(SHT2x_I2C_ADR_W);
  swrite(SHT2x_USER_REG_R);
  sstart();
  swrite(SHT2x_I2C_ADR_R);
  user_reg = sread(0);
  PRINTF("SHT21: user_reg read %d\n", user_reg);
  sstart();
  swrite(SHT2x_I2C_ADR_W);
  swrite(SHT2x_USER_REG_W);
  swrite(user_reg | 0x81); /* 11-bit RHT */
  sstop();

  return 0;
}
/*---------------------------------------------------------------------------*/
