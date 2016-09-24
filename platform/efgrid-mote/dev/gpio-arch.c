/*
 * gpio-arch.c
 *
 * Created on: May 13, 2014
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

#include "contiki-conf.h"
#include "dev/gpio.h"
#include "cc253x.h"
/*---------------------------------------------------------------------------*/
void
gpio_arch_init(void)
{
  /* TODO initialize as output only
   * GPIO should support input, output and tri-state
   */
#if MODELS_CONF_EVB485
  P0SEL &= ~(GPIO1_MASK | GPIO2_MASK | GPIO3_MASK | GPIO4_MASK | GPIO5_MASK);
  P0DIR |= (GPIO1_MASK | GPIO2_MASK | GPIO3_MASK | GPIO4_MASK | GPIO5_MASK);
#else
  P0SEL &= ~(GPIO1_MASK | GPIO2_MASK | GPIO3_MASK | GPIO4_MASK);
  P0DIR |= (GPIO1_MASK | GPIO2_MASK | GPIO3_MASK | GPIO4_MASK);
  P2SEL &= ~GPIO5_MASK;
  P2DIR |= GPIO5_MASK;
#endif
  gpio_arch_set(0);
}
/*---------------------------------------------------------------------------*/
unsigned char
gpio_arch_get(void)
{
  return (unsigned char)((GPIO1_PIN) | (GPIO2_PIN << 1) |
      (GPIO3_PIN << 2) | (GPIO4_PIN << 3) | GPIO5_PIN << 4);
}
/*---------------------------------------------------------------------------*/
void
gpio_arch_set(unsigned char gpio)
{
  GPIO1_PIN = (gpio & 0x01);
  GPIO2_PIN = (gpio & 0x02) >> 1;
  GPIO3_PIN = (gpio & 0x04) >> 2;
  GPIO4_PIN = (gpio & 0x08) >> 3;
  GPIO5_PIN = (gpio & 0x10) >> 4;
}
/*---------------------------------------------------------------------------*/
