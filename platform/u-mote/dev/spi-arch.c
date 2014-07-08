/*
 * spi-arch.c
 *
 * Created on: Apr 24, 2014
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
#include "dev/spi.h"
#include "dev/spi-arch.h"
#include "cc253x.h"

void spi_arch_cs_init(void)
{
  P1SEL &= ~(SPI_CS1_MASK | SPI_CS2_MASK | SPI_CS3_MASK | SPI_CS4_MASK | SPI_CS5_MASK);
  P1DIR |= (SPI_CS1_MASK | SPI_CS2_MASK | SPI_CS3_MASK | SPI_CS4_MASK | SPI_CS5_MASK);
}

void
spi_arch_deselect_all(void)
{
  spi_arch_select(~SPI_CS_ALL);
}

void
spi_arch_select(unsigned char cs)
{
#if (SPI0_CONF_ENABLE || SPI1_CONF_ENABLE)
  SPI_CS1_PIN = cs & 0x01;
  SPI_CS2_PIN = (cs & 0x02) >> 1;
  SPI_CS3_PIN = (cs & 0x04) >> 2;
  SPI_CS4_PIN = (cs & 0x08) >> 3;
  SPI_CS5_PIN = (cs & 0x10) >> 4;
#endif
}
