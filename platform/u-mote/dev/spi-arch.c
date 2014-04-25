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

#include "contiki.h"
#include "dev/spi.h"

void
spi_arch_deselect_all(void)
{
  spi_deselect(SPI_CS0);
  spi_deselect(SPI_CS1);
  spi_deselect(SPI_CS2);
  spi_deselect(SPI_CS3);
  spi_deselect(SPI_CS4);
}

void
spi_arch_select(unsigned char cs)
{
#if (SPI0_CONF_ENABLE || SPI1_CONF_ENABLE)
  switch (cs) {
  case SPI_CS0:
    P1_0 = 0;
    break;
  case SPI_CS1:
    P1_1 = 0;
    break;
  case SPI_CS2:
    P1_2 = 0;
    break;
  case SPI_CS3:
    P1_3 = 0;
    break;
  case SPI_CS4:
    P1_4 = 0;
    break;
  default:
    /* TODO: invalid chip select */
    break;
  }
#endif
}

void
spi_arch_deselect(unsigned char cs)
{
#if (SPI0_CONF_ENABLE || SPI1_CONF_ENABLE)
  switch (cs) {
  case SPI_CS0:
    P1_0 = 1;
    break;
  case SPI_CS1:
    P1_1 = 1;
    break;
  case SPI_CS2:
    P1_2 = 1;
    break;
  case SPI_CS3:
    P1_3 = 1;
    break;
  case SPI_CS4:
    P1_4 = 1;
    break;
  default:
    /* TODO: invalid chip select */
    break;
  }
#endif
}
