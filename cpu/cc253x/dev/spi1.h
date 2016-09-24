/*
 * spi1.h
 *
 * Created on: Mar 10, 2014
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

#ifndef SPI1_H_
#define SPI1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki-conf.h"

/*---------------------------------------------------------------------------*/
/* SPI1 Enable - Disable */
#ifdef SPI1_CONF_ENABLE
#define SPI1_ENABLE SPI1_CONF_ENABLE
#else
#define SPI1_ENABLE 0
#endif
/*---------------------------------------------------------------------------*/
/* SPI1 Function Declarations */
#if SPI1_ENABLE
void spi1_init(unsigned char mode,
  unsigned char freq,
  unsigned char endianess);
void spi1_write(unsigned char data);
unsigned char spi1_read(void);
unsigned char spi1_read_write(unsigned char data);
#else
#define spi1_init(...)
#define spi1_write(...)
#define spi1_read(...) (0)
#define spi1_read_write(...) (0)
#endif /* SPI1_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* SPI1_H_ */
