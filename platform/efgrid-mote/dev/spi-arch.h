/*
 * spi-arch.h
 *
 * Created on: Mar 12, 2014
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

#ifndef SPI_ARCH_H_
#define SPI_ARCH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki-conf.h"

#if (UMOTE_SENSORS_ON_SPI == 0)
#include "dev/spi0.h"
#define IO_ARCH_PREFIX spi0
#elif (UMOTE_SENSORS_ON_SPI == 1)
#include "dev/spi1.h"
#define IO_ARCH_PREFIX spi1
#endif

/*---------------------------------------------------------------------------*/
/* Expands to spi0_functions(), spi1_functions() */
#define spi_arch_init(...) spi_arch_init_x(IO_ARCH_PREFIX, __VA_ARGS__)
#define spi_arch_write(...) spi_arch_write_x(IO_ARCH_PREFIX, __VA_ARGS__)
#define spi_arch_read(...) spi_arch_read_x(IO_ARCH_PREFIX, __VA_ARGS__)
#define spi_arch_read_write(...) spi_arch_read_write_x(IO_ARCH_PREFIX, __VA_ARGS__)
/*---------------------------------------------------------------------------*/
/* Second round of macro substitutions. You can stop reading here */
#define spi_arch_init_x(prefix, ...) spi_arch_init_x_x(prefix, __VA_ARGS__)
#define spi_arch_write_x(prefix, ...) spi_arch_write_x_x(prefix, __VA_ARGS__)
#define spi_arch_read_x(prefix, ...) spi_arch_read_x_x(prefix, __VA_ARGS__)
#define spi_arch_read_write_x(prefix, ...) spi_arch_read_write_x_x(prefix, __VA_ARGS__)
/*---------------------------------------------------------------------------*/
#define spi_arch_init_x_x(prefix, ...) prefix##_init(__VA_ARGS__)
#define spi_arch_write_x_x(prefix, ...) prefix##_write(__VA_ARGS__)
#define spi_arch_read_x_x(prefix, ...) prefix##_read(__VA_ARGS__)
#define spi_arch_read_write_x_x(prefix, ...) prefix##_read_write(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

void spi_arch_deselect_all(void);
void spi_arch_deselect(unsigned char cs);
void spi_arch_select(unsigned char cs);

#ifdef __cplusplus
}
#endif

#endif /* SPI_ARCH_H_ */
