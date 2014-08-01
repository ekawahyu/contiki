/*
 * uart-arch.h
 *
 * Created on: Jul 23, 2014
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

#ifndef UART_ARCH_H_
#define UART_ARCH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki.h"

#if UART0_ENABLE
#include "dev/uart0.h"
#define UART_ARCH_PREFIX uart0
#else /* UART1_ENABLE */
#include "dev/uart1.h"
#define UART_ARCH_PREFIX uart1
#endif

/*---------------------------------------------------------------------------*/
/* Expands to uart0_functions(), uart1_functions() */
#define uart_arch_init(...) uart_arch_init_x(UART_ARCH_PREFIX, __VA_ARGS__)
#define uart_arch_writeb(...) uart_arch_writeb_x(UART_ARCH_PREFIX, __VA_ARGS__)
#define uart_arch_set_input(f) uart_arch_set_input_x(UART_ARCH_PREFIX, f)
/*---------------------------------------------------------------------------*/
/* Second round of macro substitutions. You can stop reading here */
#define uart_arch_init_x(prefix, ...) uart_arch_init_x_x(prefix, __VA_ARGS__)
#define uart_arch_writeb_x(prefix, ...) uart_arch_writeb_x_x(prefix, __VA_ARGS__)
#define uart_arch_set_input_x(prefix, f) uart_arch_set_input_x_x(prefix, f)
/*---------------------------------------------------------------------------*/
#define uart_arch_init_x_x(prefix, ...) prefix##_init(__VA_ARGS__)
#define uart_arch_writeb_x_x(prefix, ...) prefix##_writeb(__VA_ARGS__)
#define uart_arch_set_input_x_x(prefix, f) prefix##_set_input(f)
/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* UART_ARCH_H_ */
