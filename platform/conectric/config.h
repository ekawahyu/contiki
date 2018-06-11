/*
 * config.h
 *
 *  Created on: May 1, 2018
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2018, Conectric Network, LLC.
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

#ifndef CONFIG_H_
#define CONFIG_H_

#include "soc.h"
#include "cc253x.h"
#include "8051def.h"
#include "flash.h"

#define CONFIG_ADDR_START       0x3E800 /* flash page 125 */
#define CONFIG_MIRROR_START     0x3F000 /* flash page 126 */

#define CONFIG_SERIAL_NUMBER    0x3EFDC /* word address */
#define CONFIG_RS485_PARAMS     0x3EFD8 /* word address */
#define CONFIG_FLASH_LOCK       0x3FFF0 /* word address */

#define CONFIG_SERIAL_NUMBER_LENGTH   12
#define CONFIG_RS485_PARAMS_LENGTH    4
#define CONFIG_FLASH_LOCK_LENGTH      16

uint8_t config_sernum_read(uint8_t * sernum);
uint8_t config_sernum_write(uint8_t * sernum);
uint8_t config_rs485_params_read(uint8_t * params);
uint8_t config_update(uint32_t config_addr, uint8_t * config, uint8_t len);

#endif /* CONFIG_H_ */
