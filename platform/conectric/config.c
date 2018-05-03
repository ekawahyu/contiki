/*
 * config.c
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

#include <stdio.h>
#include "config.h"

/*
 * Configuration structure is stored as one flash page (2048 bytes) located at
 * the last page of flash memory area. Conectric platform includes CC2530F256
 * SoC with 256kB of flash divided into 128 valid pages (0-127).
 *
 * This configuration is stored on page 127.
 */

#if defined __IAR_SYSTEMS_ICC__
__root __code const uint8_t config_rs485[4] @ 0x7FFD8 =
{0x00, 0x00, 0x00, 0x02};
__code const uint8_t config_sn[12] @ 0x7FFDC =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
__code const __at(CONFIG_RS485_PARAMS) uint8_t config_rs485[4] =
{0x00, 0x00, 0x00, 0x02};
__code const __at(CONFIG_SERIAL_NUMBER) uint8_t config_sn[12] =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#endif

/*---------------------------------------------------------------------------*/
uint8_t
config_sernum_read(uint8_t * sernum)
{
  flash_read(
      FLASH_PAGE(CONFIG_SERIAL_NUMBER),
      FLASH_PAGE_OFFSET(CONFIG_SERIAL_NUMBER),
      sernum,
      CONFIG_SERIAL_NUMBER_LENGTH
  );

  return CONFIG_SERIAL_NUMBER_LENGTH;
}
/*---------------------------------------------------------------------------*/
uint8_t
config_sernum_write(uint8_t * sernum)
{
  uint8_t len;
  uint8_t curr_sernum[CONFIG_SERIAL_NUMBER_LENGTH];

  len = config_sernum_read(curr_sernum);

  /*
   * If a serial number has been assigned, they are very likely to show
   * value other than 0xFF. Any attempt to overwrite is prohibited.
   */
  while(len--) {
    if (curr_sernum[len] != 0xFF)
      return 0;
    else
      curr_sernum[len] = *sernum++;
  }

  /* All is good, flash it */
  flash_write_DMA(
      curr_sernum,
      CONFIG_SERIAL_NUMBER_LENGTH,
      FLASH_WORD_ADDR(CONFIG_SERIAL_NUMBER)
  );

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
config_rs485_params_read(uint8_t * params)
{
  flash_read(
      FLASH_PAGE(CONFIG_RS485_PARAMS),
      FLASH_PAGE_OFFSET(CONFIG_RS485_PARAMS),
      params,
      CONFIG_RS485_PARAMS_LENGTH
  );

  return CONFIG_RS485_PARAMS_LENGTH;
}
/*---------------------------------------------------------------------------*/
uint8_t
config_update(uint32_t config_addr, uint8_t * config, uint8_t len)
{
  uint32_t flash_src_addr;
  uint32_t flash_dest_addr;
  uint8_t buffer[4];
  uint16_t loop;

  /* Mirror everything onto another flash page */
  flash_page_erase(FLASH_PAGE(CONFIG_MIRROR_START));
  flash_dest_addr = CONFIG_MIRROR_START;
  flash_src_addr = CONFIG_ADDR_START;
  loop = 512;
  while(loop--) {
    flash_read(
        FLASH_PAGE(flash_src_addr),
        FLASH_PAGE_OFFSET(flash_src_addr),
        buffer,
        4
    );
    flash_write_DMA(
        buffer,
        4,
        FLASH_WORD_ADDR(flash_dest_addr)
    );
    flash_src_addr += 4;
    flash_dest_addr += 4;
  }

  /* Copy everything back except the part that needs to get updated */
  flash_page_erase(FLASH_PAGE(CONFIG_ADDR_START));
  flash_dest_addr = CONFIG_ADDR_START;
  flash_src_addr = CONFIG_MIRROR_START;
  loop = 512;
  while(loop--) {
    flash_read(
        FLASH_PAGE(flash_src_addr),
        FLASH_PAGE_OFFSET(flash_src_addr),
        buffer,
        4
    );
    if((flash_dest_addr == config_addr) && (len != 0)) {
      config_addr += 4;
      len -= 4;
      buffer[0] = *config++;
      buffer[1] = *config++;
      buffer[2] = *config++;
      buffer[3] = *config++;
    }
    flash_write_DMA(
        buffer,
        4,
        FLASH_WORD_ADDR(flash_dest_addr)
    );
    flash_src_addr += 4;
    flash_dest_addr += 4;
  }

  return 1;
}
