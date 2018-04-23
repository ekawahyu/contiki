/*
 * sernum.c
 *
 *  Created on: Apr 19, 2018
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
#include "sernum.h"

#if defined __IAR_SYSTEMS_ICC__
/*
 * The device serial number is always stored in 0xFFDC of the highest BANK of
 * our flash. This maps to address 0xFFDC of our XDATA segment, when this BANK
 * is selected. IAR __code placement uses logical code memory addressing, 0xFFDC
 * on bank 7 is equivalent to 0x7FFDC as follow. This code can be used even
 * without a bankable firmware.
 */
__code const uint8_t hardcoded_sn[12] @ 0x7FFDC =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
/*
 * The device serial number is always stored in 0xFFDC of the highest BANK of
 * our flash. This maps to address 0xFFDC of our XDATA segment, when this BANK
 * is selected. Load the bank, read 12 bytes starting at 0xFFDC and restore last BANK.
 * Since we are called from main(), this MUST be BANK1 or something is very
 * wrong. This code can be used even without a bankable firmware.
 */
#define USER_PROGRAM_PAGE   7
__code const __at(USER_PROGRAM_PAGE * 0x8000 + 0x7FDC) uint8_t hardcoded_sn[12] =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#endif

/*---------------------------------------------------------------------------*/
uint8_t
sernum_read(uint8_t * sernum) CC_NON_BANKED
{
  uint8_t len;
  uint8_t * snp = (uint8_t *)hardcoded_sn;
  uint8_t memctr = MEMCTR;

  MEMCTR = (MEMCTR & 0xF8) | 0x07;

  len = DEVICE_SERIAL_NUMBER_LENGTH;
  while(len--) *sernum++ = *snp++;

  MEMCTR = memctr;

  return DEVICE_SERIAL_NUMBER_LENGTH;
}
/*---------------------------------------------------------------------------*/
uint8_t
sernum_write(uint8_t * sernum) CC_NON_BANKED
{
  uint8_t len;
  uint8_t * snp = (uint8_t *)hardcoded_sn;
  uint8_t invert[DEVICE_SERIAL_NUMBER_LENGTH];
  uint8_t memctr = MEMCTR;

  MEMCTR = (MEMCTR & 0xF8) | 0x07;

  len = DEVICE_SERIAL_NUMBER_LENGTH;

  /*
   * Serial number has been written once they are not 0xFF, any attempt to
   * overwrite is prohibited
   */
  while(len--) {
    if (*snp++ != 0xFF) return 0;
  }

  MEMCTR = memctr;

  /* Invert endianness */
  len = DEVICE_SERIAL_NUMBER_LENGTH;
  while(len--) invert[len] = *sernum++;

  /* Write the serial number */
  if(invert) flash_write_DMA(invert, DEVICE_SERIAL_NUMBER_LENGTH, FLASH_WORD_ADDR(0x3FFDC));

  return 1;
}
/*---------------------------------------------------------------------------*/
