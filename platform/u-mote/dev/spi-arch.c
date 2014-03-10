/*
 * spi-arch.c
 *
 * Created on: Mar 7, 2014
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

#include "dev/spi-arch.h"
#include "cc253x.h"
#include "sfr-bits.h"

#include <stdio.h>

void
spi_arch_init(void)
{
	// Master Mode
	PERCFG |= 0x02; // PERCFG.U1CFG = 1
	P1SEL |= 0xE0; // P1_7, P1_6, and P1_5 are peripherals
	P1SEL &= ~0x18; // P1_0 is GPIO (SSN)
	P1DIR |= 0x3F; // SSN is set as output P1_0 to P1_0
	// Set baud rate to max (system clock frequency / 8)
	// Assuming a 32 MHz crystal (CC1110Fx/CC2510Fx),
	// max baud rate = 32 MHz / 8 = 4 MHz.
	U1BAUD = 0x57; // BAUD_M = 216
	U1GCR |= 0x10; // BAUD_E = 15
	// SPI Master Mode
	U1CSR &= ~0xA0;
	// Configure phase, polarity, and bit order
	U1GCR &= ~0xC0; // CPOL = CPHA = 0
	U1GCR |= 0x20; // ORDER = 1
}

unsigned char
spi_arch_read(void)
{
	unsigned char data_read;

	RX_BYTE = 0;
	U1DBUF = 0;
	printf("spi_arch_read .(%i)\n", RX_BYTE);
	while(!(RX_BYTE));
	printf("spi_arch_read ..(%i)\n", RX_BYTE);
	data_read = U1DBUF;

	return data_read;
}

void
spi_arch_write(unsigned char data)
{
	TX_BYTE = 0;
	U1DBUF = data;
	printf("spi_arch_write 0x%X .(%i)\n", data, TX_BYTE);
	while(!(TX_BYTE));
	printf("spi_arch_write 0x%X ..(%i)\n", data, TX_BYTE);
}

unsigned char
spi_write_read(unsigned char data)
{
	unsigned char data_read;

	U1DBUF = data;
	while(!(TX_BYTE));
	while(!(RX_BYTE));
	data_read = U1DBUF;

	return data_read;
}
