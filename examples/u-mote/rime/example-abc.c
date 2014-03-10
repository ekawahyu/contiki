/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Testing the abc layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"
#include "dev/spi-arch.h"
#include "dev/spi.h"
#include "dev/lsm330dlc.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
AUTOSTART_PROCESSES(&example_abc_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  printf("abc message received '%s'\n", (char *)packetbuf_dataptr());
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static unsigned char read_byte = 0xD1;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {

    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello from EKA:B", 17);
    abc_send(&abc);
    printf("abc message sent\n");

    leds_toggle(LEDS_RED);

	P1_0 = 0;
	//SPI_WRITE(CTRL_REG1_G);
	spi_arch_write(CTRL_REG1_G);
	P1_0 = 1;

	P1_0 = 0;
	//SPI_WRITE((DRBW_1000 | LPen_G | xyz_en_G));
	spi_arch_write((DRBW_1000 | LPen_G | xyz_en_G));
	P1_0 = 1;

	P1_0 = 0;
	//SPI_WRITE(CTRL_REG4_G);
	spi_arch_write(CTRL_REG4_G);
	P1_0 = 1;

	P1_0 = 0;
	//SPI_WRITE(WHO_AM_I_G);
	spi_arch_write(WHO_AM_I_G);
	P1_0 = 1;

	P1_0 = 0;
	//SPI_READ(read_byte);
	read_byte = spi_arch_read();
	P1_0 = 1;

	printf("read_byte 0x%X\n", read_byte);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
