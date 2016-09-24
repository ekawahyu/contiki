/*
 * Copyright (c) 2011, George Oikonomou - <oikonomou@users.sourceforge.net>
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
 */

/**
 * \file
 *         Header file use to configure differences between cc2530dk builds for
 *         the SmartRF/cc2530 and the cc2531 USB stick.
 *
 *         These configuration directives are hardware-specific and you
 *         normally won't have to modify them.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#ifndef MODELS_H_
#define MODELS_H_

/*---------------------------------------------------------------------------*/
/* LEDs */
/*---------------------------------------------------------------------------*/
/* Some files include leds.h before us */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED

/*
 * SMAC-2530 LEDs
 *  1: P1_0 (Green)
 *  2: P1_1 (Red)
 *  3: P1_4 (Yellow)
 *  4: P2_0 (Blue)
 */

#define MODEL_STRING "SMAC-2530DB\n"
#define LEDS_GREEN    1
#define LEDS_RED      2
#define LEDS_YELLOW   3
#define LEDS_BLUE     4

/* H/W Connections */
#define LED1_PIN   P1_0
#define LED2_PIN   P1_1
#define LED3_PIN   P1_4
#define LED4_PIN   P2_0

/* PxDIR and PxSEL masks */
#define LED1_MASK  0x01
#define LED2_MASK  0x02
#define LED3_MASK  0x10
#define LED4_MASK  0x01

/* H/W Connections */
#define GPIO1_PIN   P0_4
#define GPIO2_PIN   P0_5
#define GPIO3_PIN   P0_6
#define GPIO4_PIN   P0_7
#define GPIO5_PIN   P2_0

/* PxDIR and PxSEL masks */
#define GPIO1_MASK   0x10
#define GPIO2_MASK   0x20
#define GPIO3_MASK   0x40
#define GPIO4_MASK   0x80
#define GPIO5_MASK   0x01

/* H/W Connections */
#define SPI_CS1_PIN   P1_0
#define SPI_CS2_PIN   P1_1
#define SPI_CS3_PIN   P1_2
#define SPI_CS4_PIN   P1_3
#define SPI_CS5_PIN   P1_4

/* PxDIR and PxSEL masks */
#define SPI_CS1_MASK   0x01
#define SPI_CS2_MASK   0x02
#define SPI_CS3_MASK   0x04
#define SPI_CS4_MASK   0x08
#define SPI_CS5_MASK   0x10

/*---------------------------------------------------------------------------*/
/* Buttons */
/*---------------------------------------------------------------------------*/

#endif /* MODELS_H_ */
