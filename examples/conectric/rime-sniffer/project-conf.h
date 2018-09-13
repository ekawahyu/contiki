/*
 * project-conf.h
 *
 * Created on: Aug 8, 2018
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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#define STARTUP_CONF_VERBOSE                  1
#define MODELS_CONF_ANAREN_A2530E_MODULE      1

#define NETSTACK_CONF_MAC                     nullmac_driver
#define NETSTACK_CONF_RDC                     stub_rdc_driver

#define IEEE802154_CONF_PANID                 0xdede
#define CC2530_RF_CONF_CHANNEL                25
#define CC2530_RF_CONF_LEDS                   1
#if MODELS_CONF_ANAREN_A2530E_MODULE
#else
#define CC2530_RF_CONF_LOW_POWER_RX           1    /* set to 1 to conserve power during reception */
#define CC2530_RF_CONF_TX_POWER               0xD5 /* tx power range: 0x05 - 0xD5(the highest) */
#endif

#define CC2530_RF_CONF_HEXDUMP 1
#define CC2530_RF_CONF_AUTOACK 0

#define LPM_CONF_MODE                         0

#define RS485_CONF_ENABLE                     0
#define UART1_CONF_ENABLE                     0

#define BUTTON_SENSOR_CONF_ON                 0

#define CONECTRIC_BURST_NUMBER                1

#endif /* PROJECT_CONF_H_ */
