/*
 * project-conf.h
 *
 * Created on: Mar 3, 2014
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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STARTUP_CONF_VERBOSE                  1
#define MODELS_CONF_ANAREN_A2530E_MODULE      1

#define NETSTACK_CONF_MAC                     nullmac_driver
#define NETSTACK_CONF_RDC                     nullrdc_driver

#define SINK_CONF_ENTRIES 16
#define SINK_CONF_DEFAULT_LIFETIME 180 /* default life time max = 255 seconds */

#define IEEE802154_CONF_PANID                 0x2007
#define CC2530_RF_CONF_CHANNEL                25
#define CC2530_RF_CONF_LEDS                   1
#if MODELS_CONF_ANAREN_A2530E_MODULE
#else
#define CC2530_RF_CONF_LOW_POWER_RX           1    /* set to 1 to conserve power during reception */
#define CC2530_RF_CONF_TX_POWER               0xD5 /* tx power range: 0x05 - 0xD5(the highest) */
#endif

#define LPM_CONF_MODE                         0

#define RS485_CONF_ENABLE                     1
#define UART1_CONF_ENABLE                     1

#define BUTTON_SENSOR_CONF_ON                 0

#define CONECTRIC_BURST_NUMBER                1

#define COMMAND_HAS_DUMP_PACKET_BUFFER        1
#define COMMAND_HAS_CONECTRIC_NETSTACK        1

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CONF_H_ */
