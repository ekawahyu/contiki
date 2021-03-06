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
 *         Override core/dev/button-sensor.h
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 *         Ekawahyu Susilo - <ekawahyu@yahoo.com>
 */

#ifndef BUTTON_SENSOR_H_
#define BUTTON_SENSOR_H_

#include "contiki-conf.h"
#include "lib/sensors.h"

#define BUTTON_SENSOR "Button"

#define BUTTON1_PORT 1
#define BUTTON1_PIN  2
#define BUTTON2_PORT 2
#define BUTTON2_PIN  0

#ifdef BUTTON_SENSOR_CONF_ON
#define BUTTON_SENSOR_ON BUTTON_SENSOR_CONF_ON
#endif /* BUTTON_SENSOR_CONF_ON */

#ifdef BUTTON_SENSOR_CONF_INPUT_3STATE
#define BUTTON_SENSOR_INPUT_3STATE BUTTON_SENSOR_CONF_INPUT_3STATE
#else
#define BUTTON_SENSOR_INPUT_3STATE    0
#endif /* BUTTON_SENSOR_CONF_INPUT_3STATE */

extern const struct sensors_sensor button_1_sensor;
extern const struct sensors_sensor button_2_sensor;

#if BUTTON_SENSOR_ON
#if defined __IAR_SYSTEMS_ICC__
__near_func __interrupt void port_2_isr(void);
__near_func __interrupt void port_1_isr(void);
#else
void port_2_isr(void) __interrupt(P2INT_VECTOR);
void port_1_isr(void) __interrupt(P1INT_VECTOR);
#endif
#define   BUTTON_SENSOR_ACTIVATE(b) do { \
    button_##b##_sensor.configure(SENSORS_ACTIVE, 1); \
} while(0)
#define   BUTTON_SENSOR_DEACTIVATE(b) do { \
    button_##b##_sensor.configure(SENSORS_ACTIVE, 0); \
} while(0)
#else /* BUTTON_SENSOR_ON */
#define   BUTTON_SENSOR_ACTIVATE(b)
#define   BUTTON_SENSOR_DEACTIVATE(b)
#endif /* BUTTON_SENSOR_ON */

/* Define macros for buttons */
#define BUTTON_READ(b)           PORT_READ(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_FUNC_GPIO(b)      PORT_FUNC_GPIO(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_DIR_INPUT(b)      PORT_DIR_INPUT(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_PULLUPDOWN_3STATE(b) PORT_PULLUPDOWN_3STATE(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_ENABLED(b)    PORT_IRQ_ENABLED(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_CHECK(b)      PORT_IRQ_CHECK(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_ENABLE(b)     PORT_IRQ_ENABLE(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_DISABLE(b)    PORT_IRQ_DISABLE(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_FLAG_OFF(b)   PORT_IRQ_FLAG_OFF(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_ON_PRESS(b)   PORT_IRQ_EDGE_RISE(BUTTON##b##_PORT, BUTTON##b##_PIN)
#define BUTTON_IRQ_ON_RELEASE(b) PORT_IRQ_EDGE_FALL(BUTTON##b##_PORT, BUTTON##b##_PIN)

#endif /* BUTTON_SENSOR_H_ */
