/*
 * command.h
 *
 * Created on: Feb 26, 2018
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

#ifndef COMMAND_H_
#define COMMAND_H_

#include "conectric-messages.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUFFER_PAYLOAD        0
#define BUFFER_ALL            1

void hexstring_to_bytes(uint8_t * hexstring, uint8_t * bytes);
void dump_packet_buffer(uint8_t mode);
uint8_t * command_interpreter(uint8_t * command_line);

void compose_request_to_packetbuf(uint8_t * request, uint8_t seqno, uint8_t batt, linkaddr_t * ereceiver);
//void compose_request_line_to_packetbuf(request_line * line, uint8_t seqno, uint8_t batt, linkaddr_t * ereceiver);
//void request_line_create(request_line * line, uint8_t request, linkaddr_t * dest, uint8_t * data, uint8_t datalen);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_H_ */
