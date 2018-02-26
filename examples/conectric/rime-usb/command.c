/*
 * command.c
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

#include "contiki.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"
#include "command.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef CONECTRIC_VERSION_STRING
#define CONECTRIC_VERSION_STRING "Contiki-unknown"
#endif
#ifndef CONECTRIC_PROJECT_STRING
#define CONECTRIC_PROJECT_STRING "unknown"
#endif

#define REQUEST_HEADER_LEN    4

#define MESSAGE_BYTEREQ       1
#define MESSAGE_BYTECMD       2

#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *gmacp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *gmacp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *gmacp = (__code unsigned char *)0xFFE8;
#endif

/*---------------------------------------------------------------------------*/
linkaddr_t *
command_parser(void * incoming, uint8_t type)
{
  static linkaddr_t forward_addr;
  uint8_t * bytereq = (uint8_t *)incoming;
  uint8_t request;
  uint8_t seqno, mhops, hdrlen;
  uint8_t * header;
  int i;

  /*******************************************************/
  /***** INTERPRETING COMMAND LINES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * BYTECMD Protocol:
   * - It starts with any char, but '<'
   * - Non-capital letter inputs get capitalized automatically
   *
   */
  if (type == MESSAGE_BYTECMD) {

    /* Command line interpreter */
    if (bytereq[0] == 'M' && bytereq[1] == 'R') {
      gmacp = &X_IEEE_ADDR;
      putstring("MR:");
      for(i = 7; i >= 0; i--) puthex(gmacp[i]);
      putstring("\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'P') {
      dump_packet_buffer(BUFFER_PAYLOAD);
      putstring("DP:Ok\n");
    }

    else if (bytereq[0] == 'D' && bytereq[1] == 'B') {
      dump_packet_buffer(BUFFER_ALL);
      putstring("DB:Ok\n");
    }

    else if (bytereq[0] == 'V' && bytereq[1] == 'E' && bytereq[2] == 'R') {
      putstring("VER:");
      putstring(CONTIKI_VERSION_STRING "\n");
      putstring("VER:");
      putstring(CONECTRIC_PROJECT_STRING "\n");
    }

    else if (bytereq[0] == '@' && bytereq[1] == 'B' && bytereq[2] == 'O' && bytereq[3] == 'O' && bytereq[4] == 'T') {
      putstring("@BOOT:Rebooting...\n");
      while(1);
    }

    /* Unknown command */
    else {
      if (strlen(bytereq) != 0) {
        putstring(bytereq);
        putstring(":Bad command!\n");
      }
    }

  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
