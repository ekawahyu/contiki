/**
 * \file
 *         Read Modbus address from Archmeter power meter using Modbus protocols
 * \author
 *         Germán Ramos <german.ramos@sensingcontrol.com>
 *         Joakim Eriksson <joakime@sics.se>
 * \company
 *                 Sensing & Control Systems S.L.
 */

/**
 * \brief  This is the main file of modbus protocol
 *
 * We expect:
 *    TX: 0F 04 10 C6 00 02 94 18
 *    RX: 0F 04 04 5A 39 40 1C E6 98
 *
 * This modbus packet will be transmitted over RS485
 */

#include "contiki.h"
#include "dev/hwconf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "leds.h"

#include "modbus-api.h"
#include "rs485-dev.h"

/*---------------------------------------------------------------------------*/
PROCESS(modbus_test_process, "Modbus tester");
AUTOSTART_PROCESSES(&modbus_test_process);
/*---------------------------------------------------------------------------*/
/***********************************************************************/

/**
 * \brief  Modbus main function
 * \param  void
 * \return 0
 */
static unsigned int registers[] = {0x10c6, 0x10cc, 0x10ca, 0x109e, 0x10a6};

PROCESS_THREAD(modbus_test_process, ev, data)
{
  static int i;
  static struct etimer et;
  int rv = 0;
  PROCESS_BEGIN();

  printf("Starting up tester...\n");
  modbus_init();

  P2DIR |= 0x40;
  P7DIR |= 4;

  while(1) {

    printf("Waiting 10 sec.\n");
    etimer_set(&et, 10 * CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    leds_invert(LEDS_GREEN);

    P7OUT ^= 4;
    P2OUT ^= 0x40;

    /* Modbus register address: address_kW_deliver = 0x10c6;
       address_kVar_leading = 0x10cc;
       address_kVarh_lagging = 0x10ca;
       address_kW_total = 0x109e;
       address_kVar_total = 0x10a6;

       Hardware device: Archmeter PA310 */

    for(i = 0; i < 5; i++) {
      rv = modbus_read_register(registers[i]);
      if(rv == 0) {
	float f = modbus_get_float(0);
        printf("Success state after sending modbus packet\n\r");
        printf("Value read: %x = %f \n", registers[i], f);
        printf("Value %f, %.3f\n", f, f);
      } else if(rv == -1) {
        printf("Error state after sending modbus packet: -1\n\r");
      } else {
        printf("Error state after sending modbus packet: %d\n\r", rv);
      }
    }
  }
  PROCESS_END();
}
