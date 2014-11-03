#include "contiki.h"
#include "soc.h"
#include "stack.h"
#include "sys/clock.h"
#include "sys/autostart.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/gpio.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/io-arch.h"
#include "dev/dma.h"
#include "dev/cc2530-rf.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "dev/clock-isr.h"
#include "dev/port2.h"
#include "dev/lpm.h"
#include "lsm330dlc-sensor.h"
#include "ad7689-sensor.h"
#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/leds-arch.h"
#include "net/rime/rime.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
#include "debug.h"
#include "cc253x.h"
#include "sfr-bits.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <stdio.h>
int mesh_is_connected(void); /* TODO clean up this function prototype */
/*---------------------------------------------------------------------------*/
#if VIZTOOL_CONF_ON
PROCESS_NAME(viztool_process);
#endif
/*---------------------------------------------------------------------------*/
#if STARTUP_CONF_VERBOSE
#define PUTSTRING(...) putstring(__VA_ARGS__)
#define PUTHEX(...) puthex(__VA_ARGS__)
#define PUTBIN(...) putbin(__VA_ARGS__)
#define PUTCHAR(...) putchar(__VA_ARGS__)
#else
#define PUTSTRING(...)
#define PUTHEX(...)
#define PUTBIN(...)
#define PUTCHAR(...)
#endif
/*---------------------------------------------------------------------------*/
#if CLOCK_CONF_STACK_FRIENDLY
extern volatile uint8_t sleep_flag;
#endif
/*---------------------------------------------------------------------------*/
extern linkaddr_t linkaddr_node_addr;
static CC_AT_DATA uint16_t len;
/*---------------------------------------------------------------------------*/
#if ENERGEST_CONF_ON
static unsigned long irq_energest = 0;
#define ENERGEST_IRQ_SAVE(a) do { \
    a = energest_type_time(ENERGEST_TYPE_IRQ); } while(0)
#define ENERGEST_IRQ_RESTORE(a) do { \
    energest_type_set(ENERGEST_TYPE_IRQ, a); } while(0)
#else
#define ENERGEST_IRQ_SAVE(a) do {} while(0)
#define ENERGEST_IRQ_RESTORE(a) do {} while(0)
#endif
/*---------------------------------------------------------------------------*/
static void
fade(int l) CC_NON_BANKED
{
  volatile int i, a;
  int k, j;
  for(k = 0; k < 400; ++k) {
    j = k > 200 ? 400 - k : k;

    leds_on(l);
    for(i = 0; i < j; ++i) {
      a = i;
    }
    leds_off(l);
    for(i = 0; i < 200 - j; ++i) {
      a = i;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
fade_fast(int l) CC_NON_BANKED
{
  volatile int i, a;
  int k, j;
  for(k = 0; k < 200; ++k) {
    j = k > 100 ? 200 - k : k;

    leds_on(l);
    for(i = 0; i < j; ++i) {
      a = i;
    }
    leds_off(l);
    for(i = 0; i < 100 - j; ++i) {
      a = i;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_rf_params(void) CC_NON_BANKED
{
  signed char i;
  uint16_t short_addr;
  uint8_t ext_addr[8];

#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *macp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *macp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *macp = (__code unsigned char *)0xFFE8;
#endif

  PUTSTRING("Rime is 0x");
  PUTHEX(sizeof(linkaddr_t));
  PUTSTRING(" bytes long\n");

#if CC2530_CONF_MAC_FROM_PRIMARY
  PUTSTRING("Reading MAC from Info Page\n");
#else
  PUTSTRING("Reading MAC from flash\n");

  /*
   * The MAC is always stored in 0xFFE8 of the highest BANK of our flash. This
   * maps to address 0xFFF8 of our CODE segment, when this BANK is selected.
   * Load the bank, read 8 bytes starting at 0xFFE8 and restore last BANK.
   * Since we are called from main(), this MUST be BANK1 or something is very
   * wrong. This code can be used even without a bankable firmware.
   */

  /* Don't interrupt us to make sure no BANK switching happens while working */
  DISABLE_INTERRUPTS();

  /* Switch to the BANKn,
   * map CODE: 0x8000 - 0xFFFF to FLASH: 0xn8000 - 0xnFFFF */
  FMAP = CC2530_LAST_FLASH_BANK;
#endif

  /*
   * Read IEEE address from flash, store in ext_addr.
   * Invert endianness (from little to big endian)
   */
  for(i = 7; i >= 0; --i) {
    ext_addr[i] = *macp;
    macp++;
  }

#if !CC2530_CONF_MAC_FROM_PRIMARY
  /* Remap 0x8000 - 0xFFFF to BANK1 */
  FMAP = 1;
  ENABLE_INTERRUPTS();
#endif

  short_addr = ext_addr[7];
  short_addr |= ext_addr[6] << 8;

  /* Populate linkaddr_node_addr. Maintain endianness */
  memcpy(&linkaddr_node_addr, &ext_addr[8 - LINKADDR_SIZE], LINKADDR_SIZE);

  /* Now the address is stored MSB first */
#if STARTUP_CONF_VERBOSE
  PUTSTRING("Rime configured with address ");
  for(i = 0; i < LINKADDR_SIZE - 1; i++) {
    PUTHEX(linkaddr_node_addr.u8[i]);
    PUTCHAR(':');
  }
  PUTHEX(linkaddr_node_addr.u8[i]);
  PUTCHAR('\n');
#endif

  /* Write params to RF registers */
  NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
  NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, short_addr);
  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, CC2530_RF_CHANNEL);
  NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, ext_addr, 8);
  return;
}
/*---------------------------------------------------------------------------*/
int
main(void) CC_NON_BANKED
{
  /* Hardware initialization */
  clock_init();
  soc_init();
  rtimer_init();

  stack_poison();

  gpio_init();
  leds_init();
  leds_off(LEDS_ALL);

  spi_init();

  /* initialize process manager. */
  process_init();

#if DMA_ON
  dma_init();
#endif

  io_arch_init();

#if SLIP_ARCH_CONF_ENABLE
  slip_arch_init(0);
#else
  io_arch_set_input(serial_line_input_byte);
  serial_line_init();
#endif

  PUTSTRING("##########################################\n");
  putstring(CONTIKI_VERSION_STRING "\n");
  putstring(MODEL_STRING);
  switch(CHIPID) {
  case 0xA5:
    putstring("cc2530");
    break;
  case 0xB5:
    putstring("cc2531");
    break;
  case 0x95:
    putstring("cc2533");
    break;
  case 0x8D:
    putstring("cc2540");
    break;
  }

  putstring("-" CC2530_FLAVOR_STRING ", ");
  puthex(CHIPINFO1 + 1);
  putstring("KB SRAM\n");

  PUTSTRING("\nSDCC Build:\n");
#if STARTUP_CONF_VERBOSE
#ifdef HAVE_SDCC_BANKING
  PUTSTRING("  With Banking.\n");
#endif /* HAVE_SDCC_BANKING */
#ifdef SDCC_MODEL_LARGE
  PUTSTRING("  --model-large\n");
#endif /* SDCC_MODEL_LARGE */
#ifdef SDCC_MODEL_HUGE
  PUTSTRING("  --model-huge\n");
#endif /* SDCC_MODEL_HUGE */
#ifdef SDCC_STACK_AUTO
  PUTSTRING("  --stack-auto\n");
#endif /* SDCC_STACK_AUTO */

  PUTCHAR('\n');

  PUTSTRING(" Net: ");
  PUTSTRING(NETSTACK_NETWORK.name);
  PUTCHAR('\n');
  PUTSTRING(" MAC: ");
  PUTSTRING(NETSTACK_MAC.name);
  PUTCHAR('\n');
  PUTSTRING(" RDC: ");
  PUTSTRING(NETSTACK_RDC.name);
  PUTCHAR('\n');

  PUTSTRING("##########################################\n");
#endif

  watchdog_init();

  /* Initialise the H/W RNG engine. */
  random_init(0);

  /* start services */
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* initialize the netstack */
  netstack_init();
  set_rf_params();

#if BUTTON_SENSOR_ON || ADC_SENSOR_ON || LSM330DLC_SENSOR_ON
  process_start(&sensors_process, NULL);
#endif
#if BUTTON_SENSOR_ON
  BUTTON_SENSOR_ACTIVATE();
#endif
#if ADC_SENSOR_ON
  ADC_SENSOR_ACTIVATE();
#endif
#if LSM330DLC_SENSOR_ON
  LSM330DLC_SENSOR_ACTIVATE();
#endif
#if AD7689_SENSOR_ON
  AD7689_SENSOR_ACTIVATE();
#endif

#if UIP_CONF_IPV6
  memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
  queuebuf_init();
  process_start(&tcpip_process, NULL);
#endif /* UIP_CONF_IPV6 */

#if VIZTOOL_CONF_ON
  process_start(&viztool_process, NULL);
#endif

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  autostart_start(autostart_processes);

  watchdog_start();

  /* TODO not supposed to be here for RC2400HP */
#if MODELS_CONF_RC2400HP_MODULE
  RFC_OBS_CTRL0 = 0x68;
  OBSSEL1 = 0xFB;
  RFC_OBS_CTRL1 = 0x6A;
  OBSSEL3 = 0xFC;
#endif

  while(1) {
    uint8_t r;
    do {
      /* Reset watchdog and handle polls and events */
      watchdog_periodic();

#if CLOCK_CONF_STACK_FRIENDLY
      if(sleep_flag) {
        if(etimer_pending() &&
            (etimer_next_expiration_time() - clock_time() - 1) > MAX_TICKS) {
          etimer_request_poll();
        }
        sleep_flag = 0;
      }
#endif
      r = process_run();
    } while(r > 0);
    len = NETSTACK_RADIO.pending_packet();
    if(len) {
      packetbuf_clear();
      len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);
      if(len > 0) {
        packetbuf_set_datalen(len);
        NETSTACK_RDC.input();
      }
    }
#if LPM_MODE
    if (rtimer_is_scheduled() || mesh_is_connected() == 0) {
      /* keep the system awake */
    }
    else {
      /* Making sure that the next sleep timer interrupt happens at least 3ms
       * after sleep command is issued. Otherwise, the system will go to sleep
       * and never wake up again.
       *
       * If the system runs on 32kHz --> 96 counts ~3ms
       * If the system runs on 32.768kHz ---> 98 counts ~3ms
       * CLOCK_SECOND = 128 counts (see TICK_VAL as well)
       *
       * The sleep timer is being used as a systick, therefore, adding a new
       * value to it may affect every thread running with etimer. The workaround
       * for the moment is to skip ahead one ISR and manually adjust the systick
       * ahead of time. One tick adjustment is equivalent to adding 7.8ms
       */
      clock_adjust_systick_ahead_by(1*CLOCK_SECOND);

      /*
       * Set MCU IDLE or Drop to PM1. Any interrupt will take us out of LPM
       * Sleep Timer will wake us up in no more than 7.8ms (max idle interval)
       */
      SLEEPCMD = (SLEEPCMD & 0xFC) | LPM_MODE;

      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);

      /* We are only interested in IRQ energest while idle or in LPM */
      ENERGEST_IRQ_RESTORE(irq_energest);

      /* Go IDLE or Enter PM1 */
      PCON |= PCON_IDLE;

      /* First instruction upon exiting PM1 must be a NOP
       * There is no harm in adding more NOP and it seems that
       * lock up does not occur anymore in PM1 with two additional NOP
       */
      __asm_begin
        ASM(nop)
        ASM(nop)
        ASM(nop)
      __asm_end;

      //fade_fast(LEDS_RED);

      /* Remember energest IRQ for next pass */
      ENERGEST_IRQ_SAVE(irq_energest);

      ENERGEST_ON(ENERGEST_TYPE_CPU);
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
    }
#endif /* LPM_MODE */
  }
}
/*---------------------------------------------------------------------------*/
