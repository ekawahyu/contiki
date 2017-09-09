/*
 * flash.h
 *
 * Created on: June 11, 2017
 *     Author: Brian Blum
 *
 */

#include <stdint.h>

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

/* FLASH Globals */
/*---------------*/
// CODE banks get mapped into the XDATA range 8000-FFFF.
#define FLASH_PAGE_MAP         0x8000
#define FLASH_WORD_ADDR(addr) (addr >> 2)
#define FLASH_PAGE_SIZE 2048
#define FLASH_BANK_PAGES 16
#define FLASH_PAGE(addr) (addr / FLASH_PAGE_SIZE)
#define FLASH_PAGE_OFFSET(addr) (addr % FLASH_PAGE_SIZE)
#define FLASH_WRT_SZ 4

void flash_write_DMA(uint8_t *data, uint16_t length, uint16_t flashwordadr);
void flash_read(uint8_t pg, uint16_t offset, uint8_t *buf, uint16_t size);
void flash_page_erase(uint8_t pg);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */
