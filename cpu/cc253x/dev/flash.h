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
#define ADDR_PAGE(addr) (addr / FLASH_PAGE_SIZE)
#define ADDR_OFFSET(addr) (addr % FLASH_PAGE_SIZE)

void Flash_WriteDMA(uint8_t *data, uint16_t length, uint16_t flashwordadr);
void Flash_Read(uint8_t pg, uint16_t offset, uint8_t *buf, uint16_t size);
void Flash_PageErase(uint8_t pg);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */
