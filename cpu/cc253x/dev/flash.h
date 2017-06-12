/*
 * flash.h
 *
 * Created on: June 11, 2017
 *     Author: Brian Blum
 *
 */

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

static void writeWordM( uint8 pg, uint16 offset, uint8 *buf, uint16 cnt );
void FlashWrite(uint16 addr, uint8 *buf, uint16 cnt);
void FlashErase(uint8 pg);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */
