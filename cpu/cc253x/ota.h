/*
 * ota.h
 *
 * Created on: Sept 14, 2017
 *     Author: Brian Blum
 *
 */

#include <stdint.h>

#ifndef OTA_H_
#define OTA_H_

// largest supported segment for OTA image update
#define OTA_SEGMENT_MAX 64

// image version
#define OTA_NO_IMG 0x0000

// Default start address for cyclical write to Flash (Bank 2)
#ifndef OTA_FLASH_START
#define OTA_FLASH_START 0x10000
#endif

// Default end address for storing flash image)
#ifndef OTA_FLASH_END
#define OTA_FLASH_END   0x20000
#endif


void ota_clear(void);
void ota_flashwrite(uint16_t addr, uint8_t size, uint8_t *data);
void ota_copyandreset(void);

#endif /* OTA_H_ */
