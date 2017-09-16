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
#define OTA_UPDATE_MAP_SHIFT 6 // this must change for segment map index if segment size changes
// image macros
#define OTA_NO_IMG 0x0000
// for storing the segment map structure
#define OTA_SEGMAP_BYTES 128 // (((OTA_FLASH_START - OTA_FLASH_END) / OTA_SEGMENT_MAX) / 8)

// Default start address for cyclical write to Flash (Bank 2)
#ifndef OTA_FLASH_START
#define OTA_FLASH_START (uint32_t)0x10000
#endif

// Default end address for storing flash image)
#ifndef OTA_FLASH_END
#define OTA_FLASH_END   (uint32_t)0x20000
#endif

// ota image segment map
extern uint8_t img_segment_map[OTA_SEGMAP_BYTES];

void ota_init(void);
void ota_update_map(uint16_t addr);
void ota_restore_map(void);
void ota_clear(void);
void ota_flashwrite(uint16_t addr, uint8_t size, uint8_t *data);
void ota_copyandreset(void);

#endif /* OTA_H_ */
