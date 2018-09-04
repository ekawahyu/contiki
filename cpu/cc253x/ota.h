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

/* Largest supported segment for OTA image update */
#define OTA_SEGMENT_MAX 64
#define OTA_UPDATE_MAP_SHIFT 6 /* this must change for segment map index if segment size changes */

/* Image macros */
#define OTA_NO_IMG 0x0000

/* For storing the segment map structure */
#define OTA_SEGMAP_BYTES 128 /* (((OTA_FLASH_START - OTA_FLASH_END) / OTA_SEGMENT_MAX) / 8) */

/* Start / end address for working code image */
#ifndef OTA_IMG_START
#define OTA_IMG_START (uint32_t)0x00000
#endif
#ifndef OTA_IMG_END
#define OTA_IMG_END   (uint32_t)0x10000
#endif

/* Default start / end address for cyclical write to Flash (Bank 2) */
#ifndef OTA_FLASH_START
#define OTA_FLASH_START (uint32_t)0x10000
#endif
#ifndef OTA_FLASH_END
#define OTA_FLASH_END   (uint32_t)0x20000
#endif

/* OTA Packet */
#define OTA_IMG_SUCCESS         0
#define OTA_IMG_INVALID_CRC     1
#define OTA_IMG_INCOMPLETE      2
#define OTA_IMG_INVALID_VER     3

/* OTA image segment map */
extern uint8_t img_segment_map[OTA_SEGMAP_BYTES];
extern uint16_t ota_img_version;

void ota_init(void);
void ota_update_map(uint16_t addr);
void ota_restore_map(void);
uint8_t ota_next_segment(uint16_t img_size, uint16_t *next_addr);
void ota_clear(void);
void ota_flashwrite(uint16_t addr, uint8_t size, uint8_t *data);
void ota_copyandreset(void * arg);
uint8_t ota_verify_crc(uint16_t size, uint16_t crc);

#endif /* OTA_H_ */
