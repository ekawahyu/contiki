/**
 * \file
 *         Driver for cc2530 Flash Read/Write. 
 *
 * \author
 *         Original: Brian Blum <brian.blum@conectric.com>
 *         Port: Zach Shelby <zach@sensinode.com>
 *         Further Modifications:
 *               George Oikonomou <oikonomou@users.sourceforge.net>
 *
 */

#include "dev/flash.h"
#include "contiki.h"
#include "cc253x.h"

/*********************************************************************
 * @fn      writeWordM
 *
 * @brief   Writes multiple Flash-WORDs to NV.
 *
 * @param   pg - A valid NV Flash page.
 * @param   offset - A valid offset into the page.
 * @param   buf - Pointer to source buffer.
 * @param   cnt - Number of 4-byte blocks to write.
 *
 * @return  none
 */
static void writeWordM( uint8 pg, uint16 offset, uint8 *buf, uint16 cnt )
{
  offset = (offset / HAL_FLASH_WORD_SIZE) +
          ((uint16)pg * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE));
  FlashWrite(offset, buf, cnt);
}

/**************************************************************************************************
 * @fn          FlashWrite
 *
 * @brief       This function writes 'cnt' bytes to the internal flash.
 *
 * input parameters
 *
 * @param       addr - Valid flash write address: actual addr / 4 and quad-aligned.
 * @param       buf - Valid buffer space at least as big as 'cnt' X 4.
 * @param       cnt - Number of 4-byte blocks to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void FlashWrite(uint16 addr, uint8 *buf, uint16 cnt)
{
#if (defined HAL_DMA) && (HAL_DMA == TRUE)
  halDMADesc_t *ch = HAL_NV_DMA_GET_DESC();

  HAL_DMA_SET_SOURCE(ch, buf);
  HAL_DMA_SET_DEST(ch, &FWDATA);
  HAL_DMA_SET_VLEN(ch, HAL_DMA_VLEN_USE_LEN);
  HAL_DMA_SET_LEN(ch, (cnt * HAL_FLASH_WORD_SIZE));
  HAL_DMA_SET_WORD_SIZE(ch, HAL_DMA_WORDSIZE_BYTE);
  HAL_DMA_SET_TRIG_MODE(ch, HAL_DMA_TMODE_SINGLE);
  HAL_DMA_SET_TRIG_SRC(ch, HAL_DMA_TRIG_FLASH);
  HAL_DMA_SET_SRC_INC(ch, HAL_DMA_SRCINC_1);
  HAL_DMA_SET_DST_INC(ch, HAL_DMA_DSTINC_0);
  // The DMA is to be polled and shall not issue an IRQ upon completion.
  HAL_DMA_SET_IRQ(ch, HAL_DMA_IRQMASK_DISABLE);
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS);
  HAL_DMA_SET_PRIORITY(ch, HAL_DMA_PRI_HIGH);
  HAL_DMA_CLEAR_IRQ(HAL_NV_DMA_CH);
  HAL_DMA_ARM_CH(HAL_NV_DMA_CH);

  FADDRL = (uint8)addr;
  FADDRH = (uint8)(addr >> 8);
  FCTL |= 0x02;         // Trigger the DMA writes.
  while (FCTL & 0x80);  // Wait until writing is done.
#endif
}

/**************************************************************************************************
 * @fn          FlashErase
 *
 * @brief       This function erases the specified page of the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number to erase.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void FlashErase(uint8 pg)
{
  FADDRH = pg * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE / 256);
  FCTL |= 0x01;
}

