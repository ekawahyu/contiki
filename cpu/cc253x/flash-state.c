/**
 * \file
 *         Flash Logger for managing logging of debug data. 
 *
 * \author
 *         Original: Brian Blum <brian.blum@conectric.com>
 *
 */

#include "flash-state.h"
#include "dev/flash.h" 

static uint8_t read_buf[FLASH_STATE_SIZE_MAX]; 
static uint32_t pg_addr_in_use;

#define HDR_SIZE    4
#define HDR_VALID   0xAA
#define HDR_INVALID 0x00
#define HDR_UNUSED  0xFF

#define NUM_WRDS(size) ((size & 0x03) > 0x00 ? (size >> 2) + 1 : (size >> 2))
#define FLASH_READ_SIZE(size) (NUM_WRDS(size)*FLASH_WRT_SZ)

struct state_header {
      uint8_t id;
      uint8_t size;
      uint8_t valid; // 0xAA valid; 0x00 invalid
      uint8_t reserved;
};

/*---------------------------------------------------------------------------*/
void
flashstate_init()
{
  struct state_header hdr;  

  // check PG1 for in_use
  flash_read(FLASH_PAGE(FLASH_STATE_PG1_ADDR), FLASH_PAGE_OFFSET(FLASH_STATE_PG1_ADDR), (uint8_t *)&hdr, HDR_SIZE);
  if(hdr.id == FLASH_STATE_PG_HEADER) 
  {
    // PG1 in use
    pg_addr_in_use = FLASH_STATE_PG1_ADDR;
    flash_page_erase(FLASH_PAGE(FLASH_STATE_PG2_ADDR));  // shouldn't be needed, just in case
  }
  else 
  {
    // check PG2 for in_use
    flash_read(FLASH_PAGE(FLASH_STATE_PG2_ADDR), FLASH_PAGE_OFFSET(FLASH_STATE_PG2_ADDR), (uint8_t *)&hdr, HDR_SIZE);
    if(hdr.id == FLASH_STATE_PG_HEADER) 
    {
      // PG2 in use
      pg_addr_in_use = FLASH_STATE_PG2_ADDR;
      flash_page_erase(FLASH_PAGE(FLASH_STATE_PG1_ADDR));  // shouldn't be needed, just in case
    }
    else
    {
      // no PG initialized, so initialize
      pg_addr_in_use = FLASH_STATE_PG1_ADDR;
      hdr.id = FLASH_STATE_PG_HEADER;
      hdr.size = 0x00;
      hdr.valid = HDR_VALID;
      flash_write_DMA((uint8_t *)&hdr, HDR_SIZE, FLASH_WORD_ADDR(FLASH_STATE_PG1_ADDR));
      flash_page_erase(FLASH_PAGE(FLASH_STATE_PG2_ADDR));  // not needed since this should come off a firmware upgrade
    }
  }
}
/*---------------------------------------------------------------------------*/

// Write state information to flash
void flashstate_write(uint8_t stateId, uint8_t *data, uint8_t size)
{    
  struct state_header hdr;
  uint32_t flash_addr;
  
  // search for existing entry to invalidate
  flash_addr = pg_addr_in_use + HDR_SIZE;
  
  while (flash_addr < (pg_addr_in_use + FLASH_PAGE_SIZE))
  {
    flash_read(FLASH_PAGE(flash_addr), FLASH_PAGE_OFFSET(flash_addr), (uint8_t *)&hdr, HDR_SIZE);
    if(hdr.valid == HDR_VALID && hdr.id == stateId)
    {
      // found existing valid entry, invalidate (write over header valid field)
      hdr.valid = HDR_INVALID;     
      flash_write_DMA((uint8_t *)&hdr, HDR_SIZE, FLASH_WORD_ADDR(flash_addr));
      // increment address pointer by size of structure
      flash_addr += HDR_SIZE + FLASH_READ_SIZE(hdr.size);     
    } 
    else if (hdr.valid != HDR_UNUSED) 
    {
      flash_addr += HDR_SIZE + FLASH_READ_SIZE(hdr.size);
    }
    else
    {
      // unused, check to ensure that writing state does not take us beyond the page boundary
      if(FLASH_PAGE_OFFSET(flash_addr) + HDR_SIZE + size >= FLASH_PAGE_SIZE)
      {
        // goes beyond boundary, write valid entries to new page and switch pages
        uint32_t next_page = (pg_addr_in_use == FLASH_STATE_PG1_ADDR) ? FLASH_STATE_PG2_ADDR : FLASH_STATE_PG1_ADDR;
        uint32_t next_addr;
        
        // write header
        hdr.id = FLASH_STATE_PG_HEADER;
        hdr.size = 0x00;
        hdr.valid = HDR_VALID;
        flash_write_DMA((uint8_t *)&hdr, HDR_SIZE, FLASH_WORD_ADDR(next_page));
          
        // copy over valid entries
        flash_addr = pg_addr_in_use + HDR_SIZE;
        next_addr = next_page + HDR_SIZE;
        while (flash_addr < (pg_addr_in_use + FLASH_PAGE_SIZE))
        {
          flash_read(FLASH_PAGE(flash_addr), FLASH_PAGE_OFFSET(flash_addr), (uint8_t *)&hdr, HDR_SIZE);
          if(hdr.valid == HDR_VALID)
          {
            // valid, write to next page
            flash_write_DMA((uint8_t *)&hdr, HDR_SIZE, FLASH_WORD_ADDR(next_addr));
            next_addr += HDR_SIZE;
            flash_addr += HDR_SIZE;
              
            // read item
            flash_read(FLASH_PAGE(flash_addr), FLASH_PAGE_OFFSET(flash_addr), read_buf, FLASH_READ_SIZE(hdr.size));
            // write item to next page
            flash_write_DMA(read_buf, FLASH_READ_SIZE(hdr.size), FLASH_WORD_ADDR(next_addr));  
            
            // increment address pointers by size of structure
            flash_addr += FLASH_READ_SIZE(hdr.size);
            next_addr += FLASH_READ_SIZE(hdr.size); 
          } 
          else if (hdr.valid == HDR_INVALID) 
          {
            // skip entry
            flash_addr += HDR_SIZE + FLASH_READ_SIZE(hdr.size);
          }
          else // unused (end of used Flash)
          {
            // copy done, next_addr pointer in correct location
            // erase used page
            flash_page_erase(FLASH_PAGE(pg_addr_in_use));
            // update to point to new page
            pg_addr_in_use = next_page;
            flash_addr = next_addr;
            break;
          }
        }
      }

      break;  // hdr unused (free space), exit and write
    }
  }
  // flash_addr should be pointing to valid address to write to
  
  if(size <= FLASH_STATE_SIZE_MAX)
  {
    uint16_t wd; // for conversion to # words
    // write state (confirmed valid location)
    hdr.valid = HDR_VALID;
    hdr.id = stateId;
    hdr.size = size;
    // write header
    wd = NUM_WRDS(size);
    flash_write_DMA((uint8_t *)&hdr, HDR_SIZE, FLASH_WORD_ADDR(flash_addr));
    flash_addr += HDR_SIZE;
    flash_write_DMA(data, wd*FLASH_WRT_SZ, FLASH_WORD_ADDR(flash_addr));    // will write extra bytes for non-multiple 4, will handle on read
  }
}
/*---------------------------------------------------------------------------*/
// Read state from Flash
// Return: size of state information (0 means not found)
uint8_t flashstate_read(uint8_t stateId, uint8_t **data)
{
  struct state_header hdr;
  uint32_t flash_addr;

  // search for existing valid entry
  flash_addr = pg_addr_in_use + HDR_SIZE;
  
  while (flash_addr < (pg_addr_in_use + FLASH_PAGE_SIZE))
  {
    flash_read(FLASH_PAGE(flash_addr), FLASH_PAGE_OFFSET(flash_addr), (uint8_t *)&hdr, HDR_SIZE);
    if(hdr.valid == HDR_VALID && hdr.id == stateId)
    {
      flash_addr += HDR_SIZE;
      // found existing valid entry, read into buffer
      flash_read(FLASH_PAGE(flash_addr), FLASH_PAGE_OFFSET(flash_addr), read_buf, FLASH_READ_SIZE(hdr.size));
      
      // set return pointer
      *data = read_buf;
      return hdr.size;     
    } 
    else if (hdr.valid != HDR_UNUSED) 
    {
      flash_addr += HDR_SIZE + FLASH_READ_SIZE(hdr.size);
    }
    else
    {
      // unused, check to ensure that writing state does not take us beyond the page boundary
      break;  // hdr unused (free space), exit and write
    }
  }
  return 0;
}
