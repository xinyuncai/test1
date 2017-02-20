/** @} */

/**
 * @brief       : 
 *
 * @file        : cfs-coffee_arch.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#include "cfs-coffee.h"
#include "cfs-coffee-arch.h"
#include "pstorage.h"
#include "semphr.h"

#include <stdint.h>

#ifndef COFFEE_CONF_CUSTOM_PORT
/*---------------------------------------------------------------------------*/
#if !COFFEE_SECTOR_SIZE || COFFEE_SECTOR_SIZE % FLASH_PAGE_SIZE
#error COFFEE_SECTOR_SIZE must be a non-zero multiple of the flash page size
#endif
#if !COFFEE_PAGE_SIZE || COFFEE_SECTOR_SIZE % COFFEE_PAGE_SIZE
#error COFFEE_PAGE_SIZE must be a divisor of COFFEE_SECTOR_SIZE
#endif
#if COFFEE_PAGE_SIZE % FLASH_WORD_SIZE
#error COFFEE_PAGE_SIZE must be a multiple of the flash word size
#endif
#if COFFEE_START % FLASH_PAGE_SIZE
#error COFFEE_START must be aligned with a flash page boundary
#endif
#if COFFEE_SIZE % COFFEE_SECTOR_SIZE
#error COFFEE_SIZE must be a multiple of COFFEE_SECTOR_SIZE
#endif
#if COFFEE_SIZE / COFFEE_PAGE_SIZE > INT16_MAX
#error Too many Coffee pages for coffee_page_t
#endif
#if COFFEE_START < NRF52_DEV_FLASH_ADDR || \
    COFFEE_START + COFFEE_SIZE > FLASH_CCA_ADDR
#error Coffee does not fit in flash
#endif
/*---------------------------------------------------------------------------*/
void
cfs_coffee_arch_erase(uint16_t sector)
{
  // 范围检测
  if(COFFEE_START + sector * COFFEE_SECTOR_SIZE >= COFFEE_END)
  {
    return;
  }
//  taskENTER_CRITICAL();
  pstorage_handle_t pstorage_cfs;
  pstorage_cfs.module_id = 0;
  pstorage_cfs.block_id = COFFEE_START + sector * COFFEE_SECTOR_SIZE;
	 
  cmd_queue_enqueue(PSTORAGE_CLEAR_OP_CODE, &pstorage_cfs, NULL, 4096, 0);  
//  nrf_nvmc_page_erase(COFFEE_START + sector * COFFEE_SECTOR_SIZE); 
//  taskEXIT_CRITICAL();
}

void
cfs_coffee_erase(uint16_t sector)
{
  // 范围检测
  if(COFFEE_START + sector * COFFEE_SECTOR_SIZE >= COFFEE_END)
  {
    return;
  }
  taskENTER_CRITICAL(); 
  nrf_nvmc_page_erase(COFFEE_START + sector * COFFEE_SECTOR_SIZE); 
  taskEXIT_CRITICAL();
}
/*---------------------------------------------------------------------------*/
void
cfs_coffee_arch_write(const void *buf, unsigned int size, cfs_offset_t offset)
{   
#if 0
    uint8_t *dst; 
    const uint8_t *src;
    int write_start_address; 
    uint8_t read_src[COFFEE_SECTOR_SIZE];  
    uint8_t flash_addr = COFFEE_START + offset;    

    src = (const void *)(((COFFEE_START + offset) / COFFEE_START) * COFFEE_START);
    write_start_address = ((COFFEE_START + offset) / COFFEE_START) * COFFEE_START;
    
    // 范围检测
    if((write_start_address < COFFEE_START) || ((write_start_address + COFFEE_SECTOR_SIZE) > COFFEE_END))
    {
        return;
    }

    taskENTER_CRITICAL();   
    
    // 读取一个sector数据(4K) 
    for(int i = 0; i < COFFEE_SECTOR_SIZE; i++)
    {
        read_src[i] = *src++;
    }

    // 更新一个sector数据(4K)    
    for(dst = (uint8_t *)buf; size; size--)
    {
        read_src[flash_addr++] = ~*dst++;
    }

    // 擦除一个sector数据(4K)
    nrf_nvmc_page_erase(write_start_address);

    // 写入一个sector数据(4K)
    nrf_nvmc_write_bytes(write_start_address, read_src, COFFEE_SECTOR_SIZE);
    
    taskEXIT_CRITICAL();

#else // 不更新数据   只追加
    uint32_t flash_addr = COFFEE_START + offset;
    const uint8_t *src = buf;
    uint8_t page_buf[COFFEE_SECTOR_SIZE];
    for(int i = 0; i < size && size <= COFFEE_SECTOR_SIZE; i++)
    {
      page_buf[i] = ~*src++;
    }
    
//    nrf_nvmc_write_bytes(flash_addr, page_buf, size);

     pstorage_handle_t pstorage_cfs;
     pstorage_cfs.module_id = 0;
     pstorage_cfs.block_id = flash_addr;
     cmd_queue_enqueue(PSTORAGE_STORE_OP_CODE, &pstorage_cfs, page_buf, size, 0);  
#endif
}
/*---------------------------------------------------------------------------*/
void
cfs_coffee_arch_read(void *buf, unsigned int size, cfs_offset_t offset)
{
  const uint8_t *src;
  uint8_t *dst;
  
  taskENTER_CRITICAL();
  for(src = (const void *)(COFFEE_START + offset), dst = buf; size; size--) 
  {
    *dst++ = ~*src++;
  }
  taskEXIT_CRITICAL();
}
/*---------------------------------------------------------------------------*/
int
cfs_coffee_arch_delete(unsigned int size, cfs_offset_t offset)
{
  pstorage_handle_t pstorage_cfs;
  pstorage_cfs.module_id = 0;
  pstorage_cfs.block_id = offset;

  return cmd_queue_enqueue(PSTORAGE_CLEAR_OP_CODE, &pstorage_cfs, NULL, size, 0);
}
#endif /* COFFEE_CONF_CUSTOM_PORT */

/** @} */
