/*
 * Copyright (c) 2013, ADVANSEE - http://www.advansee.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538
 * @{
 *
 * \defgroup cc2538-cfs-coffee-arch cc2538 Coffee port module
 *
 * Module for the cc2538 Coffee port
 * @{
 *
 * \file
 * Header file for the cc2538 Coffee port module
 */
#ifndef CFS_COFFEE_ARCH_H_
#define CFS_COFFEE_ARCH_H_

#include <stdint.h>
#include <nrf_nvmc.h>
#ifdef FREERTOS
#include <FreeRTOS.h>
#include <task.h>
#endif

#ifdef COFFEE_CONF_CUSTOM_PORT
#include COFFEE_CONF_CUSTOM_PORT
#else
   
#define FLASH_PAGE_SIZE 4096  
#define FLASH_WORD_SIZE 4
#define NRF52_DEV_FLASH_ADDR 0
#define FLASH_CCA_ADDR  0x80000   
   
/*---------------------------------------------------------------------------*/
/** \name Coffee port constants
 * @{
 */
/** Logical sector size */
#define COFFEE_SECTOR_SIZE     FLASH_PAGE_SIZE

/** Logical page size */
#define COFFEE_PAGE_SIZE       (COFFEE_SECTOR_SIZE / 8)

/** Start offset of the file system */
#define COFFEE_START           0x60000

/** Total size in bytes of the file system */
#define COFFEE_SIZE            (128 * 1024) // 0x20000

/** End offset of the file system */
#define COFFEE_END             (COFFEE_START + COFFEE_SIZE) // 0x80000

/** Maximal filename length */
#ifdef COFFEE_CONF_NAME_LENGTH
#define COFFEE_NAME_LENGTH     COFFEE_CONF_NAME_LENGTH
#else
#define COFFEE_NAME_LENGTH     10
#endif
/** Number of file cache entries */
#ifdef COFFEE_CONF_MAX_OPEN_FILES
#define COFFEE_MAX_OPEN_FILES  COFFEE_CONF_MAX_OPEN_FILES
#else
#define COFFEE_MAX_OPEN_FILES  5
#endif
/** Number of file descriptor entries */
#ifdef COFFEE_CONF_FD_SET_SIZE
#define COFFEE_FD_SET_SIZE     COFFEE_CONF_FD_SET_SIZE
#else
#define COFFEE_FD_SET_SIZE     5
#endif
/** Maximal amount of log table entries read in one batch */
#ifdef COFFEE_CONF_LOG_TABLE_LIMIT
#define COFFEE_LOG_TABLE_LIMIT COFFEE_CONF_LOG_TABLE_LIMIT
#else
#define COFFEE_LOG_TABLE_LIMIT 16
#endif
/** Default reserved file size */
#ifdef COFFEE_CONF_DYN_SIZE
#define COFFEE_DYN_SIZE        COFFEE_CONF_DYN_SIZE
#else
#define COFFEE_DYN_SIZE        (COFFEE_SECTOR_SIZE - 50)
#endif
/** Default micro-log size */
#ifdef COFFEE_CONF_LOG_SIZE
#define COFFEE_LOG_SIZE        COFFEE_CONF_LOG_SIZE
#else
#define COFFEE_LOG_SIZE        (4 * COFFEE_PAGE_SIZE)
#endif
/** Whether Coffee will use micro logs */
#ifdef COFFEE_CONF_MICRO_LOGS
#define COFFEE_MICRO_LOGS      COFFEE_CONF_MICRO_LOGS
#else
#define COFFEE_MICRO_LOGS      0
#endif
/** Whether files are expected to be appended to only */
#ifdef COFFEE_CONF_APPEND_ONLY
#define COFFEE_APPEND_ONLY     COFFEE_CONF_APPEND_ONLY
#else
#define COFFEE_APPEND_ONLY     0
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** \name Coffee port macros
 * @{
 */
/** Erase */
#define COFFEE_ERASE(sector) \
  cfs_coffee_arch_erase(sector)
/** Write */
#define COFFEE_WRITE(buf, size, offset) \
  cfs_coffee_arch_write((buf), (size), (offset))
/** Read */
#define COFFEE_READ(buf, size, offset) \
  cfs_coffee_arch_read((buf), (size), (offset))
/** @} */
/*---------------------------------------------------------------------------*/
/** \name Coffee port types
 * @{
 */
typedef int16_t coffee_page_t; /**< Page */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name Coffee port functions
 * @{
 */

/** \brief Erases a device sector
 * \param sector Sector to erase
 */
void cfs_coffee_arch_erase(uint16_t sector);

/** \brief Writes a buffer to the device
 * \param buf Pointer to the buffer
 * \param size Byte size of the buffer
 * \param offset Device offset to write to
 */
void cfs_coffee_arch_write(const void *buf, unsigned int size,
                           cfs_offset_t offset);

/** \brief Reads from the device to a buffer
 * \param buf Pointer to the buffer
 * \param size Byte size of the buffer
 * \param offset Device offset to read from
 */
void cfs_coffee_arch_read(void *buf, unsigned int size, cfs_offset_t offset);

void cfs_coffee_erase(uint16_t sector);

int cfs_coffee_arch_delete(unsigned int size, cfs_offset_t offset);
/** @} */

#endif /* COFFEE_CONF_CUSTOM_PORT */
#endif /* CFS_COFFEE_ARCH_H_ */

/**
 * @}
 * @}
 */
