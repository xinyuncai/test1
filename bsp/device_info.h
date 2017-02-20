/**
* @brief       : 
* @file        : device_info.h
* @author      : xukai
* @version     : v0.0.1
* @date        : 2016/10/17
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/10/17    v0.0.1     xukai    first version
*/

#ifndef __DEVICE_INFO_H__
#define __DEVICE_INFO_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "boards.h"

#define DEVID_BASE      "4111201606171"
#define DEVID_LEN       16

void show_device_info(void);
const char* get_device_id(void);

/**
 * \brief Reads the device ID from flash.
 * \param devID_buf The read buff.
 * \return 0 on success, 1 no device ID, -1 on failure.
 */
int devID_from_flash(char* devID_buf);

/**
 * \brief writes the device ID to flash.
 * \param devID_buf The write buff.
 * \param len long of write buff.
 * \return 0 on success, -1 on failure.
 */
int devID_to_flash(char* devID_buf, uint8_t len);

#endif