/**
* @brief       : 
* @file        : device_info.c
* @author      : xukai
* @version     : v0.0.1
* @date        : 2016/10/17
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/8/31    v0.0.1     wangyaoting    first version
*/
#include "device_info.h"
#include "nrf_log.h"
#include "app.h"

static char device_id[32] = {0x00, };
static char* devID_file = "dev_ID";

/**
 * @breaf 通过RTT View打印设备基本信息
 */
 void show_device_info(void)
 {
    uint8_t mac_addr[7];
    char mac_str[32] = {0x00, };
    // char id_str[32] = {0x00, };
    memset(mac_str, 0x00, 32);
    // memset(id_str, 0x00, 32);
    sd_ble_gap_address_get((ble_gap_addr_t*)mac_addr);
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[6], 
            mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1]);
    
    // 显示设备版本编号
    NRF_LOG_PRINTF("Device Version: %s\n", SOFT_VERSION);
    NRF_LOG_PRINTF("Device Hardware: %s\n", HARD_VERSION);
    // 显示设备蓝牙地址
    NRF_LOG_PRINTF("BLE Mac Address: %s\n", mac_str);
    // 显示设备编号
    NRF_LOG_PRINTF("Device ID: %s\n", get_device_id());
 }

const char* get_device_id(void)
{
    // 临时方法，使用最后一字节蓝牙地址，替换设备编号中的最后3个数字
    uint8_t mac_addr[7];
    int len = 0;
    sd_ble_gap_address_get((ble_gap_addr_t*)mac_addr);
    
    len = sprintf(device_id, DEVID_BASE"%03u", mac_addr[1]);
    device_id[len] = '\0';

    return device_id;
}

/**
* @brief 设备编号写入flash.
 */
int devID_to_flash(char* devID_buf, uint8_t len)
{
	int fd_write;
	
	cfs_delete(devID_file);
	fd_write = cfs_open(devID_file, CFS_WRITE | CFS_APPEND);
	if(fd_write != -1)
    {
        cfs_write(fd_write, devID_buf, len);
        cfs_close(fd_write);
        NRF_LOG_PRINTF("devID write to flash success!\n");
		return 0;
    }
    else 
    {
        NRF_LOG_PRINTF("devID write to flash failed!.\n");
		return -1;
    }
}

/**
* @brief 从flash读取设备编号.
 */
int devID_from_flash(char* devID_buf)
{
	int fd_read;
	static char read_buf[DEVID_LEN] = {0x00, };
	static int read_buf_num = -1; 
	
	fd_read = cfs_open(devID_file, CFS_READ);
	if(fd_read != -1) 
    {
        read_buf_num = cfs_read(fd_read, read_buf, sizeof(read_buf));
        cfs_close(fd_read);
		if(read_buf_num >= DEVID_LEN)
		{
			memcpy(devID_buf, read_buf, DEVID_LEN);
			NRF_LOG_PRINTF("devID read from flash success!\n");
			return 0;
		}
		else
		{
			NRF_LOG_PRINTF("There is no useful device ID in the file!\n"); 
			return 1;
		}
    }
    else 
    {
        NRF_LOG_PRINTF("devID read from flash failed!\n");
		return -1;
    }
}

