#ifndef __BLEDEFINEDUUIDS_H__
#define __BLEDEFINEDUUIDS_H__

/**
 * @brief 车押设备蓝牙应用服务和UUID定义
 */
#define BLE_EBS                 {{0x0E, 0x94, 0x1B, 0x13, 0xFC, 0xC3, \
                                  0x5A, 0x9F, 0xA4, 0x4C, 0x5D, 0xFA, \
                                  0x00, 0x00, 0x71, 0x48}}
                                /**< WSN蓝牙服务基数 */
#define BLE_EBS_BASE            0x0010  /**< 车押设备蓝牙应用服务 基数 */
#define BLE_EBS_CMDUPDATE       0x0011  /**< 控制命令特性 */
#define BLE_EBS_VIDNUM          0x0020  /**< 车架号特性 */
#define BLE_EBS_USERINFO        0xC877  /**< 暂未使用 建议去除 */
#define BLE_EBS_WHINFO          0x85A3  /**< 暂未使用 建议去除 */
#define BLE_EBS_ESCORTSTATE     0x2E73  /**< 暂未使用 建议去除 */


/**
 * @brief 车押设备蓝牙调试服务和UUID定义
 */
#define BLE_DBS                 {{0x0E, 0x94, 0x1B, 0x13, 0xFC, 0xC3, \
                                  0x5A, 0x9F, 0xA4, 0x4C, 0x5D, 0xFA, \
                                  0x00, 0x00, 0x71, 0x48}}
                                /**< WSN蓝牙服务基数 */
#define BLE_DBS_BASE            0x00A0  /**< 车押设备蓝牙调试服务 基数 */
#define BLE_DBS_FLOG            0x00A1  /**< 强制调试特性 */
#define BLE_DBS_ADJUSTTIME      0x00A2  /**< 时间校准特性 */
#define BLE_DBS_IPADDR			0x00A3  /**< 服务器IP地址设置 */
#define BLE_DBS_IPPORT			0x00A4  /**< 服务器端口号设置 */

#endif