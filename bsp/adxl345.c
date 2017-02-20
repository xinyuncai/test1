/**
* @brief       : 
*
* @file        : adxl345.c
* @author      : wangyaoting
* @version     : v0.0.1
* @date        : 2016/8/31
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/8/31    v0.0.1     wangyaoting    first version
*/
#include "adxl345.h"
#include "nrf_log.h"
#include "nrf_drv_twi.h"
#include "app.h"

#define ADXL345_DATA_OUT_REG_NUM    6 //X Y Z三轴6个寄存器

uint16_t adxl345_devid_read(void);
void adxl345_sensor_init(void);
bool adxl345_sensor_is_init(void);
void adxl345_sensor_uninit(void);
bool adxl345_read_register(uint8_t reg_add , uint8_t *pvalue);
bool adxl345_write_register(uint8_t reg_add , uint8_t reg_value);
bool adxl345_read_buff(uint8_t reg_add , uint8_t *pregbuf , uint8_t  len);
adxl345_value_t adxl345_read_value(void);
uint8_t adxl345_read_int(void);
double adxl345_move_detect(adxl345_value_t * acc_buf);
void adxl345_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

// 外部变量 board.c中定义
extern const nrf_drv_twi_t m_i2c_adxl345;
static uint8_t init_cnt = 0;

/**
 * @breaf ADXL345接口初始化
 */
device_adxl345_t adxl345 =
{
    .init = adxl345_sensor_init,
    .is_init = adxl345_sensor_is_init,
    .devid_read = adxl345_devid_read,
    .value = adxl345_read_value,
    .read_int = adxl345_read_int,
	.uninit = adxl345_sensor_uninit,
};

/**
 * @breaf 读取adxl345 Device ID
 * @return adxl345地址
 */
uint16_t adxl345_devid_read(void)
{
    uint8_t reg = ADXL345_DEVID;
    adxl345_read_register(ADXL345_DEVID, &reg);
    return reg;
}

/**
 * @breaf 读取单个寄存器值
 *      [Start][Device Addr][W]{A}[Reg Addr]{A}
 *      [Start][Device Addr][R]{Reg Value}[NA][Stop]
 * @param[in] reg_add 寄存器地址.
 * @param[in] pvalue 寄存器值.
 * @return 读取是否成功.
 */
bool adxl345_read_register(uint8_t reg_addr, uint8_t *pvalue)
{
    nrf_drv_twi_tx(&m_i2c_adxl345, ADXL345_ADDRESS, &reg_addr, 1, true);
    nrf_drv_twi_rx(&m_i2c_adxl345, ADXL345_ADDRESS, pvalue, 1);

    return true;
}
/**
 * @breaf 向寄存器中写入单个字节数据
 *      [Start][Device Addr][W]{A}[Reg Addr]{A}[Reg Value]{A}[Stop]
 * @param[in] reg_addr 寄存器地址.
 * @param[in] reg_value 写入值.
 * @return 写入是否成功.
 */
bool adxl345_write_register( uint8_t reg_addr , uint8_t reg_value)
{
    uint8_t buf[2] = {reg_addr, reg_value};
    nrf_drv_twi_tx(&m_i2c_adxl345, ADXL345_ADDRESS, (uint8_t const*)&buf, 2, false);
    return true ;
}
/**
 * @breaf 从传感器寄存器连续读取多个字节
 * @param[in] reg_add 寄存器地址.
 * @param[in] pregbuf 读取值存放地址.
 * @param[in] len 读取长度.
 * @return 读取是否成功.
 */
bool adxl345_read_buff(uint8_t reg_addr, uint8_t *pregbuf , uint8_t len)
{
    nrf_drv_twi_tx(&m_i2c_adxl345, ADXL345_ADDRESS, (uint8_t const*)&reg_addr, 1, true);
    nrf_drv_twi_rx(&m_i2c_adxl345, ADXL345_ADDRESS, pregbuf, len);
    return true;
}

/**
 * @breaf 获得三轴加速度传感器各轴检测结果
 * @param[in] pacc_x x方向加速度值
 * @param[in] pacc_y y方向加速度值
 * @param[in] pacc_z z方向加速度值
 * @return 读取是否成功.
 */
bool adxl345_get_xyz( int16_t *pacc_x , int16_t *pacc_y , int16_t *pacc_z)
{
    adxl345_write_register(ADXL345_POWER_CTL, 0x08);

    uint8_t reg_addr = ADXL345_DATAX0;
    static uint8_t acc_reg[6];
    static int16_t acc_result[6];

	//一次读取六个寄存器值
	adxl345_read_buff(reg_addr, acc_reg, 6);
	
	//逐个读取寄存器值
//    for(int i = 0; i < 6; i++)
//    {
//        nrf_drv_twi_tx(&m_i2c_adxl345, ADXL345_ADDRESS, (uint8_t const*)&reg_addr, 1, true);
//        nrf_drv_twi_rx(&m_i2c_adxl345, ADXL345_ADDRESS, (uint8_t*)&acc_reg[i], 1);
//        reg_addr++;
//    }
		
    acc_result[1] = acc_reg[1] << 8;
    *pacc_x = acc_result[1] + acc_reg[0];
    acc_result[3] = acc_reg[3] << 8;
    *pacc_y = acc_result[3] + acc_reg[2];
    acc_result[5] = acc_reg[5] << 8;
    *pacc_z = acc_result[5] + acc_reg[4];
    
    return true;
}

/**
 * @breaf adxl345寄存器初始配置操作
 */
static void adxl345_settings(void)
{  
    uint8_t device_id = 0;
    adxl345_read_register(ADXL345_DEVID, &device_id);
    if(ADXL345_ID != device_id)
    {
        NRF_LOG_ERROR("adxl345 init failed");
//        while(true);
    }
    // ADXL345_REG_POWER_CTL[3]=0设置成待机模式,即清除测试位
    adxl345_write_register(ADXL345_POWER_CTL, 0x00); 
    //BW_RATE[4]=1；即设置LOW_POWER位低功耗
    //BW_RATE[3][2][1][0]=0x07，即设置输出速率12.5HZ，Idd=34uA
    //普通，100hz
    adxl345_write_register(ADXL345_BW_RATE, 0x07);                                                     
    //THRESH_TAP: 比例因子为62.5mg/lsb  建议大于3g 
    //2g=0X20,,,4g=0x40,,,8g=0x80,,,16g=0xff   3.5g=0x38
    adxl345_write_register(ADXL345_THRESH_TAP, 0x38); 
                                                     
    adxl345_write_register(ADXL345_OFSX, 0x1C);       // X轴偏移量
    adxl345_write_register(ADXL345_OFSY, 0x01);       // Y轴偏移量
    adxl345_write_register(ADXL345_OFSZ, 0x00);       // Z轴偏移量
    //DUR:比例因子为625us/LSB，建议大于10ms
    //6.25ms=0x0A //12.5ms=0x14。
    adxl345_write_register(ADXL345_DUR, 0x14); 
    //Latent:比例因子为1.25ms/LSB，建议大于20ms
    //2.5ms=0x02，，20ms=0x10，，，25ms=0x14
    adxl345_write_register(ADXL345_LATENT, 0x14);    
    //window:比例因子为1.25ms/LSB，建议大于80ms 
    //10ms=0x08，，80ms=0x40
    adxl345_write_register(ADXL345_WINDOW, 0x41);
    //THRESH_ACT:比例因子为62.5mg/LSB，
    //2g=0X20,,,4g=0x40,,,8g=0x80,,,16g=0xff,,,//1.5g=0x18
    adxl345_write_register(ADXL345_THRESH_ACT, 0x18);
    //THRESH_INACT:比例因子为62.5mg/LSB
    //1g=0x10  //2g=0X20,,,4g=0x40,,,8g=0x80,,,16g=0xff
    adxl345_write_register(ADXL345_THRESH_INACT, 0x10);

    //TIME_INACT:比例因子为1sec/LSB      //1s=0x01
    adxl345_write_register(ADXL345_TIME_INACT, 0x05);
    //设置为直流耦合：当前加速度值直接与门限值比较，以确定是否运动或静止  
    //x,y,z参与检测活动或静止
    adxl345_write_register(ADXL345_ACT_INACT_CTL, 0x77);
    //用于自由落地检测，比例因子为62.5mg/LSB
    //建议设置成300mg~600mg（0x05~0x09）
    adxl345_write_register(ADXL345_THRESH_FF, 0x06);       
    //所有轴的值必须小于此设置值，才会触发中断;比例因子5ms/LSB   
    //建议设成100ms到350ms(0x14~~0x46),200ms=0x28
    adxl345_write_register(ADXL345_TIME_FF, 0x28);         
    //TAP_AXES:单击/双击轴控制寄存器； 
    //1）不抑制双击  2）使能x.y,z进行敲击检查
    adxl345_write_register(ADXL345_TAP_AXES, 0x07);  
    // 中断使能   
    //1）DATA_READY[7]   2)SINGLE_TAP[6]  3)DOUBLE_TAP[5]  4)Activity[4]
    //5)inactivity[3]    6)FREE_FALL[2]   7)watermark[1]   8)overrun[0]
    adxl345_write_register(ADXL345_INT_ENABLE, 0xFF); 
                                                    
    //INT_MAC中断映射：任意位设为0发送到INT1位，设为1发送到INT2位
    //1）DATA_READY[7]   2)SINGLE_TAP[6]  3)DOUBLE_TAP[5]  4)Activity[4]
    //5)inactivity[3]    6)FREE_FALL[2]   7)watermark[1]   8)overrun[0] 
    adxl345_write_register(ADXL345_INT_MAP, 0xBD);   
    
    //1）SELF_TEST[7];2)SPI[6]; 3)INT_INVERT[5]：设置为0中断高电平有效，
    // 数据输出格式  高电平触发
    adxl345_write_register(ADXL345_DATA_FORMAT, 0x0B);
    //adxl345_write_register(ADXL345_DATA_FORMAT,0x2B);// 数据输出格式  低电平触发         
                                                    //反之设为1低电平有效    rang[1][0]
    //设置 FIFO模式
    adxl345_write_register(ADXL345_FIFO_CTL, 0x82);
    // 进入测量模式
    //1)链接位[5]    2)AUTO_SLEEP[4]   3)测量位[3]  4)休眠位[2]  5)唤醒位[1][0]
    adxl345_write_register(ADXL345_POWER_CTL, 0x28);
    uint8_t int_source; 
    adxl345_read_register(ADXL345_INT_SOURCE, &int_source);
}

/**
 * @breaf adxl345引脚配置
 */
static void adxl345_int_cfg(void)
{
    ret_code_t err_code;

    if(nrf_drv_gpiote_is_init() != true)
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(ADXL_SENSOR_INT, &in_config, adxl345_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(ADXL_SENSOR_INT, true);
}

/**
 * @breaf adxl345电源打开
 */
void adxl345_power_open(void)
{
    nrf_gpio_cfg_output(ADXL_SENSOR_CE); 
    nrf_gpio_pin_set(ADXL_SENSOR_CE);
}

/**
 * @breaf adxl345初始化
 */
void adxl345_sensor_init(void)
{
	if(init_cnt == 0)
	{
	  	init_cnt = 1;
		
		adxl345_power_open();       // 打开电源
		
		os_delay(1000);

		adxl345_settings();         // ADXL345设置

		adxl345_int_cfg();          // 中断配置
	}
}

/**
 * @breaf adxl345初始化
 */
bool adxl345_sensor_is_init(void)
{
  	if(init_cnt == 1)
	{
		return true;
	}
	return false;
}
/**
 * @breaf adxl345反初始化
 */
void adxl345_sensor_uninit(void)
{
  	init_cnt = 0;
	nrf_drv_gpiote_in_uninit(ADXL_SENSOR_INT);
	nrf_drv_gpiote_uninit();
	nrf_drv_twi_uninit(&m_i2c_adxl345);
	nrf_gpio_pin_clear(ADXL_SENSOR_CE);
}

/**
 * @breaf 更新adxl345.value的加速度值
 * @return 读取的加速度值
 */
adxl345_value_t adxl345_read_value(void)
{
    adxl345_value_t acc_value;
    adxl345_get_xyz(&acc_value.x, &acc_value.y, &acc_value.z);
    return acc_value;
}

/**
 * @breaf 读取ADXL345_INT_SOURCE中断值
 * @return ADXL345_INT_SOURCE值
 */
uint8_t adxl345_read_int(void)
{
    uint8_t retvalue;
    adxl345_read_register(ADXL345_INT_SOURCE, &retvalue);
    return retvalue;
}

/**
 * @breaf adxl345中断函数
 */
void adxl345_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static uint8_t retvalue;
    adxl345_read_register(ADXL345_INT_SOURCE, &retvalue);

    BaseType_t yield_req = pdFALSE;
	if((retvalue & 0x40) != 0x00)
	{
	  	yield_req = pdFALSE;
		xSemaphoreGiveFromISR(sem_escort_tap, &yield_req);
		portYIELD_FROM_ISR(yield_req);
	}
	
	if((retvalue & 0x02) != 0x00)
	{
		escort_dec_cmd = DEC_MOVE;
		yield_req = pdFALSE;
		xSemaphoreGiveFromISR(sem_escort_dec, &yield_req);
		portYIELD_FROM_ISR(yield_req);
	}	
}