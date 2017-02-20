/** @} */

/**
 * @brief       : 
 *
 * @file        : boards.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#include "boards.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_uart.h" 

static void bsp_leds_init(void)
{
    nrf_gpio_cfg_output(WORK_LED_RED);
    nrf_gpio_cfg_output(WORK_LED_BLUE);
    LEDS_OFF(WORK_LED_RED_MASK);
    LEDS_OFF(WORK_LED_BLUE_MASK);
}

void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{

}

/**
 * @breaf 初始化UART
 */
void bsp_uart_init(void)
{
    static uint8_t rx_temp = 0;
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = NRF_UART_BAUDRATE_115200;
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.pselrxd = RX_PIN_NUMBER;
    config.pseltxd = TX_PIN_NUMBER;

    uint32_t err_code = nrf_drv_uart_init(&config, uart_event_handler);
    
    nrf_drv_uart_rx_enable(); 
    
    nrf_drv_uart_rx(&rx_temp, 1);
}

/**
 * @breaf I2C初始化
 */
nrf_drv_twi_t m_i2c_adxl345 = NRF_DRV_TWI_INSTANCE(0);
static void bsp_twi_init(void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t i2c_adxl345_config = {
       .scl                = I2C_SCL_PIN,
       .sda                = I2C_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    nrf_drv_twi_uninit(&m_i2c_adxl345);
    err_code = nrf_drv_twi_init(&m_i2c_adxl345, &i2c_adxl345_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_i2c_adxl345);
}

/**
 * @breaf 初始化未使用IO
 */
uint8_t pin_num[] = {11, 13, 15, 17, 19, 22, 23, 24, 25};
static void bsp_unuse_pin_init(void)
{
    for(uint8_t i = 0; i < sizeof(pin_num)/sizeof(pin_num[0]); i++)
    {
        nrf_gpio_cfg_output(pin_num[i]);
        NRF_GPIO->OUTCLR = 1 << pin_num[i];
    }
}

/**
 * @breaf 硬件初始化
 */
void bsp_init(void)
{
    bsp_leds_init();            /**< 初始化LED */
    bsp_unuse_pin_init();       /**< 未使用IO */
    bsp_twi_init();             /**< 初始化I2C */
    gu620_io_config();          /**< 初始化GU620 GPIO */
}