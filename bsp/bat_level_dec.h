/**
* @brief       : 
*
* @file        : bat_level_dec.h
* @author      : wangyaoting
* @version     : v0.0.2
* @date        : 2016/10/17
*
* Change Logs  :
*
* Date        	Version      Author      		Notes
* 2016/9/22   	v0.0.1      wangyaoting    	first version
* 2016/10/17   	v0.0.2      wangyaoting    	second version
*/
#ifndef __BAT_LEVEL_DEC_H
#define __BAT_LEVEL_DEC_H

/**
* @breaf bat_level接口
*/
typedef struct bat_level_
{
    void (*init)(void);
    void (*updata)(void);
    double volts;
    void (*uninit)(void);
} bat_level_t;

extern bat_level_t bat_level;
#endif