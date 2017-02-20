/**
 * @brief       : 
 *
 * @file        : frame.h
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/9/23
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/23    v0.0.1      xujing    first version
 * status: getcounter is ok.
 *         setcounter needs to prove.
 *         alarm function needs to prove.
 */

#ifndef __FRAME_H__
#define __FRAME_H__

#include "cJSON.h"
#include "coap.h"

extern bool_t escort_bind_operate(coap_pdu * send_buf);
extern bool_t escort_unbind_operate(coap_pdu * send_buf);
extern bool_t escort_movement_operate(coap_pdu * send_buf);
extern bool_t escort_stationary_operate(coap_pdu * send_buf);
extern bool_t escort_location_operate(coap_pdu * send_buf);
extern bool_t escort_remove_operate(coap_pdu * send_buf);
extern bool_t escort_lowpower_operate(coap_pdu * send_buf);
extern bool_t escort_heartbeat_operate(coap_pdu * send_buf);
extern bool_t escort_onway_operate(coap_pdu * send_buf);
extern bool_t escort_onstation_operate(coap_pdu * send_buf);


#endif
