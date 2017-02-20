/**
 * @brief       : 
 *
 * @file        : frame.c
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


#include "app.h"

#define SIMULATION      (0u)
time_t escort_timep;

bool_t escort_bind_operate(coap_pdu* send_buf)
{

	time(&escort_timep);

    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 1);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
#if SIMULATION 
	cJSON_AddStringToObject(root, "ext1", "PTYFBS20160905222");
#else
	cJSON_AddStringToObject(root, "ext1", (char const *)escort_basis_info.vehichle_identify_number); 
#endif    
	out = cJSON_PrintUnformatted(root);
	
        /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);
	
	return TRUE;
}

bool_t escort_unbind_operate(coap_pdu* send_buf)
{
	return TRUE;
}

bool_t escort_movement_operate(coap_pdu* send_buf)
{
	time(&escort_timep);

    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 3);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);
	
	return TRUE;
}

bool_t escort_stationary_operate(coap_pdu* send_buf)
{

	time(&escort_timep);

    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 4);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 

	cJSON_Delete(root);
	free(out);
	
	return TRUE;
	
}

bool_t escort_location_operate(coap_pdu* send_buf)
{        
	time(&escort_timep);

    /** json **/
	char *out;
	double gps_data[6];
	cJSON *root, *gps_array;     
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 5);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	
#if SIMULATION
	gps_data[0] = 120.3735;
	gps_data[1] = 31.495235;
	gps_data[2] = 3.1;
	gps_data[3] = 0.5;
	gps_data[4] = 25;  
    gps_data[5] = 0;
	gps_array = cJSON_CreateDoubleArray(gps_data, 5);
#else
	gps_data[0] = gps_info.lon;
	gps_data[1] = gps_info.lat;
	gps_data[2] = gps_info.alt;
	gps_data[3] = gps_info.speed;
	gps_data[4] = gps_info.spend;
	gps_data[5] = 0;
	if(gps_data[0] == 0)
		gps_array = cJSON_CreateNull();
	else
		gps_array = cJSON_CreateDoubleArray(gps_data, 5);
#endif    
	
	cJSON_AddItemToObject(root, "gps", gps_array);
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);
	
	return TRUE;
}

bool_t escort_remove_operate(coap_pdu* send_buf)
{     
	time(&escort_timep);
	
    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 6);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);

	return TRUE;
}

bool_t escort_lowpower_operate(coap_pdu* send_buf)
{       
	time(&escort_timep);
	
    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 7);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	cJSON_AddNumberToObject(root, "batt", bat_level.volts);
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);

	return TRUE;
}

bool_t escort_heartbeat_operate(coap_pdu* send_buf)
{    
	time(&escort_timep);
	
    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 8);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	cJSON_AddNumberToObject(root, "batt", bat_level.volts);
	out = cJSON_PrintUnformatted(root);  
	
    /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);

	return TRUE;
}

bool_t escort_onway_operate(coap_pdu* send_buf)
{

	time(&escort_timep);

    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 9);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	out = cJSON_PrintUnformatted(root);
	
        /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);
	
	return TRUE;
}

bool_t escort_onstation_operate(coap_pdu* send_buf)
{

	time(&escort_timep);

    /** json **/
	cJSON *root; 
	char *out;
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "msgcode", 10);
	cJSON_AddStringToObject(root, "version", SOFT_VERSION);    
	cJSON_AddNumberToObject(root, "ts", escort_timep); 
	out = cJSON_PrintUnformatted(root);
	
        /** coap **/
	uint8_t content_format = CF_APPLICATION_JSON;
	uint16_t message_id_counter = rand(); 
	coap_init_pdu(send_buf);             
	coap_set_version(send_buf, COAP_V1); 
	coap_set_type(send_buf, CT_CON);     
	coap_set_code(send_buf, CC_POST);
	coap_set_mid(send_buf, message_id_counter++);
	coap_add_option(send_buf, CON_URI_PATH, "devices", strlen("devices"));
	coap_add_option(send_buf, CON_URI_PATH, (uint8_t*)devID_payload, strlen(devID_payload));
	coap_add_option(send_buf, CON_URI_PATH, "shadow", strlen("shadow"));
	coap_add_option(send_buf, CON_CONTENT_FORMAT, &content_format, 1);
	coap_set_payload(send_buf, (uint8_t *)out, strlen((char const*)out)); 
	
	cJSON_Delete(root);
	free(out);
	
	return TRUE;
}

/* end of file*/