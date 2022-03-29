#ifndef __ESP8266_H
#define __ESP8266_H		
#include "stm32f1xx_hal.h"

uint8_t* esp8266_check_cmd(uint8_t *str);       //检测返回数据的
uint8_t esp8266_send_cmd(uint8_t *cmd,uint8_t *ack,uint16_t waittime);
uint8_t esp8266_send_data(uint8_t *cmd,uint8_t *ack,uint16_t waittime);
void esp8266_Init(void);
void esp8266_Init_sta(void);

#endif
