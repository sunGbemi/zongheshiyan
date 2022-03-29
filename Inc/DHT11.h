#ifndef _DHT11_H_
#define _DHT11_H_
#include "main.h"
#define DHT11_DATA_OUT_Pin DHT11_Pin
#define DHT11_DATA_OUT_GPIO_Port GPIOB

void Delay_us(uint32_t i);

void DHT11_Rst(void);
uint8_t DHT11_Check(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi,uint8_t *tem,uint8_t *hum);
uint8_t DHT11_Init(void);
void DHT11_IO_IN(void);
void DHT11_IO_OUT(void);
#endif
