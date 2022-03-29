#include "esp8266.h"
#include "usart.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>

extern uint8_t RX_cnt;
extern char RX_DATA_buff[256];
#define LEN_max  256
//检测应答信号
uint8_t* esp8266_check_cmd(uint8_t *str)
{
	char *strx=0;
	strx=strstr((const char*)RX_DATA_buff,(const char*)str);
	return (uint8_t*)strx;
}

//发送命令
uint8_t esp8266_send_cmd(uint8_t *cmd,uint8_t *ack,uint16_t waittime)
{
    int8_t res=0;
    RX_cnt=0;
    HAL_UART_Transmit(&huart2, cmd, strlen(cmd),0xFFFF);
    if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			HAL_Delay(10);
            if(RX_cnt!=0)
            {
                if(esp8266_check_cmd(ack))
                {
                    printf("ack:%s\r\n",(uint8_t*)ack);
                    break;//得到有效数据 
                }
            }
		}
		if(waittime==0)res=1; 
	}
    return res;
}

//发送数据
uint8_t esp8266_send_data(uint8_t *cmd,uint8_t *ack,uint16_t waittime)
{
    int8_t res=0;
    RX_cnt=0;
    HAL_UART_Transmit(&huart2, cmd, sizeof(cmd),0xFFFF);
    if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			HAL_Delay(10);
            if(RX_cnt!=0)
            {
                if(esp8266_check_cmd(ack))
                {
                    printf("ack:%s\r\n",(uint8_t*)ack);
                    break;//得到有效数据 
                }
            }
		}
		if(waittime==0)res=1; 
	}
    return res;
}

//esp8266初始化为AP模式
void esp8266_Init(void)
{
    uint8_t esp8266_flag;
    while(esp8266_send_cmd("AT\r\n","OK",100))
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro");
    }
  //  esp8266_send_cmd("ATE1\r\n","OK",10);

    LCD_ShowString(60,60,200,200,16,"              ");
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CWMODE=2\r\n","OK",100);   //设置为AP模式
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro1");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+RST\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro2");
    }
    HAL_Delay(1000);
    HAL_Delay(1000);
    HAL_Delay(1000);
    HAL_Delay(1000);
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CWSAP=\"AABBCC\",\"12345678\",1,4\r\n","OK",100);  //配置WIFI以及密码
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro3");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CIPAP=\"192.168.123.101\"\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro4");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CIPMUX=1\r\n","OK",100);   //设置单连接模式
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro5");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CIPSERVER=1,8086\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro6");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    RX_cnt=0;
}


void esp8266_Init_sta(void)
{
    uint8_t esp8266_flag;
    while(esp8266_send_cmd("AT\r\n","OK",100))
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro");
    }
    esp8266_send_cmd("ATE1\r\n","OK",10);

    LCD_ShowString(60,60,200,200,16,"              ");
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CWMODE=1\r\n","OK",100);   //设置为STA模式
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro1");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+RST\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro2");
    }
    HAL_Delay(1000);
    HAL_Delay(1000);
    HAL_Delay(1000);
    HAL_Delay(1000);
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CWJAP=\"2-314\",\"huangkjdx0\"\r\n","OK",1000);  //连接WIFI
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro3");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    esp8266_flag=esp8266_send_cmd("AT+CIPSTART=\"TCP\",\"192.168.1.103\",8086\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro4");
    }
    esp8266_flag=esp8266_send_cmd("AT+CIPMODE=1\r\n","OK",100);
    while(esp8266_flag)
    {
        LCD_ShowString(60,60,200,200,16,"esp8266_erro5");
    }
    memset(RX_DATA_buff,0x00,strlen(RX_DATA_buff));
    RX_cnt=0;
}
