#include "dht11.h"
#include "tim.h"


#define DHT11_DQ_IN HAL_GPIO_WritePin(GPIOB, DHT11_DATA_OUT_Pin, GPIO_PIN_SET)	  //����

//��ʼ��DHT11��ͬʱ����Ƿ�������DHT11��PA11��ʼ��
 	 
uint8_t DHT11_Init(void)
{	 
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, DHT11_DATA_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DHT11_DATA_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_DATA_OUT_GPIO_Port, &GPIO_InitStruct);	

	DHT11_Rst();  
	return DHT11_Check();
}      
//��λDHT11
void DHT11_Rst(void)	   
{                 
	DHT11_IO_OUT(); 	//SET OUTPUT
	HAL_GPIO_WritePin(GPIOB, DHT11_DATA_OUT_Pin, GPIO_PIN_RESET); 	//����
	HAL_Delay(20);    	//������ʱ����18ms
	HAL_GPIO_WritePin(GPIOB, DHT11_DATA_OUT_Pin, GPIO_PIN_SET); 	//DQ=1������ 
	Delay_us(30);     	//������ʱ����20~40us
}
 
//����Ӧ
//����1��������
//����0�����ɹ�
uint8_t DHT11_Check(void) 	   
{   
	uint8_t retry=0;
	DHT11_IO_IN();//SET INPUT	 
    while (HAL_GPIO_ReadPin(GPIOB, DHT11_Pin)&&retry<100)//DHT11����40~80us
	{
		retry++;
		Delay_us(1);
	};	 
	if(retry>=100)return 1;
	else retry=0;
    while (!HAL_GPIO_ReadPin(GPIOB, DHT11_Pin)&&retry<100)//DHT11�ٴ�����40~80us
	{
		retry++;
		Delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}
 
//��ȡһ��λBit
//����1��0
uint8_t DHT11_Read_Bit(void) 			 
{
 	uint8_t retry=0;
	while(HAL_GPIO_ReadPin(GPIOB, DHT11_Pin)&&retry<100)//�ȴ���͵�ƽ
	{
		retry++;
		Delay_us(1);
	}
	retry=0;
	while(!HAL_GPIO_ReadPin(GPIOB, DHT11_Pin)&&retry<100)//�ȴ���ߵ�ƽ
	{
		retry++;
		Delay_us(1);
	}
	Delay_us(40);//�ȴ�40us
	if(HAL_GPIO_ReadPin(GPIOB, DHT11_Pin))return 1;
	else return 0;		   
}
 
//��ȡһ���ֽ�
//���ض���������
uint8_t DHT11_Read_Byte(void)    
{        
	uint8_t i,dat;
	dat=0;
	for (i=0;i<8;i++) 
	{
		dat<<=1; 
		dat|=DHT11_Read_Bit();
	}						    
	return dat;
}
 
//DHT11��ȡһ������
//temp:�¶�(��Χ:0~50��)
//humi:ʪ��(��Χ:20%~90%)
//tem���¶�С��λ
//hum��ʪ��С��λ
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi,uint8_t *tem,uint8_t *hum)    
{        
 	uint8_t buf[5];
	uint8_t i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//��ȡ40λ�ֽ�
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];
			*hum=buf[1];
			*temp=buf[2];
			*tem=buf[3];
		}
	}
	else return 1;
	return 0;	    
}
 

//DHT11���ģʽ����
void DHT11_IO_OUT()	
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DHT11_DATA_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DHT11_DATA_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_DATA_OUT_GPIO_Port, &GPIO_InitStruct);
}
 
//DHT11����ģʽ����
void DHT11_IO_IN(void)	
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_DATA_OUT_GPIO_Port, &GPIO_InitStruct);
}
