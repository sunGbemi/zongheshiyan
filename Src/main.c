/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
#include "esp8266.h"
#include "oled.h"
#include "bmp.h"
#include "24l01.h"
#include "DHT11.h"
#include "string.h"

extern uint8_t RX_buff;
uint8_t nrf24l01_buff[36];
uint16_t LED_Show=0;
char Voltage_Show[50];
char Tempture_Show[30];
char TShow[50];
char HShow[50];
char Show[50];

uint8_t temperature;         
uint8_t humidity; 
uint8_t temp;         
uint8_t humi; 
uint8_t rx_buf[5];

char T_Show[20];
char RH_Show[20];

uint8_t SHT3X_Modecommand_Buffer[2]={0x20,0x32};   //periodic mode commands 为什么是 0x20,0x32 见本帖附件page11           
uint8_t SHT3X_Fetchcommand_Bbuffer[2]={0xE0,0x00}; //Fetch data 为什么是 0xE0,0x00 见本帖附件page11
uint8_t SHT3X_Data_Buffer[6];                           //byte0,1为温度 byte4,5为湿度 见本帖附件page11        
uint8_t Humidity_S1;                               //湿度 正数   如果想得到小数点后几位见后面的分析
int8_t  Temperature_S1;                            //温度 可能为负数。如果想得到小数点后几位见后面的分析
float TemValue;
float RH_Value;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &RX_buff, 1);
  LCD_Init();
//  OLED_Init();
//  OLED_ColorTurn(0);//0正常显示，1 反色显示
//  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
//  OLED_Refresh();
//  OLED_ShowPicture(0,0,128,8,BMP1);
  HAL_Delay(1000);
//  esp8266_Init();
  NRF24L01_Init();
	
//	while(DHT11_Init())
//	{
//		HAL_Delay(1000);
//	}
  while(NRF24L01_Check()==1)
  {
//      LCD_ShowString(60,40,200,200,16,"NRF24L01_ERROR");
  }
  
//  NRF24L01_TX_Mode();
	NRF24L01_RX_Mode();
  
//  while(esp8266_check_cmd("CONNECT")==0)
//  {
//      LCD_ShowString(60,60,200,200,16,"192.168.123.101");
//      LCD_ShowString(60,80,200,200,16,"8086");
//  }
	
//	HAL_I2C_Master_Transmit(&hi2c2,0x44<<1,SHT3X_Modecommand_Buffer,2,0x10);  //第一步，发送periodic mode commands，传感器周期性的进行温湿度转换
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		DHT11_Read_Data(&temperature,&humidity,&temp,&humi);
//		rx_buf[0]=temperature;
//		rx_buf[1]=humidity;
//		sprintf(TShow,"tmp:%d",rx_buf[0]);
//		sprintf(HShow,"hum:%d",rx_buf[1]);
//		
		
		/*  发送部分的数据处理 */
//		HAL_I2C_Master_Transmit(&hi2c2,0x44<<1,SHT3X_Fetchcommand_Bbuffer,2,0x10); //第二步，随时读取传感器的数据       
//    HAL_I2C_Master_Receive(&hi2c2,(0x44<<1)+1,SHT3X_Data_Buffer,6,0x10); 
//		
//		Temperature_S1=(((SHT3X_Data_Buffer[0]<<8)+SHT3X_Data_Buffer[1])*175)/65535-45; //得到摄氏度温度
//		Humidity_S1=(((SHT3X_Data_Buffer[3]<<8)+SHT3X_Data_Buffer[4])*100)/65535;  //可以得到相对湿度
//		
//		TemValue=(float)(((SHT3X_Data_Buffer[0]<<8)+SHT3X_Data_Buffer[1])*175)/65535-45; //得到摄氏度温度
//		RH_Value=(float)(((SHT3X_Data_Buffer[3]<<8)+SHT3X_Data_Buffer[4])*100)/65535;  //可以得到相对湿度
//		
//		sprintf(Show,"temp:%.2f  RH:%.2f\r\n",TemValue,RH_Value);
//		
//		HAL_UART_Transmit(&huart1,(uint8_t*)Show,strlen(Show),0xffff);
		/*  发送部分的数据处理 */
		
//    if(NRF24L01_RxPacket(nrf24l01_buff)==0)
//    {
//        LCD_ShowString(60,100,200,200,16,nrf24l01_buff);
//    }

		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==0)
		{
//			uint8_t test2=NRF24L01_TxPacket((uint8_t *)Show);
		  if(NRF24L01_RxPacket(nrf24l01_buff)==0)
			{
					uint8_t dataone[10];
					uint8_t datatwo[10];
					uint8_t datathree[10];
					for(uint8_t i = 0;i<10;i++){
						dataone[i]=nrf24l01_buff[i];
						datatwo[i]=nrf24l01_buff[i+12];
						datathree[i]=nrf24l01_buff[i+24];
					}
					
					LCD_ShowString(40,100,200,200,16,dataone);
					LCD_ShowString(40,120,200,200,16,datatwo);
					LCD_ShowString(40,140,200,200,16,datathree);
			}
		}
		else
		{

		}
		
//    esp8266_send_cmd("AT+CIPSEND=0,4\r\n","OK",100);
//    esp8266_send_data("TEST",0,0);

    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_4)
	{
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	
	static uint16_t ADC_Value;
	static float Tempture;
	uint8_t i;
	if(hadc==&hadc1)
	{
		ADC_Value=HAL_ADC_GetValue(&hadc1);
		Tempture=(float)ADC_Value/4095.0f*3.3f*100/2.9f;
//		if(Tempture>100)
//			Tempture=0;
		for(i=0;i<30;i++)
		{
			Tempture_Show[i]=0;
		}
		LED_Show=1;
		sprintf((char*)Tempture_Show,"Tempture:%.4f",Tempture);
		
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
