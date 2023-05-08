/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	CMD_DELAY 	1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint8_t rx_data = 0;
uint8_t rx_buffer[1460] = {0};
uint16_t rx_index = 0;

uint8_t mqtt_receive = 0;
char mqtt_buffer[1460] = {0};
uint16_t mqtt_index = 0;
uint8_t connect 		= 0;
uint8_t signal = 0;

unsigned char mqttMessage_header[127];

int mqttMessageLength_header = 0;

unsigned char mqttMessage_body[127];

int mqttMessageLength_body = 0;

int32_t session = 0;

char *  clientId =  "basbousa-ny1023F4D";

const char Signal[]             = "AT+CSQ\r\n";         //check the signal quality.
const char Imei[]               = "AT+CGSN\r\n";
const char attach[]             = "AT+CGATT=1\r\n";
const char attachStatu[]        = "AT+CGATT?\r\n";
const char apn_ooredoo[]        = "AT+CIPCSGP=1,\"internet.ooredoo.tn\",\"\",\"\"\r\n";
const char apnStatu[]           = "AT+CSTT?\r\n";
const char gprscnxn[]           = "AT+CIICR\r\n";
const char localIp[]            = "AT+CIFSR\r\n";
const char localIp2[]           = "AT+SAPBR=2,1\r\n";
const char Connect[]            = "AT+CIPSTART=\"TCP\",\"broker.mqttdashboard.com\",\"1883\"\r\n";
const char data[]               = "AT+CIPSEND\r\n";
const char IpStatu[]            = "AT+CIPSTATUS\r\n";
const char mux[]                = "AT+CIPMUX=0\r\n";
const char IPR[]                ="AT+IPR=9600\r\n";
const char GSM[]                ="AT+CSCS=\"GSM\"\r\n";
const char COPS[]               ="AT+COPS?\r\n";
const char CREG[]               ="AT+CREG=1\r\n";
const char confSend[]           ="AT+CIPSPRT=1\r\n";
const char DataToServer[]       ="Hello Server ...\x1A\r\n";
const char SenD[]               ="AT+CIPSEND=16\r\n";
const char TCPClose[]           ="AT+CIPCLOSE\r\n";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */

int MQTT_Init(void);

void MQTT_Connect();

void printk(const char *fmt, ...);

void mqtt_connect_message(uint8_t *, char *);

void mqtt_publish_message(uint8_t *, char *, char *);

void mqtt_disconnect_message(uint8_t *);

void sendMQTTMessage(char* clientId, char* topic, char* message);
void clearRxBuffer(void);
void clearMqttBuffer(void);

int SIM900_SendCommand(const char *command, char *reply, uint16_t delay);

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TSC_Init();
  MX_USB_PCD_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();



  //HAL_UART_Receive_DMA (&huart3, rx_buffer, 300);


  /* USER CODE BEGIN 2 */

  	  //	AT+IPR=115200

  	//  MQTT_Init();

  	 HAL_Delay(20000);

	SIM900_SendCommand("AT\r\n", "OK\r\n", CMD_DELAY);

	SIM900_SendCommand("AT+CGATT=1\r\n", "OK\r\n", CMD_DELAY);

	SIM900_SendCommand("AT+CSTT=\"internet.ooredoo.tn\"", "OK\r\n", 2000);

	SIM900_SendCommand("AT+CIICR\r\n", "OK\r\n", CMD_DELAY);

	SIM900_SendCommand("AT+CIFSR\r\n", "", CMD_DELAY);

	SIM900_SendCommand("AT+CSQ\r\n", "", CMD_DELAY);


  	mqttMessageLength_header = 16 + strlen(clientId);

  	mqtt_connect_message(mqttMessage_header, clientId);

  	char * topic ="fakrouna";


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


		  // Connect
		  SIM900_SendCommand("AT+CIPSTART=\"TCP\",\"broker.mqttdashboard.com\",\"1883\"\r\n","OK\n\r",  2000);

		  // Check
		  SIM900_SendCommand("AT+CIPSTATUS\r\n","OK\n\r",  CMD_DELAY);

		  SIM900_SendCommand("AT+CIPSEND\r\n","",  CMD_DELAY);

		  // Header

		  for (int k = 0; k < mqttMessageLength_header; k++) {

					  		HAL_UART_Transmit(&huart3, (uint8_t *)&mqttMessage_header[k], 1, 100);
					  		// SEND HEADER MQTT
		   }

		  uint8_t end_sig = 26;

		  HAL_UART_Transmit(&huart3, &end_sig, 1, 100);

		  HAL_Delay(100);

		  // SEND MQTT BODY

		  for( int x=0 ; x < 50; x++){



				char message[25];
				sprintf(message,"Fakrouna : %li",session);
				session++;
			  	mqttMessageLength_body = 4 + strlen(topic) + strlen(message);

			  	mqtt_publish_message(mqttMessage_body, topic, message);

			  SIM900_SendCommand("AT+CIPSEND\r\n"," ", CMD_DELAY);

				  for (int k = 0; k < mqttMessageLength_body; k++) {

					HAL_UART_Transmit(&huart3, (uint8_t *)&mqttMessage_body[k], 1, 100);

				  }

				  HAL_UART_Transmit(&huart3, &end_sig, 1, 100);
				  HAL_Delay(100);
		  }



		  HAL_Delay(100);


		  SIM900_SendCommand("AT+CIPCLOSE\r\n", "", CMD_DELAY);


	      HAL_Delay(10000);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int indexof(char *string , int c )
{
    return strchr(string, c) !=NULL ? (int)(strchr(string, c ) - string) : -1;
}


char *substring(char *string, int position, int length)
{

    char *p;
    int c = 0;

    if(length < 0){
    p = "0";
    }else{

    p = malloc(length+1);

    for (c = 0; c < length; c++)
    {
        *(p+c) = *(string+position-1);
        string++;
    }

    *(p+c) = '\0';
    }

   return p;
}



int MQTT_Init(void)
{

	connect = 0;
    int error = 0;

   // HAL_UART_Receive_IT(&huart3, &rx_data, 1);

    error = SIM900_SendCommand("AT\r\n", "OK\r\n", CMD_DELAY);

    error += SIM900_SendCommand("AT+CGATT=1\r\n", "OK\r\n", CMD_DELAY);


    error += SIM900_SendCommand("AT+CSTT=\"internet.ooredoo.tn\"", "OK\r\n", CMD_DELAY);

    error += SIM900_SendCommand("AT+CIICR\r\n", "OK\r\n", CMD_DELAY);

    SIM900_SendCommand("AT+CIFSR\r\n", "", CMD_DELAY);

    SIM900_SendCommand("AT+CSQ\r\n", "", CMD_DELAY);

    if (error == 0)
    {
    	printk("OK test pass");
        return error;
    }
    else
    {
        return error;
    }
}

void MQTT_Connect(void){

		/*SIM900_SendCommand(Connect,"OK\n\r",  2000);
		SIM900_SendCommand(IpStatu,"OK\n\r",  CMD_DELAY);
		SIM900_SendCommand(data," ",  CMD_DELAY);


		mqttMessageLength = 16 + strlen(clientId);
		printk("\n %d \n",mqttMessageLength);
		mqtt_connect_message(mqttMessage, clientId);


		SIM900_SendCommand((char*)mqttMessage,"",CMD_DELAY);

		 uint8_t end_sig = 26;


		  HAL_UART_Transmit(&huart3, &end_sig, 1, 100);
		  HAL_Delay(2000);

	sendMQTTMessage(clientId,"fakrouna","fakrssssssssssouna");*/


}

int SIM900_SendCommand(const char *command, char *reply, uint16_t delay)
{
    HAL_UART_Transmit_IT(&huart3, (unsigned char *)command,
                         (uint16_t)strlen(command));
    HAL_Delay(delay);



    if (strstr(mqtt_buffer, reply) != NULL)
    {
        clearRxBuffer();
        return 0;
    }
    clearRxBuffer();
    return 1;
}

void clearRxBuffer(void)
{
    rx_index = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
}


void clearMqttBuffer(void)
{
    mqtt_receive = 0;
    mqtt_index = 0;
    memset(mqtt_buffer, 0, sizeof(mqtt_buffer));
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		    rx_buffer[rx_index++] = rx_data;

		    if (connect == 0)
		    {
		        if (strstr((char *)rx_buffer, "\r\n") != NULL && rx_index == 2)
		        {
		            rx_index = 0;
		        }
		        else if (strstr((char *)rx_buffer, "\r\n") != NULL)
		        {
		            memcpy(mqtt_buffer, rx_buffer, sizeof(rx_buffer));
		            clearRxBuffer();
		            if (strstr(mqtt_buffer, "DY CONNECT\r\n"))
		            {
		           connect = 0;
		            }
		            else if (strstr(mqtt_buffer, "CONNECT\r\n"))
		            {
		            connect = 1;
		            }
		        }
		    }

		    if (strstr((char *)rx_buffer, "CLOSED\r\n") || strstr((char *)rx_buffer, "ERROR\r\n") || strstr((char *)rx_buffer, "DEACT\r\n"))
		    {
		   connect = 0;
		    }

		    if (strstr((char *)mqtt_buffer, "+CSQ:") != NULL ) {

		        int start = indexof((char *)mqtt_buffer, ':')   +2;
		        int end = indexof((char *)mqtt_buffer, ',')  + 1;

		        signal =  atoi( substring((char *)mqtt_buffer,start, (end -start)));

		        printk("%d",signal);

		    }

		    if (rx_index >= sizeof(mqtt_buffer))
		    {
		        clearRxBuffer();
		        clearMqttBuffer();
		    }
		    HAL_UART_Receive_IT(&huart3, &rx_data, 1);
		}
}


void printk(const char *fmt, ...)
{
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart4, (uint8_t*) buffer, len, -1);
}


void sendMQTTMessage(char* clientId, char* topic, char* message)
{


/*

	HAL_Delay(2000);


	HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), 100);
	HAL_Delay(500);



 mqttMessageLength = 4 + strlen(topic) + strlen(message);

 printk("\n %d \n", mqttMessageLength);
 mqtt_publish_message(mqttMessage, topic, message);

 for (int k = 0; k < mqttMessageLength; k++) {

		HAL_UART_Transmit(&huart3, (uint8_t *)&mqttMessage[k], 1, 100);
 }


 uint8_t end_sig = 26;


  HAL_UART_Transmit(&huart3, &end_sig, 1, 100);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart3, (uint8_t *)TCPClose, strlen(TCPClose), 100);
  HAL_Delay(500);
*/

}


void mqtt_connect_message(uint8_t * mqtt_message, char * client_id) {

uint8_t i = 0;
uint8_t client_id_length = strlen(client_id);

mqtt_message[0] = 16;                      // MQTT Message Type CONNECT
mqtt_message[1] = 14 + client_id_length;   // Remaining length of the message

mqtt_message[2] = 0;                       // Protocol Name Length MSB
mqtt_message[3] = 6;                       // Protocol Name Length LSB
mqtt_message[4] = 77;                      // ASCII Code for M
mqtt_message[5] = 81;                      // ASCII Code for Q
mqtt_message[6] = 73;                      // ASCII Code for I
mqtt_message[7] = 115;                     // ASCII Code for s
mqtt_message[8] = 100;                     // ASCII Code for d
mqtt_message[9] = 112;                     // ASCII Code for p
mqtt_message[10] = 3;                      // MQTT Protocol version = 3
mqtt_message[11] = 2;                      // conn flags
mqtt_message[12] = 0;                      // Keep-alive Time Length MSB
mqtt_message[13] = 15;                     // Keep-alive Time Length LSB


mqtt_message[14] = 0;                      // Client ID length MSB
mqtt_message[15] = client_id_length;       // Client ID length LSB

// Client ID
for(i = 0; i < client_id_length + 16; i++){
mqtt_message[16 + i] = client_id[i];
}

}

void mqtt_publish_message(uint8_t * mqtt_message, char * topic, char * message) {

uint8_t i = 0;
uint8_t topic_length = strlen(topic);
uint8_t message_length = strlen(message);

mqtt_message[0] = 48;                                  // MQTT Message Type CONNECT
mqtt_message[1] = 2 + topic_length + message_length;   // Remaining length
mqtt_message[2] = 0;                                   // MQTT Message Type CONNECT
mqtt_message[3] = topic_length;                        // MQTT Message Type CONNECT

// Topic
for(i = 0; i < topic_length; i++){
mqtt_message[4 + i] = topic[i];
}

// Message
for(i = 0; i < message_length; i++){
mqtt_message[4 + topic_length + i] = message[i];
}

}

void mqtt_disconnect_message(uint8_t * mqtt_message) {
	mqtt_message[0] = 0xE0; // msgtype = connect
	mqtt_message[1] = 0x00; // length of message (?)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
