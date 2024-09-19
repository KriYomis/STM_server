/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/apps/httpd.h"
#include <stdio.h>
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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char temperature1[16];
char pressure1[16];
char humidity1[16];
uint8_t green = 0;
uint8_t blue = 0;
uint8_t red = 0;
uint8_t tempread[8];
uint8_t config[2];
uint8_t addr[1];
uint8_t comp[32];
uint8_t buf[2];
char uart_buf[50];
int uart_len;
int indx;
volatile int temperature_raw;
volatile int pressure_raw;
volatile int humidity_raw;
int finaltemp = 0;
double finalpres;
double finalhumid;
int t_fine = 0;
float outtemp, outpres, outhum;
unsigned short dig_T1, dig_P1, dig_H1, dig_H3;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9,
dig_H2, dig_H4, dig_H5, dig_H6;
const char *LEDGREEN(int index, int numParams, char *pcParam[], char *pcValue)
{
	if(green == 0)
	{
		green++;
	}
	else if(green == 1)
	{
		green--;
	}
	return "/LED.html";
}
const char *LEDRED(int LED, int numParam, char *pcParam[], char *pcValue)
{
	if(red == 0)
	{
		red++;
	}
	else if(red == 1)
	{
		red--;
	}
	return "/LED.html";
}
const char *LEDBLUE(int LED, int numParams, char *pcParam[], char *pcValue)
{
	if(blue == 0)
	{
		blue++;
	}
	else if(blue == 1)
	{
		blue--;
	}
	return "/LED.html";
}

static u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
    switch (iIndex) {
        case 0: sprintf(pcInsert, "%s", finaltemp);
        break;
        case 1: sprintf(pcInsert, "%s", finalpres);
        break;
        case 2: sprintf(pcInsert, "%s", finalhumid);
        break;
    }
return  0;
}

char const *ssi_tags[] = {
    "x",
    "y",
    "z",
};


void Get_ID()
{
	addr[0] = 0xD0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, buf, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Wake_Up()
{
	config[0] = 0x74;
	config[1] = 0x25;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, config, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	config[0] = 0x75;
	config[1] = 0xA0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, config, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	config[0] = 0x72;
	config[1] = 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, config, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Get_Trimming_Data()
{
	addr[0] = 0x88;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, comp, 6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	dig_T1 = (comp[0])+(comp[1]<<8);
	dig_T2 = (comp[2])+(comp[3]<<8);
	dig_T3 = (comp[4])+(comp[5]<<8);
}

void Get_Trim_Press()
{
	addr[0] = 0x8E;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, comp, 20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	dig_P1 = (comp[0])+(comp[1]<<8);
	dig_P2 = (comp[2])+(comp[3]<<8);
	dig_P3 = (comp[4])+(comp[5]<<8);
	dig_P4 = (comp[6])+(comp[7]<<8);
	dig_P5 = (comp[8])+(comp[9]<<8);
	dig_P6 = (comp[10])+(comp[11]<<8);
	dig_P7 = (comp[12])+(comp[13]<<8);
	dig_P8 = (comp[14])+(comp[15]<<8);
	dig_P9 = (comp[16])+(comp[17]<<8);
	dig_H1 = (comp[19]);
}
void Get_Trim_Hum()
{
	addr[0] = 0xE1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, comp, 8);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	dig_H2 = (comp[0])+(comp[1]<<8);
	dig_H3 = (comp[2]);
	dig_H4 = (comp[3]<<4)+(comp[4]>>4);
	dig_H5 = (comp[4]>>4)+(comp[5]<<4);
	dig_H6 = (comp[6]);
}
void Get_Raw_T()
{
	Wake_Up();
	addr[0] = 0xF7;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, tempread, 8);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	temperature_raw = (tempread[3]<<12)+(tempread[4]<<4)+(tempread[5]>>4);
}
void Get_Raw_P()
{
	Wake_Up();
	addr[0] = 0xF7;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
	HAL_SPI_Receive_DMA(&hspi1, tempread, 8);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	pressure_raw = (tempread[0]<<12)+(tempread[1]<<4)+(tempread[2]>>4);
}
void Get_Raw_H()
{
	Wake_Up();
		addr[0] = 0xF7;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(&hspi1, addr, 1);
		HAL_SPI_Receive_DMA(&hspi1, tempread, 8);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		humidity_raw = (tempread[6]<<8)+(tempread[7]);
	}

void Compensate_T()
{
	int var1, var2;
	var1 = ((((temperature_raw >> 3)-((int)dig_T1 << 1)))*((int)dig_T2)) >> 11;
	var2 = (((((temperature_raw >> 4)- ((int)dig_T1))*((temperature_raw >> 4)-((int)dig_T1)))>> 12)*((int)dig_T3)) >> 14;
	t_fine =(var1 + var2);
	finaltemp = (t_fine * 5 +128) >> 8;
}
void Compensate_P()
{
	double var1, var2;
	var1 = ((double)t_fine/2.0)- 64000.0;
	var2 = var1 * var1 * ((double)dig_P6 / 32768.0);
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0) + (((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 /524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0+var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0;
	}
	finalpres = 1048576.0 - (double)pressure_raw;
	finalpres = (finalpres-(var2/4096.0))* 6250.0 / var1;
	var1 = ((double)dig_P9)*finalpres*finalpres / 2147483648.0;
	var2 = finalpres *((double)dig_P8) / 32768.0;
	finalpres = finalpres + (var1 + var2 + ((double)dig_P7)) / 16.0;
}

void Componsate_H()
{
	finalhumid = (((double)t_fine)- 76800.0);
	finalhumid = (humidity_raw - (((double)dig_H4)*64.0 + ((double)dig_H5)/ 16384.0 *
			finalhumid)) * (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) /
					67108864.0 * finalhumid * (1.0 + ((double)dig_H3)/ 67108864.0 * finalhumid)));
	finalhumid = finalhumid * (1.0 - ((double)dig_H1) * finalhumid / 54288.0);
	if(finalhumid > 100)
		finalhumid = 100.0;
	else if (finalhumid < 0.0)
		finalhumid = 0.0;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	const tCGI LED[] = {
	  {
	    "/led_green.cgi",
		LEDGREEN
	  },
	  {
	    "/led_blue.cgi",
	    LEDBLUE
	  },
	  {
		"/led_red.cgi",
		LEDRED
	  }
	};

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
  MX_USB_OTG_FS_PCD_Init();
  MX_LWIP_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  httpd_init();
  http_set_cgi_handlers(LED, LWIP_ARRAYSIZE(LED));
  http_set_ssi_handler(ssi_handler,ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


  Get_ID();
  Wake_Up();

  Get_Trimming_Data();
  Get_Trim_Press();
  Get_Trim_Hum();

  uart_len = sprintf(uart_buf, "ID_0x%02x\r\n", buf[0]);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "T1_%d\r\n", dig_T1);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "T2_%d\r\n", dig_T2);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "T3_%d\r\n", dig_T3);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P1_%d\r\n", dig_P1);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P2_%d\r\n", dig_P2);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P3_%d\r\n", dig_P3);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P4_%d\r\n", dig_P4);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P5_%d\r\n", dig_P5);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P6_%d\r\n", dig_P6);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P7_%d\r\n", dig_P7);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P8_%d\r\n", dig_P8);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "P9_%d\r\n", dig_P9);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H1_%d\r\n", dig_H1);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H2_%d\r\n", dig_H2);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H3_%d\r\n", dig_H3);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H4_%d\r\n", dig_H4);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H5_%d\r\n", dig_H5);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  uart_len = sprintf(uart_buf, "H6_%d\r\n", humidity_raw);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MX_LWIP_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Get_Raw_T();
	  Get_Raw_P();
	  Get_Raw_H();
	  Compensate_T();
	  Compensate_P();
	  Componsate_H();
	  outtemp = (float)finaltemp / 100;
	  uart_len = sprintf(uart_buf, "TEMP_%0.2f\r\n", outtemp);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);

	  outpres = (float)finalpres;
	  uart_len = sprintf(uart_buf, "PRESS_%0.1f\r\n", outpres);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);

	  outhum = (float)finalhumid;
	  uart_len = sprintf(uart_buf, "HUM_%0.1f\r\n", outhum);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 100);

	  if(green == 1)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  }
	  else if(green==0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  }
	  if(blue == 1)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	  }
	  else if(blue==0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	  }
	  if(red == 1)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

	  }
	  else if(red==0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
