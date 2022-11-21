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
#include "stream.h"
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
const uint8_t cc1200_rx_settings[50*3] =
{
	0x00, 0x01, 0x08,
	0x00, 0x03, 0x09,
	0x00, 0x08, 0x1F,
	0x00, 0x0A, 0x9F, //RX bw
	0x00, 0x0B, 0x00,
	0x00, 0x0C, 0x5D,
	0x00, 0x0D, 0x00,
	0x00, 0x0E, 0x8A,
	0x00, 0x0F, 0xCB,
	0x00, 0x10, 0xAC,
	0x00, 0x11, 0x00,
	0x00, 0x12, 0x45,
	0x00, 0x13, 0x3F, //symbol rate 2
	0x00, 0x14, 0x75, //symbol rate 1
	0x00, 0x15, 0x10, //symbol rate 0
	0x00, 0x16, 0x37,
	0x00, 0x17, 0xEC,
	0x00, 0x19, 0x11,
	0x00, 0x1B, 0x51,
	0x00, 0x1C, 0x87,
	0x00, 0x1D, 0x00,
	0x00, 0x20, 0x14,
	0x00, 0x26, 0x03,
	0x00, 0x27, 0x00,
	0x00, 0x28, 0x20,
	0x00, 0x2B, 0x3F,
	0x00, 0x2E, 0xFF,
	0x2F, 0x00, 0x1C,
	0x2F, 0x01, 0x02, //AFC, 0x22 - on, 0x02 - off
	0x2F, 0x05, 0x0D,
	0x2F, 0x0C, 0x57, //freq round((float)435000000/5000000*(1<<16))
	0x2F, 0x0D, 0x00, //freq
	0x2F, 0x0E, 0x00, //freq
	0x2F, 0x10, 0xEE,
	0x2F, 0x11, 0x10,
	0x2F, 0x12, 0x07,
	0x2F, 0x13, 0xAF,
	0x2F, 0x16, 0x40,
	0x2F, 0x17, 0x0E,
	0x2F, 0x19, 0x03,
	0x2F, 0x1B, 0x33,
	0x2F, 0x1D, 0x17,
	0x2F, 0x1F, 0x00,
	0x2F, 0x20, 0x6E,
	0x2F, 0x21, 0x1C,
	0x2F, 0x22, 0xAC,
	0x2F, 0x27, 0xB5,
	0x2F, 0x32, 0x0E,
	0x2F, 0x36, 0x03,
	0x2F, 0x91, 0x08,
};

const uint8_t cc1200_tx_settings[50*3] =
{
	0x00, 0x01, 0x08,
	0x00, 0x03, 0x09,
	0x00, 0x08, 0x1F,
	0x00, 0x0A, 0x59, //deviation
	0x00, 0x0B, 0x01, //deviation, LSB - exponent
	0x00, 0x0C, 0x5D,
	0x00, 0x0D, 0x00,
	0x00, 0x0E, 0x8A,
	0x00, 0x0F, 0xCB,
	0x00, 0x10, 0xAC,
	0x00, 0x11, 0x00,
	0x00, 0x12, 0x45,
	0x00, 0x13, 0x83, //symbol rate 2 - 24kSa/s
	0x00, 0x14, 0xA9, //symbol rate 1
	0x00, 0x15, 0x2A, //symbol rate 0
	0x00, 0x16, 0x37,
	0x00, 0x17, 0xEC,
	0x00, 0x19, 0x11,
	0x00, 0x1B, 0x51,
	0x00, 0x1C, 0x87,
	0x00, 0x1D, 0x00,
	0x00, 0x20, 0x14,
	0x00, 0x26, 0x03,
	0x00, 0x27, 0x00,
	0x00, 0x28, 0x20,
	0x00, 0x2B, 0x05, //power (0x01..0x3F)
	0x00, 0x2E, 0xFF,
	0x2F, 0x00, 0x1C,
	0x2F, 0x01, 0x22,
	0x2F, 0x05, 0x09, //16x upsampler, CFM enable
	0x2F, 0x0C, 0x57, //freq 439M = 0x57CCCD
	0x2F, 0x0D, 0xCC, //freq
	0x2F, 0x0E, 0xCD, //freq
	0x2F, 0x10, 0xEE,
	0x2F, 0x11, 0x10,
	0x2F, 0x12, 0x07,
	0x2F, 0x13, 0xAF,
	0x2F, 0x16, 0x40,
	0x2F, 0x17, 0x0E,
	0x2F, 0x19, 0x03,
	0x2F, 0x1B, 0x33,
	0x2F, 0x1D, 0x17,
	0x2F, 0x1F, 0x00,
	0x2F, 0x20, 0x6E,
	0x2F, 0x21, 0x1C,
	0x2F, 0x22, 0xAC,
	0x2F, 0x27, 0xB5,
	0x2F, 0x32, 0x0E,
	0x2F, 0x36, 0x03,
	0x2F, 0x91, 0x08,
};

uint8_t tx_data[8];
uint8_t rx_data[3]={0,0,0};

volatile uint8_t irq_pend=0;

float buff[81];			//look-back buffer for the FIR
uint8_t pushed=0;		//how many samples have we pushed to the buffer
float mac;				//multiply-accumulate result
int8_t sample=0;		//sample value, calculated

uint16_t byte=0;		//byte read from the stream
uint8_t dibit=6;		//dibit ^^ (MSB pair first)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 1, 10);
	//HAL_UART_Transmit_IT(&huart1, rx_data, 1);
	//HAL_GPIO_TogglePin(TST_PIN_GPIO_Port, TST_PIN_Pin);
	irq_pend=1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//BTN1
	if(GPIO_Pin==GPIO_PIN_0)
	{
		;//
	}

	//BTN2
	else if(GPIO_Pin==GPIO_PIN_1)
	{
		HAL_GPIO_TogglePin(SPK_MUTE_GPIO_Port, SPK_MUTE_Pin);
	}

	//BTN3
	else if(GPIO_Pin==GPIO_PIN_2)
	{
		HAL_GPIO_TogglePin(TST_PIN_GPIO_Port, TST_PIN_Pin);
	}
}

void CC1200_Reset(void)
{
	HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, 1);
	HAL_Delay(50);

	tx_data[0]=0x30;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 1, 5);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

void CC1200_Init(uint8_t *init_seq)
{
	for(uint8_t i=0; i<50; i++)
	{
		  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
		  if(init_seq[i*3])
			  HAL_SPI_Transmit(&hspi1, &init_seq[i*3], 3, 100);
		  else
			  HAL_SPI_Transmit(&hspi1, &init_seq[i*3+1], 2, 100);
		  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
		  HAL_Delay(10);
	}
}

void CC1200_Offset(int16_t offset)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0A;
	tx_data[2]=*((uint8_t*)&offset+1);
	tx_data[3]=*((uint8_t*)&offset);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 4, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

void CC1200_BurstModeIncr(uint8_t enable)
{
	tx_data[0]=0x2F;
	tx_data[1]=0x06;
	tx_data[2]=enable;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 3, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

void CC1200_RXMode(void)
{
	tx_data[0]=0x34;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

void CC1200_TXMode(void)
{
	tx_data[0]=0x35;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

void CC1200_RXStart(void)
{
	tx_data[0]=0x2F|0xC0;
	tx_data[1]=0x7D;
	tx_data[2]=0;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
}
void CC1200_TXStart(void)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x7E;
	tx_data[2]=0;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
}

void CC1200_TXRXEnd(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

//set frequency - burst mode
void CC1200_SetFreq(uint32_t freq)
{
	uint32_t val=(float)freq/5000000*(1<<16);

	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0C;
	tx_data[2]=(val>>16)&0xFF;
	tx_data[3]=(val>>8)&0xFF;
	tx_data[4]=val&0xFF;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 5, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}

//set power (0x01..0x3F)
void CC1200_SetPwr(uint8_t pwr)
{
	tx_data[0]=0x2B;
	tx_data[1]=pwr;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 2, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
}
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  //chip reset
  CC1200_Reset();
  HAL_Delay(100);

  //chip config
  CC1200_Init(cc1200_tx_settings);
  //CC1200_Init(cc1200_rx_settings);

  //frequency - override the setting in the init sequence
  CC1200_SetFreq(439000000);

  //power - override the setting in the init sequence
  CC1200_SetPwr(0x3C);

  //freq offset compensation
  CC1200_Offset(303);
  HAL_Delay(10);

  //mode - TX/RX
  CC1200_TXMode();
  //CC1200_RXMode();

  //dont increment address in burst mode
  CC1200_BurstModeIncr(0);

  //start write/read burst - tx/rx reg
  CC1200_TXStart();
  //CC1200_RXStart();

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //TX
	  if(irq_pend)
	  {
		  //do the filtering here
		  for(uint8_t i=0; i<41-1; i++)
			  buff[i]=buff[i+1];
		  pushed++;
		  pushed%=5;
		  if(pushed==0)
		  {
			  uint8_t s=(m17_stream[byte]>>dibit)&3;

			  if(s == 0b00)
				  buff[40]=1.0;
			  else if(s == 0b01)
				  buff[40]=3.0;
			  else if(s == 0b10)
				  buff[40]=-1.0;
			  else
				  buff[40]=-3.0;

			  if(dibit>0)
				  dibit-=2;
			  else
			  {
				  dibit=6;
				  byte++;
			  }

			  if(byte==STREAM_SIZE)
				  byte=0;
		  }
		  else
		  {
			  buff[40]=0.0;
		  }

		  mac=0.0;
		  for(uint8_t i=0; i<41; i++)
			  mac+=buff[i]*taps2[i];

		  sample=mac*40.0;

		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&sample, 1, 5);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)&sample, 1);

		  irq_pend=0;
	  }

	  //RX
	  /*if(irq_pend)
	  {
		  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 1, 10);
		  uint16_t s=((int8_t)rx_data[0]+128);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, s);
		  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		  irq_pend=0;
	  }*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPK_MUTE_GPIO_Port, SPK_MUTE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MIC_GAIN_Pin|TST_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MIC_MUTE_GPIO_Port, MIC_MUTE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin EXT_INT_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin|BTN3_Pin|EXT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPK_MUTE_Pin MIC_GAIN_Pin */
  GPIO_InitStruct.Pin = SPK_MUTE_Pin|MIC_GAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_MUTE_Pin */
  GPIO_InitStruct.Pin = MIC_MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MIC_MUTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TST_PIN_Pin */
  GPIO_InitStruct.Pin = TST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TST_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CC_GPIO0_Pin */
  GPIO_InitStruct.Pin = CC_GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CC_GPIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CC_GPIO2_Pin CC_GPIO3_Pin */
  GPIO_InitStruct.Pin = CC_GPIO2_Pin|CC_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
