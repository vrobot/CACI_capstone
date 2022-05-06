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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
#include "lora_sx1276.h"
#include "string.h"
#include "arm_math.h"
#include "fir_coeffs1400-2600hz.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LORA_TRANSMIT 1
#define CLAP_THRESHOLD 100.0f
#define CLAP_DURATION 5
#define MIC_SAMPLE_FREQUENCY 44000
#define MIC_SAMPLES_PER_MS (MIC_SAMPLE_FREQUENCY/1000)  // == 48
#define MIC_NUM_CHANNELS 1
#define MIC_MS_PER_PACKET 20
#define MIC_SAMPLES_PER_PACKET (MIC_SAMPLES_PER_MS * MIC_MS_PER_PACKET) // == 960
#define MOVING_AVG_THRESHOLD CLAP_THRESHOLD
#define MOVING_AVG_LEN CLAP_DURATION
#define NUM_SEND_PACKETS 10
#define SEND_LEN (255 * NUM_SEND_PACKETS)
#define NODE 1

#if (NODE == 1)
#define NODE_DELAY 0
#elif (NODE == 2)
#define NODE_DELAY 100000000
#elif (NODE == 3)
#define NODE_DELAY 200000000
#endif

#define BLOCK_SIZE MIC_SAMPLES_PER_PACKET / 2
#define FILTER_LEN 101
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_b;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile int32_t *_sampleBuffer[MIC_SAMPLES_PER_PACKET * 2];
int16_t circular_buf_mov[MOVING_AVG_LEN];
int16_t circular_buf_mov_right[MOVING_AVG_LEN];
uint32_t head_mov;
uint32_t tail_mov;

int16_t circular_buf_full[SEND_LEN];
uint32_t head_full;
uint32_t tail_full;
float moving_sum;
float moving_avg;
int64_t moving_sum_left;
float moving_avg_left;
int64_t moving_sum_right;
float moving_avg_right;
bool _running;
uint16_t counter;
int done;
uint32_t devID = NODE;
uint32_t startPadding = 0xABABABAB;
uint32_t endMetaPadding = 0xCDCDCDCD;
uint32_t endPadding = 0xEFEFEFEF;
lora_sx1276 lora;
uint8_t iterfacing_success[] = "Interfacing SUCCESS";
uint8_t iterfacing_failed[] = "Interfacing FAILED";
uint8_t transmission_success[] = "Transmission SUCCESS";
uint8_t transmission_failed[] = "Transmission FAILED";

uint8_t GPS_buffer[2048];
char buffStr[2048];
char nmeaSnt[80];
uint8_t GPS_latest_data[65];

arm_fir_instance_f32 S;
float buffer[FILTER_LEN + BLOCK_SIZE - 1];
uint32_t blockSize = BLOCK_SIZE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendData(volatile int32_t *data_in);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t timerVal;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// uint32_t test = TIM2->CNT;
	// HAL_UART_Transmit(&huart1, (uint8_t*)&test, 4, 70);
	__HAL_TIM_SET_COUNTER(htim,0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2){
		char *string;
		memset(buffStr, 0, 2048);
		sprintf(buffStr, "%s", GPS_buffer);
		string = strdup(buffStr);

		char* token;
		while ((token = strsep(&string, "\n")) != NULL) {
			memset(nmeaSnt, 0, 80);
			sprintf(nmeaSnt, "%s\n\r", token);
			if ((strstr(nmeaSnt, "$GPGLL") != 0) && strlen(nmeaSnt) > 48 && strlen(nmeaSnt) < 65) {
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
				//Raw Data
				memset(GPS_latest_data, 0, 65);
				memcpy(GPS_latest_data, nmeaSnt, strlen(nmeaSnt));
				// HAL_UART_Transmit(&huart1, (uint8_t*)GPS_latest_data, strlen(GPS_latest_data), 70);
			}
		}
		HAL_UART_Receive_DMA(&huart2, GPS_buffer, 2048);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  _running = false;
  head_mov = 0;
  tail_mov = 1;
  head_full = 0;
  tail_full = 1;
  moving_sum = 0;
  moving_avg = 0;
  moving_sum_left = 0;
  moving_avg_left = 0;
  moving_sum_right = 0;
  moving_avg_right = 0;
  counter = 0;
  done = 0;
  memset(circular_buf_mov, 0, MOVING_AVG_LEN*sizeof(int16_t));
  memset(circular_buf_mov_right, 0, MOVING_AVG_LEN*sizeof(int16_t));
  memset(circular_buf_full, 0, SEND_LEN*sizeof(int16_t));
  memset(_sampleBuffer, 0, MIC_SAMPLES_PER_PACKET*2*sizeof(int32_t));
  arm_fir_init_f32(&S, FILTER_LEN, (float *) coeffs, buffer, blockSize);



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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_DMA(&huart2, GPS_buffer, 2048);

#if LORA_TRANSMIT
  uint8_t res = lora_init(&lora, &hspi1, NSS_GPIO_Port, NSS_Pin, LORA_BASE_FREQUENCY_US);
  if (res != LORA_OK) {
	  HAL_UART_Transmit(&huart1, iterfacing_failed, sizeof(iterfacing_failed), 1000);
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.ClockSource = SAI_CLKSOURCE_NA;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */
  if ((HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*) _sampleBuffer, MIC_SAMPLES_PER_PACKET * 2)) == HAL_OK) {
	  HAL_Delay(1000);
	  _running = true;
  }
  /* USER CODE END SAI1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF3 PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	sendData(_sampleBuffer);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	sendData(&_sampleBuffer[MIC_SAMPLES_PER_PACKET]);
}

void sendData(volatile int32_t *data_in) {

	float data_in_float[MIC_SAMPLES_PER_PACKET / 2];
	float filt[MIC_SAMPLES_PER_PACKET / 2];
	volatile int32_t *data_in_tmp = data_in;
	for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {
		int16_t tmp = (int16_t) ((data_in_tmp[0]>>8) & 0xffff);
		data_in_float[i] = (float) tmp;
		data_in_tmp += 2;
	}
	arm_fir_f32(&S, data_in_float, filt, blockSize);


	if (_running) {
		for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {

			moving_sum += (int16_t) fabsf(filt[i]); // * abs(sample_right);
			moving_sum -= (int16_t) abs(circular_buf_mov[tail_mov]); // * abs(circular_buf_mov_right[tail_mov]);
			circular_buf_mov[tail_mov] = (int16_t) filt[i];
			circular_buf_full[tail_full] = (int16_t) filt[i];
			moving_avg = moving_sum / MOVING_AVG_LEN;

			if ((counter > 0) && (!done)){
				counter++;
				if (counter == (SEND_LEN/2)){ // TODO: need to change this to SEND_LEN-100;

					for(int delay = 0; delay < NODE_DELAY; delay++); // non-blocking delay used to offset the transmissions of each node to prevent garbled transmissions

					uint8_t GPS_data_len = (uint8_t) strlen((char *) GPS_latest_data);
					uint8_t metaData[16+GPS_data_len];
					memcpy(metaData, &startPadding, 4);
					memcpy(metaData+4, &timerVal, 4);
					memcpy(metaData+8, &devID, 4);
					memcpy(metaData+12, GPS_latest_data, GPS_data_len);
					memcpy(metaData+12+GPS_data_len, &endMetaPadding, 4);

					uint8_t sendBuf[SEND_LEN*2];
					memcpy(sendBuf, &circular_buf_full[tail_full], ((SEND_LEN)-tail_full)*2);
					memcpy(&sendBuf[(SEND_LEN-tail_full)*2], circular_buf_full, tail_full*2);

#if LORA_TRANSMIT
					uint8_t packet_res = lora_send_packet(&lora, metaData, 16+GPS_data_len);

					if (packet_res != LORA_OK) {
						HAL_UART_Transmit(&huart1, &packet_res, sizeof(packet_res), 1000);
					}
					else {
						HAL_UART_Transmit(&huart1, transmission_success, sizeof(transmission_success), 1000);
					}

					while(lora_is_transmitting(&lora)); // non-blocking delay

#else
					HAL_UART_Transmit(&huart1, (uint8_t*) sendBuf, SEND_LEN*2, 1000);
#endif

#if LORA_TRANSMIT

					for(int send_loop_cnt = 0; send_loop_cnt < ((SEND_LEN*2)/255 + ((SEND_LEN*2) % 255 != 0)); send_loop_cnt++){
						packet_res = lora_send_packet(&lora, sendBuf+(255*send_loop_cnt), 255);
						if (packet_res != LORA_OK) {
							HAL_UART_Transmit(&huart1, &packet_res, sizeof(packet_res), 1000);
						}
						else {
							HAL_UART_Transmit(&huart1, transmission_success, sizeof(transmission_success), 1000);
						}
						HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_3);
						while(lora_is_transmitting(&lora)); // non-blocking delay
					}

					packet_res = lora_send_packet(&lora, (uint8_t *) &endPadding, 4);
					if (packet_res != LORA_OK) {
						HAL_UART_Transmit(&huart1, &packet_res, sizeof(packet_res), 1000);
					}
					else {
						HAL_UART_Transmit(&huart1, transmission_success, sizeof(transmission_success), 1000);
					}
#endif

					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
					done = 1;
				}
			}

			if ((moving_avg >= MOVING_AVG_THRESHOLD) && (counter == 0)){
				timerVal = TIM2->CNT;
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
				counter += 1;
				done = 0;
			}

			tail_mov = (tail_mov + 1) % (MOVING_AVG_LEN);
			head_mov = (head_mov + 1) % (MOVING_AVG_LEN);

			tail_full = (tail_full + 1) % (SEND_LEN);
			head_full = (head_full + 1) % (SEND_LEN);

			data_in += 2;
		}


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

