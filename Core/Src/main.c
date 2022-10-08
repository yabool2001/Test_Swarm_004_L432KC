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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_HANDLER						htim6
#define TIM_INSTANCE					TIM6
#define SWARM_UART_HANDLER				&huart1
#define DBG_UART_HANDLER				&huart2
#define SWARM_UART_INSTANCE				USART1
#define DBG_UART_INSTANCE				USART2
#define UART_TX_TIMEOUT					100
#define SWARM_UART_RX_MAX_BUFF_SIZE		100
#define SWARM_ANSWER_MAX_BUFF_SIZE		100
#define SWARM_UART_TX_MAX_BUFF_SIZE		193
#define DBG_UART_TX_MAX_BUFF_SIZE		200
#define SWARM_TD_PAYLOAD_MAX_BUFF_SIZE	192
#define M138_INITIALISED				127
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char             						swarm_uart_rx_buff[SWARM_UART_RX_MAX_BUFF_SIZE] ;
char             						swarm_uart_tx_buff[SWARM_UART_TX_MAX_BUFF_SIZE] ;
char             						dbg_uart_tx_buff[DBG_UART_TX_MAX_BUFF_SIZE] ;
uint8_t									tim_on = 0 ;
uint8_t									answer_from_swarm = 0 ; // To jest bardzo ważne, bo 0 oznacza otwarty dma, którego nie można otwierać drugi raz, bo się zawiesi
uint8_t									m138_init_status_reg = 0 ; // b0: dev_id, b1: rt rate=0, b2: pw rate = 0, b3: dt rate = 0, b4: gs rate = 0, b5: gj rate = 0, b6: gn rate = 0,
uint32_t								m138_dev_id = 0 ;
float									m138_voltage = 0 ;
char									m138_fix[50] ;

// SWARM AT Commands
const char*			cs_at					= "$CS" ;
const char*			rt_0_at					= "$RT 0" ;
const char*			rt_q_rate_at			= "$RT ?" ;
const char*			pw_0_at					= "$PW 0" ;
const char*			pw_q_rate_at			= "$PW ?" ;
const char*			pw_mostrecent_at		= "$PW @" ;
const char*			dt_0_at					= "$DT 0" ;
const char*			dt_q_rate_at			= "$DT ?" ;
const char*			gs_0_at					= "$GS 0" ;
const char*			gs_q_rate_at			= "$GS ?" ;
const char*			gj_0_at					= "$GJ 0" ;
const char*			gj_q_rate_at			= "$GJ ?" ;
const char*			gn_0_at					= "$GN 0" ;
const char*			gn_q_rate_at			= "$GN ?" ;
const char*			gn_mostrecent_at		= "$GN @" ;
const char*			sl_at_comm				= "$SL S=10" ;
// SWARM AT Answers
const char*         cs_answer				= "$CS DI=0x\0" ;
const char*         rt_ok_answer			= "$RT OK*22\0" ;
const char*         rt_0_answer				= "$RT 0*16\0" ;
const char*         pw_ok_answer			= "$PW OK*23\0" ;
const char*         pw_0_answer				= "$PW 0*17\0" ;
const char*         pw_mostrecent_answer	= "$PW \0" ;
const char*         dt_ok_answer			= "$DT OK*34\0" ;
const char*         dt_0_answer				= "$DT 0*00\0" ;
const char*         gs_ok_answer			= "$GS OK*30\0" ;
const char*         gs_0_answer				= "$GS 0*04\0" ;
const char*         gj_ok_answer			= "$GJ OK*29\0" ;
const char*         gj_0_answer				= "$GJ 0*1d\0" ;
const char*         gn_ok_answer			= "$GN OK*2d\0" ;
const char*         gn_0_answer				= "$GN 0*19\0" ;
const char*         gn_mostrecent_answer	= "$GN \0" ;
const char*			td_ok_answer			= "$TD OK,\0" ;
const char*         sl_ok_answer			= "$SL OK*3b\0" ;
const char*         sl_wake_answer			= "$SL WAKE\0" ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
uint8_t 			store_m138_dev_id 			( uint32_t* , char* ) ;
uint8_t 			store_m138_voltage 			( float* , char* ) ;
uint8_t				store_m138_fix				( char* , char* ) ;
void 				reset_m138_var 				( void ) ;
void				m138_init 					( void ) ;
uint8_t				swarm_cc 					( const char* , const char* ) ;
void 				tim_init 					( void ) ;
void 				tim_start 					( void ) ;
void 				green_toggle 				( void ) ;
HAL_StatusTypeDef 	send_string_2_swarm_uart 	( char* ) ;
HAL_StatusTypeDef 	send_string_2_dbg_uart 		( char* ) ;
HAL_StatusTypeDef 	receive_swarm_uart_dma 		( void ) ;
uint8_t				nmea_checksum				( const char* , size_t ) ;
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  send_string_2_dbg_uart ( "Hello! Test_Swarm_004_L432KC started\n" ) ;
  tim_init () ;
  m138_init () ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( m138_init_status_reg != M138_INITIALISED )
		  m138_init () ;
	  if ( swarm_cc ( pw_mostrecent_at , pw_mostrecent_answer ) )
		  store_m138_voltage ( &m138_voltage , swarm_uart_rx_buff ) ;
	  if ( swarm_cc ( gn_mostrecent_at , gn_mostrecent_answer ) )
		  store_m138_fix ( m138_fix , swarm_uart_rx_buff ) ;
	  sprintf ( dbg_uart_tx_buff , "$TD HD=60,\"%u;%s\"\n" , (unsigned int) m138_dev_id , m138_fix ) ;
	  send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
/*
	  if ( swarm_cc ( sl_at_comm , sl_ok_answer ) )
	  {
		  sprintf ( dbg_uart_tx_buff , "Swarm went sleep for 10s.\n" ) ;
		  send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
	  }
*/
	  reset_m138_var () ;
	  HAL_Delay ( 12000 ) ;
	  //HAL_UART_Receive ( &huart1 , (uint8_t*) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE , 1000 ) ;
	  //HAL_PWREx_EnterSTOP2Mode ( PWR_STOPENTRY_WFI ) ;
	  //HAL_PWREx_EnterSTOP1Mode ( PWR_STOPENTRY_WFI ) ;
	  //HAL_PWREx_EnterSTOP0Mode ( PWR_STOPENTRY_WFI ) ;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim6.Init.Prescaler = 32000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t store_m138_dev_id ( uint32_t* id , char* s )
{
	if ( ! strstr ( s , "DI=0x" ) )
		return 0 ;
	s = strtok ( (char*) s , "=" ) ;
	s = strtok ( NULL , "," ) ;
	*id = (uint32_t) strtol ( s , NULL , 16 ) ;
	return 1 ;
}
uint8_t store_m138_voltage ( float* d , char* s )
{
	if ( ! strstr ( s , "$PW " ) )
		return 0 ;
	s = strtok ( (char*) s , " " ) ;
	s = strtok ( NULL , "," ) ;
	*d = (float) strtof ( s , NULL ) ;
	return 1 ;
}
uint8_t store_m138_fix ( char* d , char* s )
{
	if ( ! strstr ( s , "$GN " ) )
		return 0 ;
	s = strtok ( (char*) s , " " ) ;
	s = strtok ( NULL , "*" ) ;
	size_t l =  strlen ( s ) ;
	memcpy ( d , s , l ) ;
	d[l] = '\0' ;
	return 1 ;
}
void reset_m138_var ()
{
	m138_voltage = 0 ;
	m138_fix[0] = '\0' ;
}
/*
uint8_t swarm_cc ( const char* at_command , const char* expected_answer )
{
	uint8_t try ;
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;
	sprintf ( swarm_uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;

	for ( try = 0 ; try < 5 ; try++ )
	{
		receive_swarm_uart_dma () ;
		tim_start () ;
		send_string_2_swarm_uart ( swarm_uart_tx_buff ) ;
		while ( tim_on )
			if ( strncmp ( swarm_uart_rx_buff , expected_answer , strlen ( expected_answer ) ) == 0 )
			{
				sprintf ( dbg_uart_tx_buff , "try no. %u success for %s\n" , try , at_command ) ;
				send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
				return 1 ;
			}
			else
				break ;
	}
	sprintf ( dbg_uart_tx_buff , "%s %s\n" , (char*) expected_answer , "not received." ) ;
	send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
	return 0 ;
}
*/
uint8_t swarm_cc ( const char* at_command , const char* expected_answer )
{
	uint8_t try ;
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;
	sprintf ( swarm_uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;

	for ( try = 0 ; try < 5 ; try++ )
	{
		if ( answer_from_swarm == 0 )
			if ( receive_swarm_uart_dma () != HAL_OK )
			{
				sprintf ( dbg_uart_tx_buff , "try no. %u receive_swarm_uart_dma () != HAL_OK for %s\n" , try , at_command ) ;
				send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
			}
		tim_start () ;
		send_string_2_swarm_uart ( swarm_uart_tx_buff ) ;
		while ( tim_on )
			if ( answer_from_swarm == 1 )
			{
				sprintf ( dbg_uart_tx_buff , "try no. %u answer_from_swarm = 1 for %s\n" , try , at_command ) ;
				send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
				answer_from_swarm = 0 ;
				if ( strncmp ( swarm_uart_rx_buff , expected_answer , strlen ( expected_answer ) ) == 0 )
				{
					sprintf ( dbg_uart_tx_buff , "try no. %u success for %s\n" , try , at_command ) ;
					send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
					return 1 ;
				}
				else
					break ;
			}
	}
	sprintf ( dbg_uart_tx_buff , "%s %s\n" , (char*) expected_answer , "not received." ) ;
	send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
	return 0 ;
}

/*
uint8_t swarm_cc ( const char* at_command , const char* expected_answer )
{
	uint8_t try ;
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;
	sprintf ( swarm_uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;

	for ( try = 0 ; try < 5 ; try++ )
	{
		if ( receive_swarm_uart_dma () == HAL_OK )
		{
			tim_start () ;
			send_string_2_swarm_uart ( swarm_uart_tx_buff ) ;
			while ( tim_on )
				if (answer_from_swarm == 1 )
				{
					if ( strncmp ( swarm_uart_rx_buff , expected_answer , strlen ( expected_answer ) ) == 0 )
					{
						return 1 ;
					}
					else
						break ;
				}
		}
		else
		{
			sprintf ( swarm_uart_tx_buff , "try no. %u receive_swarm_uart_dma () != HAL_OK\n" , try ) ;
			send_string_2_dbg_uart ( swarm_uart_tx_buff ) ;
		}
	}
	sprintf ( swarm_uart_tx_buff , "%s %s\n" , (char*) expected_answer , "not received." ) ;
	send_string_2_dbg_uart ( swarm_uart_tx_buff ) ;
	return 0 ;
}
*/
void m138_init ()
{
	if ( swarm_cc ( cs_at , cs_answer ) )
		if ( store_m138_dev_id ( &m138_dev_id , swarm_uart_rx_buff ) )
			m138_init_status_reg = m138_init_status_reg | 1 ;
	if ( swarm_cc ( rt_0_at , rt_ok_answer ) )
		if ( swarm_cc ( rt_q_rate_at , rt_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 2 ;
	if ( swarm_cc ( pw_0_at , pw_ok_answer ) )
		if ( swarm_cc ( pw_q_rate_at , pw_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 4 ;
	if ( swarm_cc ( dt_0_at , dt_ok_answer ) )
		if ( swarm_cc ( dt_q_rate_at , dt_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 8 ;
	if ( swarm_cc ( gs_0_at , gs_ok_answer ) )
		if ( swarm_cc ( gs_q_rate_at , gs_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 16 ;
	if ( swarm_cc ( gj_0_at , gj_ok_answer ) )
		if ( swarm_cc ( gj_q_rate_at , gj_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 32 ;
	if ( swarm_cc ( gn_0_at , gn_ok_answer ) )
		if ( swarm_cc ( gn_q_rate_at , gn_0_answer ) )
			m138_init_status_reg = m138_init_status_reg | 64 ;

	sprintf ( dbg_uart_tx_buff , "%s%u\n" , "m138_init_status_reg = " , m138_init_status_reg ) ;
	send_string_2_dbg_uart ( dbg_uart_tx_buff ) ;
}

void green_toggle ()
{
	HAL_GPIO_TogglePin ( GREEN_GPIO_Port , GREEN_Pin ) ;
}

HAL_StatusTypeDef send_string_2_swarm_uart ( char* s )
{
	return HAL_UART_Transmit ( SWARM_UART_HANDLER , (uint8_t *) s , strlen ( s ) , UART_TX_TIMEOUT ) ;
}
HAL_StatusTypeDef send_string_2_dbg_uart ( char* s )
{
	return HAL_UART_Transmit ( DBG_UART_HANDLER , (uint8_t *) s , strlen ( s ) , UART_TX_TIMEOUT ) ;
}

uint8_t nmea_checksum ( const char *message , size_t len )
{
	size_t i = 0 ;
	uint8_t cs ;
	if ( message [0] == '$' )
		i++ ;
	for ( cs = 0 ; ( i < len ) && message [i] ; i++ )
		cs ^= ( (uint8_t) message [i] ) ;
	return cs;
}

void tim_init ()
{
	__HAL_TIM_CLEAR_IT ( &TIM_HANDLER , TIM_IT_UPDATE ) ;
}
void tim_start ()
{
	HAL_TIM_Base_Start_IT ( &TIM_HANDLER ) ;
	tim_on = 1 ;
}
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM_INSTANCE )
	{
		HAL_TIM_Base_Stop_IT ( &TIM_HANDLER ) ;
		tim_on = 0 ;
	}
}

HAL_StatusTypeDef receive_swarm_uart_dma ()
{
	HAL_StatusTypeDef r = HAL_UARTEx_ReceiveToIdle_DMA ( SWARM_UART_HANDLER , (uint8_t*) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
	__HAL_DMA_DISABLE_IT ( &hdma_usart1_rx, DMA_IT_HT ) ; //Disable Half Transfer interrupt.
	return r ;
}

void HAL_UARTEx_RxEventCallback ( UART_HandleTypeDef *huart , uint16_t Size )
{
    if ( huart->Instance == SWARM_UART_INSTANCE )
    {
    	answer_from_swarm = 1 ;
    	swarm_uart_rx_buff[Size] = 0 ;
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
