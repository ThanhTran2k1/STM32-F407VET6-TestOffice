/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stm32_port.h"
#include "esp_loader.h"
#include "example_common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIGHER_BAUDRATE 230400

#define OFF_POWER	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define ON_POWER	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define HOLD_ESP_IO0		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)
#define RELEASE_ESP_IO0		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

//#define _DEBUG_
//#define TEST_CAN
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
extern uint8_t fTransmitCpt;
extern uint8_t fESPFlashProcess;
extern uint8_t uBuf[1024];
extern uint16_t current_len;
extern uint32_t written_len;
extern uint32_t binary_len;
extern uint8_t flash_block_len;

extern const uint8_t BlinkESP_ino_bin[];
extern const uint32_t BlinkESP_ino_bin_size ;
extern const uint8_t BlinkESP_ino_bootloader_bin[];
extern const uint32_t BlinkESP_ino_bootloader_bin_size ;
extern const uint8_t BlinkESP_ino_partitions_bin[];
extern const uint32_t BlinkESP_ino_partitions_bin_size ;

uint8_t APPICATION_ROM_1[1024];
uint8_t APPICATION_ROM_1[1024];
uint8_t APPICATION_ROM_1[1024];
uint8_t APPICATION_ROM_1[1024];
uint8_t APPICATION_ROM_1[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern uint8_t SMV_Step;

#ifdef _DEBUG_
uint8_t debug_buf[4000] = {};
#endif

esp_loader_error_t ESP_FLASH_Start(uint32_t address, uint32_t size);
esp_loader_error_t ESP_FLASH_WriteBlock(uint8_t* payload, uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Brief	: Copy len bytes of s to t from offset
 * @Param	: Source buffer
 * @Param	: Target buffer
 */
static void omemcpy(uint8_t *s, uint8_t *t, uint8_t offset, uint32_t len)
{
	for(uint32_t i = 0; i < len; i++)
	{
		t[i + offset] = s[i];
	}
}
/*
 * Brief	: Switch ADC channel, poll, get value
 * @Param	: ADC Channel
 * @retval	: 16bit ADC Value
 */
static uint16_t ADC_GetValueFromChannel(uint32_t c)
{
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
	HAL_ADC_Stop(&hadc1);
//	HAL_ADC_DeInit(&hadc1);
//	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//	  */
//	  hadc1.Instance = ADC1;
//	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//	  hadc1.Init.ScanConvMode = DISABLE;
//	  hadc1.Init.ContinuousConvMode = ENABLE;
//	  hadc1.Init.DiscontinuousConvMode = DISABLE;
//	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//	  hadc1.Init.NbrOfConversion = 1;
//	  hadc1.Init.DMAContinuousRequests = DISABLE;
//	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }

  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = c;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }


  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  return HAL_ADC_GetValue(&hadc1);

}

CAN_TxHeaderTypeDef	TxHeader;
CAN_RxHeaderTypeDef	RxHeader;

uint32_t TxMailBox;
uint8_t CANTx_Dat[8], CANRx_Dat[8];

#ifdef TEST_CAN
uint16_t joystickAdcValue = 0, joystickbalanceAdcValue = 0;
/*
 * @brief: Convert from CAN frame to CANopen Frame
 * @param: id: Node ID
 * @param: index: 2bytes index + 1byte sub index
 * @param: data: maximum 4bytes of transceiving data
 * @param: len: data lenght, maximum 4 bytes
 * @param: rw: 0 to read (x40), 1 to write (x20)
 * @reval: HAL_Status
 */
HAL_StatusTypeDef CANopenSendSDO(uint8_t id, uint32_t index, uint32_t data, uint8_t len, uint8_t rw)
{
	uint8_t txbuf[8];
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;

	TxHeader.RTR = CAN_RTR_DATA;//sending data frame
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.StdId = 0x600+id;
	txbuf[0] = 0x2F;
	if(!rw) txbuf[0] = 0x4F;
	if(len == 2) txbuf[0] &= 0xFB;
	else if(len == 4) txbuf[0] &= 0xF3;

	txbuf[1] = (index>>8)&0xff;
	txbuf[2] = (index>>16)&0xff;
	txbuf[3] = index&0xff;

	txbuf[4] = data & 0xff;
	txbuf[5] = (data>>8)&0xff;
	txbuf[6] = (data>>16)&0xff;
	txbuf[7] = (data>>24)&0xff;
	return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txbuf, &TxMailBox);;
}
#endif
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  /*
   * Initiate INA module
   */
  setCalibration_32V_2A();

  loader_stm32_config_t config = {
      .huart = &huart2,
      .port_io0 = GPIOD,
      .pin_num_io0 = GPIO_PIN_4,
      .port_rst = GPIOD,
      .pin_num_rst = GPIO_PIN_3,
  };

  loader_port_stm32_init(&config);

  uint16_t v = 0;
  uint8_t TxBuf[17] = "$             #\n\r",\
		  substr[11];

  OFF_POWER;

#ifdef TEST_CAN
  //Test CAN
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
HAL_CAN_Start(&hcan1);

////Boot up:
TxHeader.DLC = 8;
TxHeader.IDE = CAN_ID_STD;
TxHeader.StdId = 601;
TxHeader.RTR = CAN_RTR_DATA;//sending data frame
TxHeader.TransmitGlobalTime = DISABLE;
//CANTx_Dat[0] = 1;
//CANTx_Dat[1] = 1;
//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CANTx_Dat, &TxMailBox);
//
TxHeader.DLC = 8;
TxHeader.IDE = CAN_ID_STD;
TxHeader.RTR = CAN_RTR_DATA;//sending data frame
//TxHeader.StdId = 0x601;
//
//CANTx_Dat[0] = 0x2B;
//CANTx_Dat[1] = 0x40;
//CANTx_Dat[2] = 0x60;
//CANTx_Dat[3] = 0x00;
//CANTx_Dat[4] = 0x2F;
//CANTx_Dat[5] = 0x00;
//CANTx_Dat[6] = 0x00;
//CANTx_Dat[7] = 0x00;
//
//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CANTx_Dat, &TxMailBox);

//Enable Motor;
CANopenSendSDO(1, 0x604000, 0x2F, 2, 1);

//Set operation mode to Speed mode
CANopenSendSDO(1, 0x606000, 3, 1, 1);
joystickbalanceAdcValue = ADC_GetValueFromChannel(ADC_CHANNEL_8);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  CANopenSendSDO(1, 0x604000, 0x06, 2, 1);
#ifdef TEST_CAN
	  joystickAdcValue = ADC_GetValueFromChannel(ADC_CHANNEL_8);

	  if(joystickAdcValue > joystickbalanceAdcValue)
	  {
		  //Rotate Motor CW
		  CANopenSendSDO(1, 0x607E00, 0, 1, 1);
		  //Set target speed
		  uint16_t speed = joystickAdcValue - joystickbalanceAdcValue;
		  if(speed > 2000) speed = 2000;
		  if(speed < 100) speed = 0;
		  CANopenSendSDO(1, 0x60FF00, speed*2000, 4, 1);
	  }
	  else
	  {
		  //Rotate Motor CCW
		  CANopenSendSDO(1, 0x607E00, 1, 1, 1);
		  //Set target speed
		  uint16_t speed = joystickbalanceAdcValue - joystickAdcValue;
		  if(speed > 2000) speed = 2000;
		  if(speed < 100) speed = 0;
		  CANopenSendSDO(1, 0x60FF00, speed*2000, 4, 1);
	  }
#endif
//


	  switch(SMV_Step)
	  {
	  case IDLE:
		  break;
	  case POWER_ON:
		  ON_POWER;
		  HAL_Delay(200);
		  CDC_Transmit_FS((uint8_t*) "$PG           #\n\r", 17);
		  break;
	  case POWER_OFF:
		  OFF_POWER;
		  CDC_Transmit_FS((uint8_t*) "$PG           #\n\r", 17);
		  break;
	  case CHECK_CURRENT:

		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET)
		  {
			  ON_POWER;
			  HAL_Delay(1000);
		  }

		  v = (uint16_t) getCurrent_mA();
		  //v = (uint16_t) lround((double)getCurrent_mA());
//		  v= getBusVoltage_V();
//		  v = 1234;//Test

		  if(v > 2000)//Overload:
		  {
			  //Turn off relay
			  OFF_POWER;
		  }
		  sprintf(substr, "%4u       ", v);
		  omemcpy(substr, TxBuf, 2, 11);
		  TxBuf[0] = '$';
		  TxBuf[1] = 'C';
		  CDC_Transmit_FS( TxBuf, 17);
		  SMV_Step = IDLE;
		  break;

	  case CHECK_5V:
		  v = ADC_GetValueFromChannel(ADC_CHANNEL_0);//4096 max
		  sprintf(substr, "%4u       ", v);
		  omemcpy(substr, TxBuf, 2, 11);
		  TxBuf[1] = 'A';
		  CDC_Transmit_FS( TxBuf, 17);
		  SMV_Step = IDLE;
		  break;
	  case CHECK_4V2:
		  v = ADC_GetValueFromChannel(ADC_CHANNEL_1);//4096 max
		  sprintf(substr, "%4u       ", v);
		  omemcpy(substr, TxBuf, 2, 11);
		  TxBuf[1] = 'B';
		  CDC_Transmit_FS( TxBuf, 17);
		  SMV_Step = IDLE;
		  break;
	  case CHECK_3V3:
		  v = ADC_GetValueFromChannel(ADC_CHANNEL_2);//4096 max
		  sprintf(substr, "%4u       ", v);
		  omemcpy(substr, TxBuf, 2, 11);
		  TxBuf[1] = 'D';
		  CDC_Transmit_FS( TxBuf, 17);

		  SMV_Step = IDLE;
		  break;
	  case CONNECT_ESP:
		  //Hold ESP32's GPIO0 to ground then reset
		  OFF_POWER;
		  HOLD_ESP_IO0;
		  HAL_Delay(1000);
		  ON_POWER;
		  HAL_Delay(1000);
		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
			  target_chip_t target = esp_loader_get_target();
			  if(target == ESP32_CHIP)
			  {
				  CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
			  }
			  else CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
		  }
		  else CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
		  SMV_Step = IDLE;
		  break;

	  case RESET_ESP:
		  //Hold ESP32's GPIO0 to ground then reset
		  RELEASE_ESP_IO0;
		  OFF_POWER;
		  HAL_Delay(500);
		  ON_POWER;
		  HAL_Delay(1000);
		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
			  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
		  }
		  else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
		  SMV_Step = IDLE;
		  break;

	  case START_FLASH_BOOTLOADER:
		  break;
	  case CHECK_RELAY:

		  break;

#ifdef _DEBUG_
		  CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
#else

		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
			  target_chip_t target = esp_loader_get_target();
			  if(target == ESP32_CHIP)
			  {
				  if(ESP_FLASH_Start(BOOTLOADER_ADDRESS_V0, binary_len) != ESP_LOADER_SUCCESS)
				  {
					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
				  }
//				  if(flash_binary(BlinkESP_ino_bootloader_bin, BlinkESP_ino_bootloader_bin_size , BOOTLOADER_ADDRESS_V0) != ESP_LOADER_SUCCESS)
//				  {
//					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
//				  }else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
//				  if(flash_binary(BlinkESP_ino_partitions_bin, BlinkESP_ino_partitions_bin_size , PARTITION_ADDRESS) != ESP_LOADER_SUCCESS)
//				  {
//					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
//				  }else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
//				  if(flash_binary(BlinkESP_ino_bin, BlinkESP_ino_bin_size , APPLICATION_ADDRESS) != ESP_LOADER_SUCCESS)
//				  {
//					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
//				  }
				  else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
			  }
		  }
		  else CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
#endif
		  SMV_Step = IDLE;
		  break;

	  case START_FLASH_PARTITION:
		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
			  target_chip_t target = esp_loader_get_target();
			  if(target == ESP32_CHIP)
			  {
				  if(ESP_FLASH_Start(PARTITION_ADDRESS, binary_len) != ESP_LOADER_SUCCESS)
//				  if(flash_binary(Partition, Partition_size, PARTITION_ADDRESS) != ESP_LOADER_SUCCESS)
				  {
					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
				  }
				  else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
			  }
		  }
		  else CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
		  SMV_Step = IDLE;
		  break;
	  case START_FLASH_APP:
		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
			  target_chip_t target = esp_loader_get_target();
			  if(target == ESP32_CHIP)
			  {
				  if(ESP_FLASH_Start(APPLICATION_ADDRESS, binary_len) != ESP_LOADER_SUCCESS)
				  //if(flash_binary(Application, Application_size, APPLICATION_ADDRESS) != ESP_LOADER_SUCCESS)
				  {
					  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
				  }
				  else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
			  }
		  }
		  else CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
		  SMV_Step = IDLE;
		  break;
	  case RECEIVE_BIN:
		  if(fESPFlashProcess)
		  {
			  esp_loader_error_t err;
			  err = esp_loader_flash_write(uBuf, current_len);
			  if (err != ESP_LOADER_SUCCESS) {
				  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
				  SMV_Step = IDLE;
			  }
			  else
			  {
				  written_len += current_len;
				  fESPFlashProcess = 0;
			  }

//			  HAL_UART_Receive(&huart2, pData, Size, Timeout)
//			  CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);
//			  if(ESP_FLASH_WriteBlock(uBuf, flash_block_len) != ESP_LOADER_SUCCESS)
//			  {
//				  CDC_Transmit_FS((uint8_t*) "$FE           #\n\r", 17);
//				  SMV_Step = IDLE;
//			  }
//			  else CDC_Transmit_FS((uint8_t*) "$FG           #\n\r", 17);

		  }
		  break;

//	  case FLASH_ESP:
//		  //Test uBuf:
//		  //printf(uBuf);
//		  for(uint16_t i = 0; i < 2000; i++)
//		  {
//			  debug_buf[i] = uBuf[i];
//		  }
//
//		  if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
//			  target_chip_t target = esp_loader_get_target();
//			  if(target == ESP32_CHIP)
//			  {
//
//				  flash_binary(uBuf, bin_len, BOOTLOADER_ADDRESS_V0);
//			  }
//		  }
//		  else SMV_Step = IDLE;
//		  	  free(uBuf);
//		  break;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	hcan1.Instance = CAN1;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
//CAN_FilterTypeDef CAN_Filter;
//CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
//CAN_Filter.FilterBank = 18;
//CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//CAN_Filter.FilterIdHigh = 0x581 << 5;
//CAN_Filter.FilterIdLow = 0;
//CAN_Filter.FilterMaskIdHigh = 0xFFFF << 5;
//CAN_Filter.FilterMaskIdLow = 0xFFFF;
//CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
//CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
//CAN_Filter.SlaveStartFilterBank = 20;
//
//HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 120;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 115200;
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CANRx_Dat);
}

#ifdef _DEBUG_
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for(i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}
#endif

esp_loader_error_t ESP_FLASH_Start(uint32_t address, uint32_t size)
{
	esp_loader_error_t err = ESP_LOADER_SUCCESS;

	printf("Erasing flash (this may take a while)...\n");
	err = esp_loader_flash_start(address, size, 1024);
	if (err != ESP_LOADER_SUCCESS) {
	    printf("Erasing flash failed with error %d.\n", err);
	    return err;
	}
	printf("Start programming\n");

	return err;
}

esp_loader_error_t ESP_FLASH_WriteBlock(uint8_t* payload, uint32_t size)
{
	esp_loader_error_t err = ESP_LOADER_SUCCESS;
    err = esp_loader_flash_write(payload, size);
    if (err != ESP_LOADER_SUCCESS) {
        printf("\nPacket could not be written! Error %d.\n", err);
        return err;
    }
    return err;
};


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
