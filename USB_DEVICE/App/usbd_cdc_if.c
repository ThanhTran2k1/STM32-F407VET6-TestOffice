/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
//#include "stm32f407xx.h"

#include "stm32_port.h"
#include "esp_loader.h"
#include "example_common.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern  ADC_HandleTypeDef 	hadc1;
extern	UART_HandleTypeDef 	huart2;

uint8_t fESPFlashProcess = 0;
uint8_t fTransmitCpt = 0;
/* Declare State machine variable */
SM_TEST_Step SMV_Step = IDLE;
//Length of coming firmware:
uint32_t binary_len = 0;
uint8_t uBuf[1024];//[MAX_CDC_BUFFER_SIZE] = {};
uint8_t flash_block_len;

uint8_t CheckSimETH_Timeout;
uint8_t CheckETH_Timeout;

//Current length of uBuf, maximum 1024.
uint16_t current_len;
//Written length, when reach binary_len flash will be done.
uint32_t written_len;
//uint8_t uBuf[4000] ={};
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
//#define HIGHER_BAUDRATE 230400
//
//#define OFF_POWER	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
//#define ON_POWER	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define _DEBUG_

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

uint32_t time_=0;
#ifdef _DEBUG_
uint8_t dbuf[64] = {};
#endif

DIAI_Params_t 	AIDI_Param = {0};
SIM_POWER_t		SIM_Power = {0};
/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
//uint32_t hex2int(char *hex) {
uint8_t hex2int(char *hex){
    //uint32_t val = 0;
	uint8_t val = 0;
    //while (*hex) {
	for(uint8_t i = 0; i < 2; i++)
	{
        // get current character then increment
        uint8_t byte = *hex++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  if(Buf[0] == '$' && Buf[14] == '#' && *Len == 15)
  {
	  switch(Buf[1])
	  {
	  case 'P'://Power ON/OFF
	  	  if(Buf[2] == '0') SMV_Step = POWER_OFF;
	  	  else if(Buf[2] == '1') SMV_Step = POWER_ON;

	  break;
	  case 'C': //Measure Current
		  SMV_Step = CHECK_CURRENT;
		  break;
	  case 'A': //measuring 5 voltage at PA0 (ADC1_IN0)
		  SMV_Step = CHECK_5V;
		  break;
	  case 'B': //measuring 4v2 voltage at PA1 (ADC1_IN1)
		  SMV_Step = CHECK_4V2;
		  break;
	  case 'D': //measuring 3v3 voltage at PA2 (ADC1_IN2)
		  SMV_Step = CHECK_3V3;
		  break;
	  case 'F':///Flash
		  switch(Buf[2])
		  {
		  case 'C':
			  SMV_Step = CONNECT_ESP;
			  break;
		  case 'R':
			  SMV_Step = RESET_ESP;
			  break;
		  case 'B'://Start flash bootloader binary
			  SMV_Step = START_FLASH_BOOTLOADER;
			  break;
		  case 'P'://Start flash partition binary
			  SMV_Step = START_FLASH_PARTITION;
			  break;
		  case 'A'://Start flash application binary
			  SMV_Step = START_FLASH_APP;
			  break;
		  case 'F'://Started Received Binary and write
			  current_len = 0;
			  written_len = 0;
			  binary_len = 0;
			  for(uint8_t i = 3; i <= 13; i++)
			  {
				  if(Buf[i] == ' ') break;
				  binary_len *= 10;
				  binary_len += Buf[i] - 48;
			  }
			  CDC_Transmit_FS(Buf, Len);
			  SMV_Step = RECEIVE_BIN;
			  break;
		  case 'E':
			  free(uBuf);
			  if(Buf[3]==' '&&Buf[4]==' '&&Buf[5]==' '&&Buf[6]==' '&&Buf[7]==' ')
			  SMV_Step = IDLE;
			  break;
		  }
		  break;
	  case 'M':
		  SMV_Step = CHECK_RELAY;
		  break;
	  case '1':
		  //
		  break;
	  case '2'://Check Battery capacity, Temperature
		  SMV_Step = GET_BATTERY_CAPACITY;
		  break;
	  case '3'://Check AIDI:
		  AIDI_Param.DO_Freq[0] = hex2int(&Buf[2]);
		  AIDI_Param.DO_Freq[1] = hex2int(&Buf[4]);
		  AIDI_Param.DO_Freq[2] = hex2int(&Buf[6]);
		  AIDI_Param.DO_Freq[3] = hex2int(&Buf[8]);
		  AIDI_Param.AO_Level = hex2int(&Buf[10]);
		  AIDI_Param.Timeout = hex2int(&Buf[12]);
		  SMV_Step = CHECK_AIDI;
		  break;
	  case '4':
		  SIM_Power.ONOFF = Buf[2]-48;
		  SIM_Power.Timeout = hex2int(&Buf[3]);
		  SMV_Step = CHECK_1V8;
		  break;
	  case '5':
		  SMV_Step = CHECK_SIM_STATUS;
		  break;
	  case '6':
		  SMV_Step = CHECK_SIM_SIGNAL;
		  break;
	  case '7':
		  CheckSimETH_Timeout = hex2int(&Buf[2]);
	  	  SMV_Step = CHECK_SIM_ETH;
	  	  break;
	  case '8':
		  CheckETH_Timeout = hex2int(&Buf[2]);
		  SMV_Step = CHECK_ETH;
		  break;
	  case '9':
		  SMV_Step = CHECK_LED;
		  break;
	  }
  }
  else if(SMV_Step == RECEIVE_BIN)
  {
//	  if(HAL_UART_Transmit(&huart2, Buf, *Len, 1000)==HAL_OK)
//		  fESPFlashProcess = 1;
//Here per block we have 64bytes:
	  if(current_len + *Len <= 1024)
	  {
		  strcat(uBuf, Buf);
		  CDC_Transmit_FS((uint8_t*) "$PG           #\n\r", 17);
		  current_len += *Len;

	  }else fESPFlashProcess = 1;

//	  free(uBuf);
//	  flash_block_len = *Len;
//
//	  uBuf = (uint8_t*) malloc(flash_block_len);
//	  for(uint8_t i = 0; i < flash_block_len; i++)
//	  {
//		  uBuf[i] = Buf[i];
//#ifdef _DEBUG_
//		  dbuf[i] = Buf[i];
//#endif
//
//		  //*uBuf++;
//	  }
//	  time_+=1;
//	  fESPFlashProcess = 1;
  }
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  fTransmitCpt = 1;
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
