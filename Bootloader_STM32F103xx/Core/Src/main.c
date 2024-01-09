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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN
#define D_UART	&huart3
#define C_UART	&huart2
#define RX_BUFFER_SIZE	200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void printmsg(char *format,...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Glob_u8BLRxBuffer[RX_BUFFER_SIZE];
uint8_t Glob_u8BLCommCode[] = {
								BL_GET_VER,
								BL_GET_HELP,
								BL_GET_CID,
								BL_GET_RDP_STATUS,
								BL_GO_TO_ADDR,
								BL_FLASH_ERASE,
								BL_MEM_WRITE,
								BL_EN_R_W_PROTECT,
								BL_MEM_READ,
								BL_READ_SECTOR_STATUS,
};

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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  printmsg("DMSG: Welcome to Custom BootLoader...\n\r");
  HAL_Delay(2000);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(0 == HAL_GPIO_ReadPin(Boot_Button_GPIO_Port, Boot_Button_Pin))
	  {
		  printmsg("DMSG: Boot Button Is Pressed .. Going to BL Mood\n\r");
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_Delay(2000);
		  Boot_VidReadData();

	  }
	  else
	  {
		  printmsg("DMSG: Boot Button Is Not Pressed .. Going to User Application\n\r");
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_Delay(2000);
		  Boot_VidJumpUserCode();

	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Boot_Button_Pin */
  GPIO_InitStruct.Pin = Boot_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Boot_Button_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

void Boot_VidJumpUserCode()
{
/*Pointer to function which will hold the address of the reset handler of the user app*/
	void (*Loc_PAppRstHandler)(void);
	printmsg("DBMSG: Bootloader will jumb to user app\n\r");
	HAL_Delay(1000);
/*
 * Configure the M stack pointer by reading the value form the base address
 * of user application flash page.
 * Note: The first byte is MSP Value and Second byte is Reset Handle address value
 * */
	uint32_t MSP = (*(volatile uint32_t*)APP_FLASH_PAGE_BASE_ADDRESS);
	printmsg("DBMSG: MSP Address = %#x\n\r",MSP);
/*
 * API __set_MSP change the MSP value which access the MSP register and install it with the MSP
 * Value of the user application
 * */
	__set_MSP(MSP);
	// Store the VTOR = APP_FLASH_PAGE_BASE_ADDRESS in startup code

	/*
	 * Now fetch the new reset handler address of the user application
	 * from the location APP_FLASH_PAGE_BASE_ADDRESS + 4
	 * */
	uint32_t ResetHandler_Address = (*(volatile uint32_t*)(APP_FLASH_PAGE_BASE_ADDRESS+4));
	Loc_PAppRstHandler = (void*)ResetHandler_Address;
	printmsg("DBMSG: App Reset Handler Address = %#x\n\r",Loc_PAppRstHandler);
	/*Jumping to Application reset handler address*/
	Loc_PAppRstHandler();



}
void Boot_VidReadData()
{

	uint8_t Loc_u8RxLen = 0;
	while(1)
	{
		memset(Glob_u8BLRxBuffer,0,200);
		HAL_UART_Receive(C_UART, Glob_u8BLRxBuffer, 1, HAL_MAX_DELAY);
		Loc_u8RxLen = Glob_u8BLRxBuffer[0];
		HAL_UART_Receive(C_UART, &Glob_u8BLRxBuffer[1], Loc_u8RxLen, HAL_MAX_DELAY);
		switch(Glob_u8BLRxBuffer[1])
		{
			case BL_GET_VER:
				bootloader_handel_getver_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_GET_HELP:
				bootloader_handel_gethelp_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_GET_CID:
				bootloader_handel_getcid_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handel_getrdp_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_GO_TO_ADDR:
				bootloader_handel_goto_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_FLASH_ERASE:
				bootloader_handel_flash_erase_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_MEM_WRITE:
				bootloader_handel_mem_write_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_EN_R_W_PROTECT:
				bootloader_handel_en_rw_protect_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_MEM_READ:
				bootloader_handel_mem_read_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handel_read_sector_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_OTP_READ:
				bootloader_handel_otp_read_cmd(Glob_u8BLRxBuffer);
				break;
			case BL_DIS_R_W_PROTECT:
				bootloader_handel_dis_rw_protect_cmd(Glob_u8BLRxBuffer);
				break;
			default:
				printmsg("DBMSG: Invalid command code recived from host\n");
		}

	}

}
void bootloader_handel_getver_cmd(uint8_t *pBuffer)
{
	uint8_t Loc_u8BLVersion;
	printmsg("DBMSG: Boot loader handle getver command:\n");

	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = (*(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4)) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],1);
		Loc_u8BLVersion = BL_VERSION;
		printmsg("DBMSG: BL_Version: %d %#x\n",Loc_u8BLVersion, Loc_u8BLVersion);
		HAL_UART_Transmit(C_UART, &Loc_u8BLVersion, 1, HAL_MAX_DELAY);
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}


}
void bootloader_handel_gethelp_cmd(uint8_t *pBuffer)
{
	printmsg("DBMSG: Boot loader handle gethelp command:\n");

	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],sizeof(Glob_u8BLCommCode));
		HAL_UART_Transmit(C_UART, Glob_u8BLCommCode,sizeof(Glob_u8BLCommCode) , HAL_MAX_DELAY);
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}

}
void bootloader_handel_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t Loc_u16ChipID = 0;
	printmsg("DBMSG: Boot loader handle getcid command:\n");

	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],2);
		Loc_u16ChipID = Chip_u16GetID();
		printmsg("DBMSG: Chip ID: %d %#x\n",Loc_u16ChipID, Loc_u16ChipID);
		//Loc_u8ChipID casted to u8 as uart sends 1 byte per time and the length of total data is 2 bytes
		HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u16ChipID, 2, HAL_MAX_DELAY);
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}

}
uint16_t Chip_u16GetID(void)
{
	uint16_t Loc_u16Cid;
	//Loc_u16Cid is reserved in the first 12bit of 32bit register
	// so the register is casted to u16 and masked for the first 12bit
	Loc_u16Cid =  (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return Loc_u16Cid;
}
void bootloader_handel_getrdp_cmd(uint8_t *pBuffer)
{
	uint16_t Loc_u16RDPValue = 0;
	printmsg("DBMSG: Boot loader handle getrdp command:\n");

	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],2);
		Loc_u16RDPValue = Mem_u16GetRDP();
		printmsg("DBMSG: RDP Value: %d %#x\n",Loc_u16RDPValue, Loc_u16RDPValue);
		//Loc_u16RDPValue casted to u8 as uart sends 1 byte per time and the length of total data is 2 bytes
		HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u16RDPValue, 2, HAL_MAX_DELAY);
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}
}
uint16_t Mem_u16GetRDP(void)
{
	uint16_t Loc_u16RDBStatus;
	volatile uint32_t *Ptr_u32RDBAdd = (uint32_t*)0x1FFFF800;
	Loc_u16RDBStatus =  (uint16_t)(*Ptr_u32RDBAdd);
	return Loc_u16RDBStatus;
}

void bootloader_handel_goto_cmd(uint8_t *pBuffer)
{
	uint32_t Loc_u32MemAdd = 0;
	uint8_t	Loc_u8ValidAdd = ADDR_VALID;
	uint8_t Loc_u8InvalidAdd = ADDR_INVALID;
	printmsg("DBMSG: Boot loader handle goto command:\n");
	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],1);
		Loc_u32MemAdd = (uint32_t)pBuffer[2];
		printmsg("DBMSG: Go to Address Value: %d %#x\n",Loc_u32MemAdd, Loc_u32MemAdd);
		if(MEM_u8VerifyAdd(Loc_u32MemAdd) == ADDR_VALID)
		{
			HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u8ValidAdd, 1, HAL_MAX_DELAY);
			//Must increment 1 to the address consideing the T bit
			Loc_u32MemAdd += 1;
			void (*Jump_To_ADD)(void) = (void*)Loc_u32MemAdd;
			printmsg("DBMSG: Jumping to Address..\n");
			Jump_To_ADD();
		}
		else
		{
			printmsg("DBMSG: Invalid Address..\n");
			HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u8InvalidAdd, 1, HAL_MAX_DELAY);
		}
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}

}
uint8_t MEM_u8VerifyAdd(uint32_t Cpy_u32MemAdd)
{
/*
 * We can Jump only accessible addresses not XN region such as:
 * 		-	System Memory
 * 		-	SRAM Memory
 * 		- 	External SRAM Memory
 * 		-	Backup Memory
 * 		-	Flash Memory
 * 		-	External Flash memory
 * 		-	etc..
 * in STM32F103C8T6 we only have Flash and SRAM Memories
 * */
	if(Cpy_u32MemAdd >= SRAM_BASE && Cpy_u32MemAdd <= SRAM_END)
	{
		return ADDR_VALID;
	}
	else if(Cpy_u32MemAdd >= FLASH_BASE && Cpy_u32MemAdd <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else
	{
		return ADDR_INVALID;
	}
}
void bootloader_handel_flash_erase_cmd(uint8_t *pBuffer)
{
	uint8_t Loc_u8EraseStatus = 0x00;
	printmsg("DBMSG: Boot loader handle flash erase command:\n");

	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],1);
		printmsg("DBMSG: Initial page: %d  Number of pages: %d\n",pBuffer[2], pBuffer[3]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		Loc_u8EraseStatus = Mem_u8FlashErase(pBuffer[2],pBuffer[3]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		//Loc_u16RDPValue casted to u8 as uart sends 1 byte per time and the length of total data is 2 bytes
		printmsg("DBMSG: Flash Erase status: %d",Loc_u8EraseStatus);
		HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u8EraseStatus, 1, HAL_MAX_DELAY);
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}

}
uint8_t Mem_u8FlashErase(uint8_t Cpy_u8PageNum, uint8_t Cpy_u8NumOfPages)
{
	HAL_StatusTypeDef Status;
	FLASH_EraseInitTypeDef FlashErase_Handel;
	uint32_t Loc_u32PageAddress;
	uint32_t Loc_u32PageError;
	/* There are 64 page in STM32F103C8T6 from page [0 to 63] */
	/* Number of Cpy_u8NumOfPages must be from 0 to 63 */
	/* if Cpy_u8InitPage == 0xff -> Means Mass Erase */
	if (Cpy_u8NumOfPages > 64)
	{
		return INVALID_PAGE; // ADD DEFIN
	}

	if ((Cpy_u8PageNum == 0xff) || ((Cpy_u8PageNum <= 63) && (Cpy_u8PageNum + Cpy_u8NumOfPages <=64 ) ))
	{
		if (Cpy_u8PageNum == (uint8_t)0xff)
		{
			FlashErase_Handel.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			Loc_u32PageAddress = FLASH_BASE + (Cpy_u8PageNum * (uint32_t)1024);
			//uint8_t Loc_u8RemainingPage = 64 - Cpy_u8NumOfPages;
			//if(Cpy_u8NumOfPages > Loc_u8RemainingPage)
			//{
				//Cpy_u8NumOfPages = Loc_u8RemainingPage;
			//}

			FlashErase_Handel.TypeErase = FLASH_TYPEERASE_PAGES;
			FlashErase_Handel.PageAddress = Loc_u32PageAddress;
			FlashErase_Handel.NbPages = Cpy_u8NumOfPages;
		}
		FlashErase_Handel.Banks = FLASH_BANK_1;
		HAL_FLASH_Unlock();
		Status = (uint8_t)HAL_FLASHEx_Erase(&FlashErase_Handel, &Loc_u32PageError);
		HAL_FLASH_Lock();
		return Status;
	}
	return INVALID_PAGE;
}
void bootloader_handel_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t Loc_u8WriteStatus = 0x00;
	uint8_t Loc_u8CheckSum = 0;
	uint8_t Loc_u8Len = 0;
	Loc_u8Len = pBuffer[0];
	uint8_t Loc_u8PayLoadLen = pBuffer[6];
	uint32_t Loc_u32MemAddress = (*(uint32_t*)(&pBuffer[2]));
	Loc_u8CheckSum = pBuffer[Loc_u8Len];
	printmsg("DBMSG: Boot loader handle Mem Write command:\n");
	//Length of the whole Command Backet
	uint32_t Loc_u32CommandBacketLen = pBuffer[0]+1;
	//Extract the CRC sent by the host
	// 4 is the length size of the CRC -> 4 BYTES / 32 bits
	uint32_t Loc_u32HostCRC = *(uint32_t*)(pBuffer + Loc_u32CommandBacketLen - 4) ;

	if(! bootloader_verify_crc(&pBuffer[0],Loc_u32CommandBacketLen-4 ,Loc_u32HostCRC))
	{
		printmsg("DBMSG: Checksum success..\n");
		bootloader_send_ack(pBuffer[0],1);
		printmsg("DBMSG: Memory write address %d...\n",Loc_u32MemAddress);
		if(MEM_u8VerifyAdd(Loc_u32MemAddress) == ADDR_VALID)
		{
			printmsg("DBMSG: Memory address valid...\n");
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			Loc_u8WriteStatus = Mem_u8FlashWrite(&pBuffer[7],Loc_u32MemAddress, Loc_u8PayLoadLen);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			printmsg("DBMSG: Flash Write status: %d",Loc_u8WriteStatus);
			HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u8WriteStatus, 1, HAL_MAX_DELAY);
			}
		else
		{
			printmsg("DBMSG: Invalid Address..\n");
			Loc_u8WriteStatus = ADDR_INVALID;
			HAL_UART_Transmit(C_UART, (uint8_t*)&Loc_u8WriteStatus, 1, HAL_MAX_DELAY);
		}
	}
	else
	{
		printmsg("DBMSG: Checksum fail..\n");
		bootloader_send_nack();
	}
}
uint8_t Mem_u8FlashWrite(uint8_t *dBuffer, uint32_t Cpy_u32MemAddr,uint32_t Cpy_u32Len)
{
	uint8_t Loc_u8Status =HAL_OK;
	HAL_FLASH_Unlock();
	for(uint32_t i = 0 ; i<Cpy_u32Len ; i++)
	{
		Loc_u8Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Cpy_u32MemAddr+i, dBuffer[i]);
	}
	HAL_FLASH_Lock();
	return Loc_u8Status;
}
void bootloader_handel_en_rw_protect_cmd(uint8_t *pBuffer)
{

}
void bootloader_handel_mem_read_cmd(uint8_t *pBuffer)
{

}
void bootloader_handel_read_sector_cmd(uint8_t *pBuffer)
{

}
void bootloader_handel_otp_read_cmd(uint8_t *pBuffer)
{

}
void bootloader_handel_dis_rw_protect_cmd(uint8_t *pBuffer)
{

}
/****************************************** CRC **********************************************/
void bootloader_send_ack(uint8_t Cpy_u8CommandCode, uint8_t Cpy_u8FollowLen)
{
	uint8_t Loc_u8AckBuff[2];
	Loc_u8AckBuff[0] = BL_ACK;
	Loc_u8AckBuff[1] = Cpy_u8FollowLen;
	HAL_UART_Transmit(C_UART, Loc_u8AckBuff, 2, HAL_MAX_DELAY);

}

void bootloader_send_nack()
{
	uint8_t Loc_u8NAck = BL_NACK;
	HAL_UART_Transmit(C_UART, &Loc_u8NAck, 1, HAL_MAX_DELAY);
}
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t Cpy_u8Len, uint32_t crc_host)
{
	uint32_t uwCRCValue= 0xff;
	for(uint32_t i = 0 ; i < Cpy_u8Len ; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	__HAL_CRC_DR_RESET(&hcrc);

	if(uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

		return VERIFY_CRC_FAIL;

}

/****************************************************************************************/


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
