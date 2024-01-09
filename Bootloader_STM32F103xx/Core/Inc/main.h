/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Boot_VidJumpUserCode();
void Boot_VidReadData();

/*********************************** Bootloader Commands Prototypes ***************************************/
void bootloader_handel_getver_cmd(uint8_t *pBuffer);

void bootloader_handel_gethelp_cmd(uint8_t *pBuffer);

void bootloader_handel_getcid_cmd(uint8_t *pBuffer);
uint16_t Chip_u16GetID(void);

void bootloader_handel_getrdp_cmd(uint8_t *pBuffer);
uint16_t Mem_u16GetRDP(void);

void bootloader_handel_goto_cmd(uint8_t *pBuffer);
uint8_t MEM_u8VerifyAdd(uint32_t Cpy_u32MemAdd);

void bootloader_handel_flash_erase_cmd(uint8_t *pBuffer);
uint8_t Mem_u8FlashErase(uint8_t Cpy_u8InitPage, uint8_t Cpy_u8NumOfPages);

void bootloader_handel_mem_write_cmd(uint8_t *pBuffer);
uint8_t Mem_u8FlashWrite(uint8_t *dBuffer, uint32_t Cpy_u32MemAddr,uint32_t Cpy_u32Len);

void bootloader_handel_en_rw_protect_cmd(uint8_t *pBuffer);

void bootloader_handel_mem_read_cmd(uint8_t *pBuffer);

void bootloader_handel_read_sector_cmd(uint8_t *pBuffer);

void bootloader_handel_otp_read_cmd(uint8_t *pBuffer);

void bootloader_handel_dis_rw_protect_cmd(uint8_t *pBuffer);

/*********************************** ACK/NACK Prototypes ************************************************/
void bootloader_send_ack(uint8_t Cpy_u8CommandCode, uint8_t Cpy_u8FollowLen);
void bootloader_send_nack();

/************************************ CRC Prototypes ****************************************************/
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t Cpy_u8Len, uint32_t crc_host);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Boot_Button_Pin GPIO_PIN_15
#define Boot_Button_GPIO_Port GPIOB
#define APP_FLASH_PAGE_BASE_ADDRESS 0x08009000

/*BOOTLOADER CODE COMMANDS*/
#define BL_GET_VER				0x51
#define BL_GET_HELP				0x52
#define BL_GET_CID				0x53
#define BL_GET_RDP_STATUS		0x54
#define BL_GO_TO_ADDR			0x55
#define BL_FLASH_ERASE			0x56
#define BL_MEM_WRITE			0x57
#define BL_EN_R_W_PROTECT		0x58
#define BL_MEM_READ				0x59
#define BL_READ_SECTOR_STATUS	0x5A
#define BL_OTP_READ				0x5B
#define BL_DIS_R_W_PROTECT		0x5C
#define BL_COMM_CODE_SIZE		12
/*ACK*/
#define BL_ACK	0XA5
#define BL_NACK	0x7F
/*CRC*/
#define VERIFY_CRC_FAIL		1
#define VERIFY_CRC_SUCCESS	0

#define BL_VERSION		0x10

#define ADDR_VALID		0x00
#define ADDR_INVALID	0x01

#define FLASH_SIZE		(64*1024)
#define SRAM_SIZE		(20*1024)
#define	FLASH_END		((FLASH_BASE) + (FLASH_SIZE))
#define SRAM_END		((SRAM_BASE) + (SRAM_SIZE))

#define INVALID_PAGE	0x01

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
