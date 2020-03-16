/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"



#define uint8_t unsigned char 
#define uint16_t unsigned short 
#define uint32_t unsigned int   
#define int32_t int 
#define int16_t short 
#define int8_t char 
/* USER CODE BEGIN Includes */

#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99
#define ENTER_QPI_MODE_CMD                   0x38
#define EXIT_QPI_MODE_CMD                    0xFF
/* 识别操作 */
#define READ_ID_CMD                          0x90
#define DUAL_READ_ID_CMD                     0x92
#define QUAD_READ_ID_CMD                     0x94
#define READ_JEDEC_ID_CMD                    0x9F
/* 读操作 */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_INOUT_FAST_READ_CMD             0xEB
/* 写操作 */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04
/* 寄存器操作 */
#define READ_STATUS_REG1_CMD                  0x05
#define READ_STATUS_REG2_CMD                  0x35
#define READ_STATUS_REG3_CMD                  0x15
#define WRITE_STATUS_REG1_CMD                 0x01
#define WRITE_STATUS_REG2_CMD                 0x31
#define WRITE_STATUS_REG3_CMD                 0x11
/* 编程操作 */
#define PAGE_PROG_CMD                        0x02
#define QUAD_INPUT_PAGE_PROG_CMD             0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12
/* 擦除操作 */
#define SECTOR_ERASE_CMD                     0x20
#define CHIP_ERASE_CMD                       0xC7
#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75


#define QSPI_FLASH                         QUADSPI
#define QSPI_FLASH_CLK_ENABLE()            __QSPI_CLK_ENABLE()
#define QSPI_FLASH_CLK_PIN                 GPIO_PIN_10
#define QSPI_FLASH_CLK_GPIO_PORT           GPIOB
#define QSPI_FLASH_CLK_GPIO_ENABLE()       __GPIOB_CLK_ENABLE()
#define QSPI_FLASH_CLK_GPIO_AF             GPIO_AF10_QUADSPI
#define QSPI_FLASH_BK1_IO0_PIN             GPIO_PIN_1
#define QSPI_FLASH_BK1_IO0_PORT            GPIOB
#define QSPI_FLASH_BK1_IO0_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
#define QSPI_FLASH_BK1_IO0_AF              GPIO_AF10_QUADSPI
#define QSPI_FLASH_BK1_IO1_PIN             GPIO_PIN_0
#define QSPI_FLASH_BK1_IO1_PORT            GPIOB
#define QSPI_FLASH_BK1_IO1_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
#define QSPI_FLASH_BK1_IO1_AF              GPIO_AF10_QUADSPI

// 两线，IO2,IO3不需要
#if 0

#define QSPI_FLASH_BK1_IO2_PIN             GPIO_PIN_7
#define QSPI_FLASH_BK1_IO2_PORT            GPIOF
#define QSPI_FLASH_BK1_IO2_CLK_ENABLE()    __GPIOF_CLK_ENABLE()
#define QSPI_FLASH_BK1_IO2_AF              GPIO_AF9_QUADSPI
#define QSPI_FLASH_BK1_IO3_PIN             GPIO_PIN_6
#define QSPI_FLASH_BK1_IO3_PORT            GPIOF
#define QSPI_FLASH_BK1_IO3_CLK_ENABLE()    __GPIOF_CLK_ENABLE()
#define QSPI_FLASH_BK1_IO3_AF              GPIO_AF9_QUADSPI
#endif

#define QSPI_FLASH_CS_PIN                 GPIO_PIN_11
#define QSPI_FLASH_CS_GPIO_PORT           GPIOB
#define QSPI_FLASH_CS_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define QSPI_FLASH_CS_GPIO_AF             GPIO_AF10_QUADSPI







/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */


void QSPI_FLASH_Init(void);
static uint8_t QSPI_WriteEnable();
uint8_t BSP_QSPI_Erase_Block(uint32_t BlockAddress);
static uint8_t QSPI_AutoPollingMemReady(uint32_t Timeout);
uint32_t QSPI_FLASH_ReadID(void);
uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
uint8_t BSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);


void QSPI_MAIN(void);


/* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)


#define W25Q128FV_PAGE_SIZE  0x100

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
