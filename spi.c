/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
//#include "stm32l4xx_hal_qspi.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

QSPI_HandleTypeDef hqspi;
QSPI_HandleTypeDef QSPIHandle;
/* Definition for QSPI clock resources */
#define QSPI_CLK_ENABLE()          __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()         __HAL_RCC_QSPI_CLK_DISABLE()
#define QSPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_DMA_CLK_ENABLE()      __HAL_RCC_DMA1_CLK_ENABLE()

#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()

/* Definition for QSPI Pins */
#define QSPI_CS_PIN                GPIO_PIN_11
#define QSPI_CS_GPIO_PORT          GPIOE
#define QSPI_CLK_PIN               GPIO_PIN_10
#define QSPI_CLK_GPIO_PORT         GPIOE
#define QSPI_D0_PIN                GPIO_PIN_12
#define QSPI_D0_GPIO_PORT          GPIOE
#define QSPI_D1_PIN                GPIO_PIN_13
#define QSPI_D1_GPIO_PORT          GPIOE
#define QSPI_D2_PIN                GPIO_PIN_14
#define QSPI_D2_GPIO_PORT          GPIOE
#define QSPI_D3_PIN                GPIO_PIN_15
#define QSPI_D3_GPIO_PORT          GPIOE



/* N25Q128A13EF840E Micron memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      23
#define QSPI_PAGE_SIZE                       256

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_INOUT_FAST_READ_CMD             0xEB

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2
#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SECTOR_ERASE_CMD                     0xD8
#define BULK_ERASE_CMD                       0xC7
#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              8
#define DUMMY_CLOCK_CYCLES_READ_QUAD         10

/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)

/* Size of buffers */
#define BUFFERSIZE                 (COUNTOF(aTxBuffer) - 1)



/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PA1     ------> SPI1_SCK
    PA4     ------> SPI1_NSS
    PA6     ------> SPI1_MISO
    PA12     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI3 GPIO Configuration    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA1     ------> SPI1_SCK
    PA4     ------> SPI1_NSS
    PA6     ------> SPI1_MISO
    PA12     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_12);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();
  
    /**SPI3 GPIO Configuration    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}




void QSPI_MAIN(void)
{
 uint32_t FlashID=0x00;
 printf("qspi test.....\r\n");
  QSPI_FLASH_Init();
   FlashID = QSPI_FLASH_ReadID();

  printf("FlashID is :%x \r\n",FlashID);
   
}


/**
  * @}
  */

/** @addtogroup STM32L475E_IOT01_QSPI_Private_Functions 
  * @{
  */

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi : QSPI handle
  * @retval None
  */
static uint8_t QSPI_ResetMemory()
{
  QSPI_CommandTypeDef sCommand;

  /* Initialize the reset enable command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */  
  if (QSPI_AutoPollingMemReady( HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}


/**
* @brief  初始化QSPI存储器
* @retval QSPI存储器状态
*/
uint8_t BSP_QSPI_Init(void)
{
QSPI_CommandTypeDef s_command;
uint8_t value = 0x40;// W25Q128FV_FSR_QE;
	/* QSPI存储器复位 */
	if (QSPI_ResetMemory() != QSPI_OK) {
	return QSPI_NOT_SUPPORTED;
	}
	
	/* 使能写操作 */
	if (QSPI_WriteEnable() != QSPI_OK) {
	return QSPI_ERROR;
	}

#if 0	
	/* 设置四路使能的状态寄存器，使能四通道IO2和IO3引脚 */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = WRITE_STATUS_REG2_CMD;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	/* 配置命令 */
	if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
	return QSPI_ERROR;
	}
	/* 传输数据 */
	if (HAL_QSPI_Transmit(&QSPIHandle, &value, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
	return QSPI_ERROR;
	}
#endif	
	/* 自动轮询模式等待存储器就绪 */
	if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK) {
	printf("BSP_QSPI_Init : QSPI_AutoPollingMemReady error \r\n");
	return QSPI_ERROR;
	}
return QSPI_OK;
}

/**
* @brief  从QSPI存储器中读取大量数据.
* @param  pData: 指向要读取的数据的指针
* @param  ReadAddr: 读取起始地址
* @param  Size: 要读取的数据大小
* @retval QSPI存储器状态
*/
uint8_t BSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
QSPI_CommandTypeDef s_command;
/* 初始化读命令 */
s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
s_command.Instruction       = READ_CMD;
s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
s_command.Address           = ReadAddr;
s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
s_command.DataMode          = QSPI_DATA_1_LINE;
s_command.DummyCycles       = 0;
s_command.NbData            = Size;
s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
/* 配置命令 */
if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
return QSPI_ERROR;
}
/* 接收数据 */
if(HAL_QSPI_Receive(&QSPIHandle, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)!= HAL_OK) {
return QSPI_ERROR;
}
return QSPI_OK;
}

/**
* @brief  将大量数据写入QSPI存储器
* @param  pData: 指向要写入数据的指针
* @param  WriteAddr: 写起始地址
* @param  Size: 要写入的数据大小
* @retval QSPI存储器状态
*/
uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
QSPI_CommandTypeDef s_command;
uint32_t end_addr, current_size, current_addr;
/* 计算写入地址和页面末尾之间的大小 */
current_addr = 0;
while (current_addr <= WriteAddr) {
current_addr += W25Q128FV_PAGE_SIZE;
}
current_size = current_addr - WriteAddr;
/* 检查数据的大小是否小于页面中的剩余位置 */
if (current_size > Size) {
current_size = Size;
}
/* 初始化地址变量 */
current_addr = WriteAddr;
end_addr = WriteAddr + Size;
/* 初始化程序命令 */
s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
s_command.Instruction       = QUAD_INPUT_PAGE_PROG_CMD;
s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
s_command.DataMode          = QSPI_DATA_4_LINES;
s_command.DummyCycles       = 0;
s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
/* 逐页执行写入 */
do {
s_command.Address = current_addr;
s_command.NbData  = current_size;
/* 启用写操作 */
if (QSPI_WriteEnable() != QSPI_OK) {
return QSPI_ERROR;
}
/* 配置命令 */
if(HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
return QSPI_ERROR;
}
/* 传输数据 */
if(HAL_QSPI_Transmit(&QSPIHandle, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
return QSPI_ERROR;
}
/* 配置自动轮询模式等待程序结束 */
if(QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK) {
return QSPI_ERROR;
}
/* 更新下一页编程的地址和大小变量 */
current_addr += current_size;
pData += current_size;
current_size = ((current_addr + W25Q128FV_PAGE_SIZE) > end_addr) ? (end_addr-current_addr) :W25Q128FV_PAGE_SIZE;
} while (current_addr < end_addr);
return QSPI_OK;
} 

/**
* @brief  读取FLASH ID
* @param   无
* @retval FLASH ID
*/
uint32_t QSPI_FLASH_ReadID(void)
{
	QSPI_CommandTypeDef s_command;
	uint32_t Temp = 0;
	uint8_t pData[3]=0x00;
	/* 读取JEDEC ID */
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction       = READ_JEDEC_ID_CMD;
	s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
	s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DummyCycles       = 0;
	s_command.NbData            = 3;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	
	if(HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	printf("something wrong ....\r\n");
	/* 用户可以在这里添加一些代码来处理这个错误 */
	while (1) {
	}
	}
	if(HAL_QSPI_Receive(&QSPIHandle, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)!= HAL_OK) {
	printf("something wrong ....\r\n");
	/* 用户可以在这里添加一些代码来处理这个错误 */
	while (1) {
	}
	}
	printf("data1:%x \r\n",pData[0]);
	printf("data2:%x \r\n",pData[1]);
	printf("data3:%x \r\n",pData[2]);
	Temp = ( pData[2] | pData[1]<<8 )| ( pData[0]<<16 );
	printf("value is :%x \r\n",Temp);
	return Temp;
}

/**
* @brief  发送写入使能，等待它有效.
* @param  QSPIHandle: QSPI句柄
* @retval 无
*/
static uint8_t QSPI_WriteEnable()
{
QSPI_CommandTypeDef     s_command;
QSPI_AutoPollingTypeDef s_config;
/* 启用写操作 */
s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
s_command.Instruction       = WRITE_ENABLE_CMD;
s_command.AddressMode       = QSPI_ADDRESS_NONE;
s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
s_command.DataMode          = QSPI_DATA_NONE;
s_command.DummyCycles       = 0;
s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
if(HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
return QSPI_ERROR;
}
/* 配置自动轮询模式等待写启用 */
s_config.Match           = 0x02;//W25Q128FV_FSR_WREN;
s_config.Mask            = 0x02;//W25Q128FV_FSR_WREN;
s_config.MatchMode       = QSPI_MATCH_MODE_AND;
s_config.StatusBytesSize = 1;
s_config.Interval        = 0x10;
s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
s_command.Instruction    = READ_STATUS_REG1_CMD;
s_command.DataMode       = QSPI_DATA_1_LINE;
s_command.NbData         = 1;
if(HAL_QSPI_AutoPolling(&QSPIHandle, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
return QSPI_ERROR;
}
return QSPI_OK;
}
/**
* @brief  读取存储器的SR并等待EOP
* @param  QSPIHandle: QSPI句柄
* @param  Timeout 超时
* @retval 无
*/
static uint8_t QSPI_AutoPollingMemReady(uint32_t Timeout)
{
QSPI_CommandTypeDef     s_command;
QSPI_AutoPollingTypeDef s_config;
/* 配置自动轮询模式等待存储器准备就绪 */
s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
s_command.Instruction       = READ_STATUS_REG1_CMD;
s_command.AddressMode       = QSPI_ADDRESS_NONE;
s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
s_command.DataMode          = QSPI_DATA_1_LINE;
s_command.DummyCycles       = 0;
s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
s_config.Match           = 0x00;
s_config.Mask            = 0x01;////W25Q128FV_FSR_BUSY;
s_config.MatchMode       = QSPI_MATCH_MODE_AND;
s_config.StatusBytesSize = 1;
s_config.Interval        = 0x10;
s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
if (HAL_QSPI_AutoPolling(&QSPIHandle, &s_command, &s_config, Timeout) != HAL_OK) {
return QSPI_ERROR;
}
return QSPI_OK;
}

/**
* @brief  擦除QSPI存储器的指定块
* @param  BlockAddress: 需要擦除的块地址
* @retval QSPI存储器状态
*/
uint8_t BSP_QSPI_Erase_Block(uint32_t BlockAddress)
{
QSPI_CommandTypeDef s_command;
/* 初始化擦除命令 */
s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
s_command.Instruction       = SECTOR_ERASE_CMD;
s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
s_command.Address           = BlockAddress;
s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
s_command.DataMode          = QSPI_DATA_NONE;
s_command.DummyCycles       = 0;
s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
/* 启用写操作 */
if (QSPI_WriteEnable() != QSPI_OK) {
return QSPI_ERROR;
}
/* 发送命令 */
if(HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
return QSPI_ERROR;
}
/* 配置自动轮询模式等待擦除结束 */
if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK) {
return QSPI_ERROR;
}
return QSPI_OK;
}

void QSPI_FLASH_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* 使能 QSPI 及 GPIO 时钟 */
	QSPI_FLASH_CLK_ENABLE();
	QSPI_FLASH_CLK_GPIO_ENABLE();
	QSPI_FLASH_BK1_IO0_CLK_ENABLE();
	QSPI_FLASH_BK1_IO1_CLK_ENABLE();
	//QSPI_FLASH_BK1_IO2_CLK_ENABLE();
	//QSPI_FLASH_BK1_IO3_CLK_ENABLE();
	QSPI_FLASH_CS_GPIO_CLK_ENABLE();
	//设置引脚
	/*!< 配置 QSPI_FLASH 引脚: CLK */
	GPIO_InitStruct.Pin = QSPI_FLASH_CLK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = QSPI_FLASH_CLK_GPIO_AF;
	HAL_GPIO_Init(QSPI_FLASH_CLK_GPIO_PORT, &GPIO_InitStruct);
	/*!< 配置 QSPI_FLASH 引脚: IO0 */
	GPIO_InitStruct.Pin = QSPI_FLASH_BK1_IO0_PIN;
	GPIO_InitStruct.Alternate = QSPI_FLASH_BK1_IO0_AF;
	HAL_GPIO_Init(QSPI_FLASH_BK1_IO0_PORT, &GPIO_InitStruct);
	/*!< 配置 QSPI_FLASH 引脚: IO1 */
	GPIO_InitStruct.Pin = QSPI_FLASH_BK1_IO1_PIN;
	GPIO_InitStruct.Alternate = QSPI_FLASH_BK1_IO1_AF;
	HAL_GPIO_Init(QSPI_FLASH_BK1_IO1_PORT, &GPIO_InitStruct);

#if 0	
	/*!< 配置 QSPI_FLASH 引脚: IO2 */
	GPIO_InitStruct.Pin = QSPI_FLASH_BK1_IO2_PIN;
	GPIO_InitStruct.Alternate = QSPI_FLASH_BK1_IO2_AF;
	HAL_GPIO_Init(QSPI_FLASH_BK1_IO2_PORT, &GPIO_InitStruct);
	/*!< 配置 QSPI_FLASH 引脚: IO3 */
	GPIO_InitStruct.Pin = QSPI_FLASH_BK1_IO3_PIN;
	GPIO_InitStruct.Alternate = QSPI_FLASH_BK1_IO3_AF;
	HAL_GPIO_Init(QSPI_FLASH_BK1_IO3_PORT, &GPIO_InitStruct);
#endif	
	/*!< 配置 SPI_FLASH_SPI 引脚: NCS */
	GPIO_InitStruct.Pin = QSPI_FLASH_CS_PIN;
	GPIO_InitStruct.Alternate = QSPI_FLASH_CS_GPIO_AF;
	HAL_GPIO_Init(QSPI_FLASH_CS_GPIO_PORT, &GPIO_InitStruct);
	/* QSPI_FLASH 模式配置 */
	QSPIHandle.Instance = QUADSPI;
	QSPIHandle.Init.ClockPrescaler = 2;
	QSPIHandle.Init.FifoThreshold = 4;
	QSPIHandle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	QSPIHandle.Init.FlashSize = 23;
	QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
	QSPIHandle.Init.ClockMode = QSPI_CLOCK_MODE_0;
	HAL_QSPI_Init(&QSPIHandle);
	/*初始化QSPI接口*/
	BSP_QSPI_Init();
}










/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
