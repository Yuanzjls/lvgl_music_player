/**
 ******************************************************************************
 * @file    BSP/CM7/Src/sdram.c
 * @author  MCD Application Team
 * @brief   This example code shows how to use the SDRAM Driver
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sdram.h"
#include "stm32h747i_discovery_sdram.h"
#include "dma_uart.h"
/** @addtogroup STM32H7xx_HAL_Examples
 * @{
 */

/** @addtogroup BSP
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE ((uint32_t)0x0100)
#define WRITE_READ_ADDR ((uint32_t)0x1000)
#define SDRAM_WRITE_READ_ADDR 0xD0177000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t sdram_aTxBuffer[BUFFER_SIZE];
uint32_t sdram_aRxBuffer[BUFFER_SIZE];

/* DMA transfer complete flag */
/* Private function prototypes -----------------------------------------------*/

static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);
static uint8_t Buffercmp(uint32_t *pBuffer1, uint32_t *pBuffer2, uint16_t BufferLength);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  SDRAM Demo
 * @param  None
 * @retval None
 */
void SDRAM_demo(void)
{

    /* SDRAM device configuration */
    //    if (BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
    //    {
    //        DMA_Uart_Send((uint8_t *)"SDRAM Initialization : FAILED.\r\n", 32);
    //        DMA_Uart_Send((uint8_t *)"SDRAM Test Aborted.\r\n", 21);
    //    }
    //    else
    //    {
    //        DMA_Uart_Send((uint8_t *)"SDRAM Initialization : OK.\r\n", 28);
    //    }
    /* Fill the buffer to write */
    Fill_Buffer(sdram_aTxBuffer, BUFFER_SIZE, 0xA244250F);

    /* Write data to the SDRAM memory */
    if (HAL_SDRAM_Write_32b(&hsdram[0], (uint32_t *)(SDRAM_WRITE_READ_ADDR + WRITE_READ_ADDR), (uint32_t *)sdram_aTxBuffer, BUFFER_SIZE) != BSP_ERROR_NONE)
    {
        DMA_Uart_Send((uint8_t *)"SDRAM WRITE : FAILED.\r\n", 24);
        DMA_Uart_Send((uint8_t *)"SDRAM Test Aborted.\r\n", 22);
    }
    else
    {
        DMA_Uart_Send((uint8_t *)"SDRAM WRITE : OK.\r\n", 20);
    }

    /* Read back data from the SDRAM memory */
    if (HAL_SDRAM_Read_32b(&hsdram[0], (uint32_t *)(SDRAM_WRITE_READ_ADDR + WRITE_READ_ADDR), (uint32_t *)sdram_aRxBuffer, BUFFER_SIZE) != BSP_ERROR_NONE)
    {
        DMA_Uart_Send((uint8_t *)"SDRAM READ : FAILED.\r\n", 23);
        DMA_Uart_Send((uint8_t *)"SDRAM Test Aborted.\r\n", 22);
    }
    else
    {
        DMA_Uart_Send((uint8_t *)"SDRAM READ : OK.\r\n", 18);
    }

    if (Buffercmp(sdram_aTxBuffer, sdram_aRxBuffer, BUFFER_SIZE) > 0)
    {
        DMA_Uart_Send((uint8_t *)"SDRAM COMPARE : FAILED.\r\n", 25);
        DMA_Uart_Send((uint8_t *)"SDRAM Test Aborted.\r\n", 21);
    }
    else
    {
        DMA_Uart_Send((uint8_t *)"SDRAM Test : OK.\r\n", 18);
    }
}

/**
 * @brief  Fills buffer with user predefined data.
 * @param  pBuffer: pointer on the buffer to fill
 * @param  uwBufferLenght: size of the buffer to fill
 * @param  uwOffset: first value to fill on the buffer
 * @retval None
 */
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLength, uint32_t uwOffset)
{
    uint32_t tmpIndex = 0;

    /* Put in global buffer different values */
    for (tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++)
    {
        pBuffer[tmpIndex] = tmpIndex + uwOffset;
    }
    /* Clean Data Cache to update the content of the SDRAM */
    SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, uwBufferLength * 4);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param  BufferLength: buffer's length
 * @retval 1: pBuffer identical to pBuffer1
 *         0: pBuffer differs from pBuffer1
 */
static uint8_t Buffercmp(uint32_t *pBuffer1, uint32_t *pBuffer2, uint16_t BufferLength)
{
    /* Invalidate Data Cache to get the updated content of the SRAM*/
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)pBuffer2, BufferLength * 4);

    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return 1;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return 0;
}

/**
 * @}
 */

/**
 * @}
 */
