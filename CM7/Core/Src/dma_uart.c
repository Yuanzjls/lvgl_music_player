#include "dma_uart.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
static osSemaphoreId_t dma_uart_sem;
extern UART_HandleTypeDef huart1;
void Init_DMA_Semaphore(void)
{
    dma_uart_sem = osSemaphoreNew(1, 1, NULL); // Initial count 1
}

void DMA_Uart_Send(uint8_t *data, uint16_t size)
{
    osSemaphoreAcquire(dma_uart_sem, osWaitForever);
    HAL_UART_Transmit_DMA(&huart1, data, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        osSemaphoreRelease(dma_uart_sem);
    }
}

void DMA_Semaphore_Release(void)
{
    osSemaphoreRelease(dma_uart_sem);
}
