#ifndef _dma_uart_h
#define _dma_uart_h

#include "cmsis_os.h"

void Init_DMA_Semaphore(void);
void DMA_Uart_Send(uint8_t *data, uint16_t size);
void DMA_Semaphore_Release(void);

#endif
