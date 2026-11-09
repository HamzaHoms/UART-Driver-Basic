/*
 * uart_wrapper_stream.h
 *
 *  Created on: Oct 21, 2026
 *      Author: Hamza
 */

#ifndef INC_UART_WRAPPER_STREAM_H_
#define INC_UART_WRAPPER_STREAM_H_

#include "buffers.h"
#include "stm32f4xx.h"

#define UART_RX_DR_THRSHLD 0.7

#define UART_RX_FIFO_SIZE 512
#define UART_TX_FIFO_SIZE 512

#define UART_RX_DMA_BUF_SIZE 256
#define UART_TX_DMA_BUF_SIZE 256


typedef enum{
  UART_CLEAR = 0,
  UART_BUSY = 1,
  UART_TX_PAUSE = 2,
  UART_ERROR = 3,
  UART_DESYNC = 4
}UART_State;

typedef struct{
  UART_HandleTypeDef* _huart;
  UART_State _tx_sts;
  UART_State _rx_sts;
  void (*_DataReady_Callback)(void);
  void (*_TxCplt_Callback)(void);

  FIFO_t* _rx_fifo;

  FIFO_t* _tx_fifo;

  uint8_t _tx_buf_dma[UART_TX_DMA_BUF_SIZE];
  uint8_t _rx_buf_dma[UART_RX_DMA_BUF_SIZE];
  uint32_t _rx_dma_p;

} wUART;

void wUART_Init(wUART* wuart, UART_HandleTypeDef* huart);
void wUART_SetDRCallback(wUART* wuart, void (*DataReady_Callback)(void));
void wUART_SetTXCpltCallback(wUART* wuart,   void (*TxCplt_Callback)(void));
USART_TypeDef* wUART_GetUSARTInstance(wUART* wuart);

HAL_StatusTypeDef wUART_WriteTX(wUART* wuart, uint8_t* buf, uint32_t len);
HAL_StatusTypeDef wUART_OpenRXDMA(wUART* wuart);
HAL_StatusTypeDef wUART_CloseRXDMA(wUART* wuart);

uint32_t wUART_ReadRXFIFO(wUART* wuart, uint8_t* buf, uint32_t len);

void wUART_RX_FCISR_Impl(wUART* wuart);
void wUART_RX_HCISR_Impl(wUART* wuart);
void wUART_TX_FCISR_Impl(wUART* wuart);

#endif /* INC_UART_WRAPPER_STREAM_H_ */
