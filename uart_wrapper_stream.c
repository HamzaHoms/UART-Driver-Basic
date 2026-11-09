/*
 * uart_wrapper_stream.c
 *
 *  Created on: Oct 21, 2026
 *      Author: Hamza
 */

#include "uart_wrapper_stream.h"

void wUART_Init(wUART* wuart, UART_HandleTypeDef* huart)
{
  wuart->_huart = huart;
  wuart->_tx_sts = UART_CLEAR;
  wuart->_rx_sts = UART_CLEAR;

  FIFO_INIT(rx_fifo, UART_RX_FIFO_SIZE);
  FIFO_INIT(tx_fifo, UART_TX_FIFO_SIZE);

  wuart->_rx_fifo = &rx_fifo;

  wuart->_tx_fifo = &tx_fifo;
}

void wUART_SetDRCallback(wUART* wuart, void (*DataReady_Callback)(void))
{
  wuart->_DataReady_Callback = DataReady_Callback;
}

void wUART_SetTXCpltCallback(wUART* wuart,   void (*TxCplt_Callback)(void))
{
  wuart->_TxCplt_Callback = TxCplt_Callback;
}

USART_TypeDef* wUART_GetUSARTInstance(wUART* wuart)
{
  return wuart->_huart->Instance;
}

HAL_StatusTypeDef wUART_WriteTX(wUART* wuart, uint8_t* buf, uint32_t len)
{
  if(len > UART_TX_DMA_BUF_SIZE) return HAL_ERROR;

    if(wuart->_tx_sts != UART_CLEAR)
      {
        BUFFER_Status sts = FIFO_Append(wuart->_tx_fifo, buf, len);
        if(sts == BUF_ERROR) return HAL_ERROR;

        return HAL_OK;
      }

    wuart->_tx_sts = UART_BUSY;

    memcpy(wuart->_tx_buf_dma, buf, len);

    return HAL_UART_Transmit_DMA(wuart->_huart, wuart->_tx_buf_dma, len);
}

HAL_StatusTypeDef wUART_OpenRXDMA(wUART* wuart)
{
  FIFO_Clear(wuart->_rx_fifo);
  HAL_StatusTypeDef sts = HAL_UART_Receive_DMA(wuart->_huart, wuart->_rx_buf_dma, UART_RX_DMA_BUF_SIZE);

  if(sts == HAL_OK)
    {
      wuart->_rx_sts = UART_BUSY;
      return HAL_OK;
    }

  wuart->_rx_sts = UART_ERROR;
  return HAL_ERROR;
}

HAL_StatusTypeDef wUART_CloseRXDMA(wUART* wuart)
{
  wuart->_rx_sts = UART_CLEAR;
  return HAL_UART_DMAStop(wuart->_huart);
}

uint32_t wUART_ReadRXFIFO(wUART* wuart, uint8_t* buf, uint32_t len)
{
  return FIFO_Pop(wuart->_rx_fifo, buf, len);
}

void wUART_RX_FCISR_Impl(wUART* wuart)
{
  uint32_t loc_dma_p = wuart->_rx_dma_p;
  wuart->_rx_dma_p = 0;

  uint32_t dma_len = UART_RX_DMA_BUF_SIZE - loc_dma_p;

  if(FIFO_GetEmpty(wuart->_rx_fifo) < dma_len) FIFO_Discard(wuart->_rx_fifo,  dma_len);

  FIFO_Append(wuart->_rx_fifo, wuart->_rx_buf_dma, dma_len);

  if(FIFO_GetCount(wuart->_rx_fifo) > (UART_RX_FIFO_SIZE * UART_RX_DR_THRSHLD)\
      && wuart->_DataReady_Callback != NULL) wuart->_DataReady_Callback();
}

void wUART_RX_HCISR_Impl(wUART* wuart)
{
  uint32_t loc_dma_p = wuart->_rx_dma_p;
  wuart->_rx_dma_p = UART_RX_DMA_BUF_SIZE/2;

  uint32_t dma_len = UART_RX_DMA_BUF_SIZE/2 - loc_dma_p;

  if(FIFO_GetEmpty(wuart->_rx_fifo) < dma_len) FIFO_Discard(wuart->_rx_fifo,  dma_len);

  FIFO_Append(wuart->_rx_fifo, wuart->_rx_buf_dma, dma_len);

  if(FIFO_GetCount(wuart->_rx_fifo) > (UART_RX_FIFO_SIZE * UART_RX_DR_THRSHLD)\
      && wuart->_DataReady_Callback != NULL) wuart->_DataReady_Callback();
}

void wUART_TX_FCISR_Impl(wUART* wuart)
{
  if(FIFO_GetCount(wuart->_tx_fifo) == 0)
      {
      wuart->_tx_sts = UART_CLEAR;
        if(wuart->_TxCplt_Callback != NULL) wuart->_TxCplt_Callback();
        return;
      }

  uint32_t pop_size = FIFO_Pop(wuart->_tx_fifo, wuart->_tx_buf_dma, UART_TX_DMA_BUF_SIZE);
  HAL_UART_Transmit_DMA(wuart->_huart, wuart->_tx_buf_dma, pop_size);
}
