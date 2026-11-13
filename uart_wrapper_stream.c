
#include "uart_wrapper_stream.h"

void wUART_Init
(
wUART* wuart, //the target uart wrapper instance pointer
UART_HandleTypeDef* huart //pointer to the HAL uart handler to be used in 'wuart'
)
{
  wuart->_huart = huart;
  wuart->_tx_sts = UART_CLEAR;
  wuart->_rx_sts = UART_CLEAR;
  wuart->_rx_dr_thr = 0;

  FIFO_Init(&wuart->_rx_fifo, wuart->_rx_fifo_buf, UART_RX_FIFO_SIZE);
  FIFO_Init(&wuart->_tx_fifo, wuart->_tx_fifo_buf, UART_TX_FIFO_SIZE);
}


void wUART_SetDRCallback
(
wUART* wuart, //the target uart wrapper instance pointer
void (*DataReady_Callback)(void) //the data-ready callback function pointer
)
{
  wuart->_DataReady_Callback = DataReady_Callback;
}


void wUART_SetDRThreshold
(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t threshold //the _rx_dr_thr value, any value larger than 100 will be truncated to 100
)
{
  wuart->_rx_dr_thr = threshold > 100 ? 100 : threshold;
}


void wUART_SetTXCpltCallback
(
wUART* wuart, //the target uart wrapper instance pointer
void (*TxCplt_Callback)(void) //the tx-complete callback function pointer
)
{
  wuart->_TxCplt_Callback = TxCplt_Callback;
}


USART_TypeDef* wUART_GetUSARTInstance
(
wUART* wuart //the target uart wrapper instance pointer
)
{
  return wuart->_huart->Instance;
}


HAL_StatusTypeDef wUART_WriteTX
(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t* buf, //the buffer that will be sent
uint32_t len //the number of bytes to be sent
)
{
  if(len > UART_TX_DMA_BUF_SIZE) return HAL_ERROR;

    if(wuart->_tx_sts != UART_CLEAR)
      {
        BUFFER_Status sts = FIFO_Append(&wuart->_tx_fifo, buf, len);
        if(sts == BUF_ERROR) return HAL_ERROR;

        return HAL_OK;
      }

    wuart->_tx_sts = UART_BUSY;

    memcpy(wuart->_tx_buf_dma, buf, len);

    return HAL_UART_Transmit_DMA(wuart->_huart, wuart->_tx_buf_dma, len);
}


HAL_StatusTypeDef wUART_OpenRXDMA
(
wUART* wuart //the target uart wrapper instance pointer
)
{
  FIFO_Clear(&wuart->_rx_fifo);
  HAL_StatusTypeDef sts = HAL_UART_Receive_DMA(wuart->_huart, wuart->_rx_buf_dma, UART_RX_DMA_BUF_SIZE);

  if(sts == HAL_OK)
    {
      wuart->_rx_sts = UART_BUSY;
      return HAL_OK;
    }

  wuart->_rx_sts = UART_ERROR;
  return HAL_ERROR;
}


HAL_StatusTypeDef wUART_CloseRXDMA
(
wUART* wuart //the target uart wrapper instance pointer
)
{
  wuart->_rx_sts = UART_CLEAR;
  return HAL_UART_DMAStop(wuart->_huart);
}


uint32_t wUART_ReadRXFIFO
(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t* buf, //the buffer that will be read into
uint32_t len //the max number of bytes to read
)
{
  wuart->_rx_sts = UART_BUSY;
  uint32_t read_n = FIFO_Pop(&wuart->_rx_fifo, buf, len);
  if(wuart->_rx_sts != UART_OVERWRITE)
    {
      wuart->_rx_sts = UART_CLEAR;
      return read_n;
    }

  return UART_ERROR;
}


void wUART_RX_FCISR_Impl(wUART* wuart)
{
  __asm__ volatile ("" ::: "memory");

  uint32_t loc_dma_p = wuart->_rx_dma_p;
  wuart->_rx_dma_p = 0;

  uint32_t dma_len = UART_RX_DMA_BUF_SIZE - loc_dma_p;

  if(FIFO_GetEmpty(&wuart->_rx_fifo) < dma_len)
    {
      if(wuart->_rx_sts != UART_CLEAR) wuart->_rx_sts = UART_OVERWRITE;
      FIFO_Discard(&wuart->_rx_fifo,  dma_len);
    }

  FIFO_Append(&wuart->_rx_fifo, wuart->_rx_buf_dma, dma_len);

  if(FIFO_GetCount(&wuart->_rx_fifo) > (wuart->_rx_fifo._size * wuart->_rx_dr_thr / 100)\
      && wuart->_DataReady_Callback != NULL) wuart->_DataReady_Callback();
}


void wUART_RX_HCISR_Impl(wUART* wuart)
{
  __asm__ volatile ("" ::: "memory");

  uint32_t loc_dma_p = wuart->_rx_dma_p;
  wuart->_rx_dma_p = UART_RX_DMA_BUF_SIZE/2;

  uint32_t dma_len = UART_RX_DMA_BUF_SIZE/2 - loc_dma_p;

  if(FIFO_GetEmpty(&wuart->_rx_fifo) < dma_len)
    {
      if(wuart->_rx_sts != UART_CLEAR) wuart->_rx_sts = UART_OVERWRITE;
      FIFO_Discard(&wuart->_rx_fifo,  dma_len);
    }

  FIFO_Append(&wuart->_rx_fifo, wuart->_rx_buf_dma, dma_len);

  if(FIFO_GetCount(&wuart->_rx_fifo) > (wuart->_rx_fifo._size * wuart->_rx_dr_thr / 100)\
      && wuart->_DataReady_Callback != NULL) wuart->_DataReady_Callback();
}


void wUART_RX_IDLEISR_Impl(wUART* wuart)
{
  __asm__ volatile ("" ::: "memory");

  uint32_t loc_dma_p = UART_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(wuart->_huart->hdmarx);

  uint32_t dma_len = loc_dma_p - wuart->_rx_dma_p;

  if(FIFO_GetEmpty(&wuart->_rx_fifo) < dma_len)
    {
      if(wuart->_rx_sts != UART_CLEAR) wuart->_rx_sts = UART_OVERWRITE;
      FIFO_Discard(&wuart->_rx_fifo,  dma_len);
    }

  FIFO_Append(&wuart->_rx_fifo, wuart->_rx_buf_dma + wuart->_rx_dma_p, dma_len);

  if(wuart->_DataReady_Callback != NULL) wuart->_DataReady_Callback();

  wuart->_rx_dma_p = loc_dma_p;
}


void wUART_TX_FCISR_Impl(wUART* wuart)
{
  if(FIFO_GetCount(&wuart->_tx_fifo) == 0)
      {
      wuart->_tx_sts = UART_CLEAR;
        if(wuart->_TxCplt_Callback != NULL) wuart->_TxCplt_Callback();
        return;
      }

  uint32_t pop_size = FIFO_Pop(&wuart->_tx_fifo, wuart->_tx_buf_dma, UART_TX_DMA_BUF_SIZE);
  HAL_UART_Transmit_DMA(wuart->_huart, wuart->_tx_buf_dma, pop_size);
}
