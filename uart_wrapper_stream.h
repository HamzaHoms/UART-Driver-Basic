/*
 * uart_wrapper_stream.h
 *
 *  Created on: Oct 21, 2026
 *      Author: Hamza
 */

#ifndef INC_UART_WRAPPER_STREAM_H_
#define INC_UART_WRAPPER_STREAM_H_

#include "buffers.h"

#if defined STM32F103xB
#include "stm32f1xx.h"

#elif defined STM32F411xE
#include "stm32f4xx.h"

#else
#error "stm32xxx version not supported"

#endif


#define UART_RX_FIFO_SIZE 512
#define UART_TX_FIFO_SIZE 512

#define UART_RX_DMA_BUF_SIZE 256
#define UART_TX_DMA_BUF_SIZE 256


typedef enum{
  UART_CLEAR = 0,
  UART_BUSY = 1,
  UART_TX_PAUSE = 2,
  UART_ERROR = 3,
  UART_DESYNC = 4,
  UART_OVERWRITE = 5
}UART_State;


typedef struct{
  UART_HandleTypeDef* _huart;
  volatile UART_State _tx_sts;
  volatile UART_State _rx_sts;
  void (*_DataReady_Callback)(void);
  void (*_TxCplt_Callback)(void);
  uint8_t _rx_dr_thr;

  uint8_t _rx_fifo_buf[UART_RX_FIFO_SIZE];
  uint8_t _tx_fifo_buf[UART_TX_FIFO_SIZE];

  FIFO_t _rx_fifo;
  FIFO_t _tx_fifo;

  uint8_t _tx_buf_dma[UART_TX_DMA_BUF_SIZE];
  uint8_t _rx_buf_dma[UART_RX_DMA_BUF_SIZE];
  volatile uint32_t _rx_dma_p;

} wUART;

//Setup Functions=======================================================================

//Initializes the uart wrapper internal variables and FIFOs
void wUART_Init(
wUART* wuart, //the target uart wrapper instance pointer
UART_HandleTypeDef* huart //pointer to the HAL uart handler to be used in 'wuart'
);

//sets the uart wrapper's callback function that is called upon data being ready in the RX FIFO
//there are two cases where this callback could be called:
// 1) when the filled portion of the RX FIFO exceeds _rx_dr_thr% (_rx_dr_thr = 70 would mean the callback is called at 70% full)
// 2) when the uart line goes idle and the idle line isr fires, this only works if the uart is receiving chunks of data, not constant streams
void wUART_SetDRCallback(
wUART* wuart, //the target uart wrapper instance pointer
void (*DataReady_Callback)(void) //the data-ready callback function pointer
);

//sets the previously mentioned _rx_dr_thr, by default _rx_dr_thr is 0, which means any IRQ firing calls it instantly
void wUART_SetDRThreshold(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t threshold //the _rx_dr_thr value, any value larger than 100 will be truncated to 100
);

//sets the uart wrapper's callback function that is called upon completion of uart transmission
//this callback is only called once the tx fifo is completely emptied, it is not called on every tx complete
//this means that a saturated tx fifo that is constantly being filled will prevent this callback from being called
void wUART_SetTXCpltCallback(
wUART* wuart, //the target uart wrapper instance pointer
void (*TxCplt_Callback)(void) //the tx-complete callback function pointer
);

//returns the uart handler instance value, for example USART1, USART2, etc..
USART_TypeDef* wUART_GetUSARTInstance(
wUART* wuart //the target uart wrapper instance pointer
);

//transmits 'len' bytes from 'buf' on the uart tx line
//if a tx is already active the target bytes are copied into the tx fifo
//this function kick starts the uart transmission, and the transmissions keep going until the tx fifo is empty
HAL_StatusTypeDef wUART_WriteTX(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t* buf, //the buffer that will be sent
uint32_t len //the number of bytes to be sent
);

//starts the circular dma reception on uart rx line and resets the rx fifo
//the reception is not stopped unless an error occurs or wUART_CloseRXDMA is called
//the bytes are received into the dma buffer and automatically moved into rx fifo in a double buffer configuration
//this means that the dma buffer is consantly being read from and written to
//the bytes in the dma buffer are moved into the rx fifo using the 3 wUART_RX_XXXXISR_imple functions shown below
HAL_StatusTypeDef wUART_OpenRXDMA(
wUART* wuart //the target uart wrapper instance pointer
);

//stops the dma reception on the uart rx line
HAL_StatusTypeDef wUART_CloseRXDMA(
wUART* wuart //the target uart wrapper instance pointer
);

//reads 'len' bytes from the rx fifo into 'buf'
//returns the number of bytes read, in case the rx fifo has less than 'len' bytes in it
uint32_t wUART_ReadRXFIFO(
wUART* wuart, //the target uart wrapper instance pointer
uint8_t* buf, //the buffer that will be read into
uint32_t len //the max number of bytes to read
);

//below are functions that need to be placed into their respective ISR HAL callback functions
//these functions are responsible for moving the ready bytes from the dma buffer into the rx fifo
//the uart wrapper cannot function without them
void wUART_RX_FCISR_Impl(wUART* wuart);
void wUART_RX_HCISR_Impl(wUART* wuart);
void wUART_RX_IDLEISR_Impl(wUART* wuart);

//below is the tx complete function, it restarts the tx dma operation on the bytes that are in the tx fifo
//it does that until the tx fifo is empty as specified in the comments for wUART_WriteTX
//once the tx fifo is empty the tx-complete callback is called
void wUART_TX_FCISR_Impl(wUART* wuart);

#endif /* INC_UART_WRAPPER_STREAM_H_ */
