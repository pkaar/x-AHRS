/*******************************************************************************
 * uart.c
 *
 * Description:
 *  This module is used to initialize Kinetis KL-series universal asynchronous
 *  receiver-transmitter (UART0) module as described below.
 *
 * Functions:
 *  error_t uart_init(uint32_t baud)
 *  error_t uart_transmit(uint8_t* data, uint16_t size)
 *  error_t uart_receive(uint8_t* data, uint16_t size)
 *
 * Note:
 *  This module is intended to be used on Freescales Kinetis MKL25Z128
 *  processors.
 *
 * History:
 *  pka, 30/SEP/2015, initial code
 ******************************************************************************/

#ifndef UART_H_
#define UART_H_

#include "error.h"
#include "mkl25z128xxx4.h"

error_t uart_init(uint32_t baud);
error_t uart_transmit(uint8_t* data, uint16_t size);
error_t uart_receive(uint8_t* data, uint16_t size);

extern void uart_rx_callback(void) __attribute__((weak));

#endif /* UART_H_ */
