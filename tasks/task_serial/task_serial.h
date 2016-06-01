/*******************************************************************************
 * task_serial.h
 *
 * Description:
 *  This module is used to initialize serial communication module. The module
 *  uses UART interface to transmit and receive data. To ensure data integrity
 *  the message is transmitted with a start and end byte, length information
 *  and a CRC checksum. A valid serial data stream has to follow the convention
 *  |STX|LENGTH|DATA|DATA| ... |DATA|CRC16|CRC16|ETX|.
 *  Data transmission is done calling the appropriate function. Receiving is
 *  implemented within a task which is executed when data is available. Data
 *  is interpreted within this task. Valid or invalid data is reported by
 *  transmitting ACK and NAK respectively.
 *
 * Functions:
 *  error_t task_serial_init(void);
 *  error_t serial_transmit(uint8_t* data, uint16_t size);
 *
 * Note:
 *  This module was intended to be used on Kinetis MKL25Z128 microcontrollers.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/

#ifndef TASK_SERIAL_H_
#define TASK_SERIAL_H_

#include "error.h"
#include "mkl25z128xxx4.h"

#define TASK_SERIAL_MSG_CMD_BLOCK       0x10
#define TASK_SERIAL_MSG_DAT_BLOCK       0x11

error_t task_serial_init(void);
error_t serial_transmit(uint8_t* data, uint16_t size);

#endif /* TASK_SERIAL_H_ */
