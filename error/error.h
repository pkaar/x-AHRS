/*
 * error.h
 *
 *  Created on: Aug 26, 2015
 *      Author: pkaar
 */

#ifndef ERROR_H_
#define ERROR_H_

typedef enum {
    ERR_OK,                     /* no error */
    ERR_FAIL,                   /* unspecified error */
    ERR_PARAM_RANGE,            /* parameter out of range */
    ERR_PARAM_VALUE,            /* parameter of incorrect value */
    ERR_BUF_EMPTY,              /* buffer empty */
    ERR_BUF_FULL,               /* buffer full */
    ERR_GPIO_PIN,               /* invalid GPIO pin */
    ERR_GPIO_MODE,              /* invalid GPIO mode */
    ERR_GPIO_PULLMODE,          /* invalid GPIO pull mode */
    ERR_GPIO_LEVEL,             /* invalid GPIO logical level */
    ERR_GPIO_INIT,              /* GPIO pin not initialized */
    ERR_I2C_SPEED,              /* invalid I2C speed */
    ERR_I2C_BUSY,
    ERR_I2C_ACK,
    ERR_UART_SPEED,
    ERR_UART_BUSY
} error_t;

#endif /* ERROR_H_ */
