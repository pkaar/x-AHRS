/*******************************************************************************
 * i2c.h
 *
 * Description:
 *  This module initializes Kinetis MKL25Z128 I2C0 module to work in master
 *  mode. The module can be initialized to work in standard mode (100 kHz) and
 *  high speed mode (400 kHz) and provides routines for transmitting and
 *  receiving bytes.
 *
 * Functions:
 *  error_t i2c_init(uint8_t mode)
 *  error_t i2c_transmit(uint8_t addr, uint8_t* data, uint32_t size)
 *  error_t i2c_receive(uint8_t addr, uint8_t* data, uint32_t size)
 *
 * Note:
 *  This module is intended to be used on Freescales Kinetis MKL25Z128
 *  processors.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/

#ifndef I2C_H_
#define I2C_H_

#include "clock.h"
#include "error.h"
#include "mkl25z128xxx4.h"

#define I2C_STANDARD_MODE      0x00
#define I2C_FAST_MODE          0x01

error_t i2c_init(uint8_t mode);
error_t i2c_transmit(uint8_t addr, uint8_t* data, uint32_t size);
error_t i2c_receive(uint8_t addr, uint8_t* data, uint32_t size);

#endif /* I2C_H_ */
