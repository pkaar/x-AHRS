/*******************************************************************************
 * hmc5883l.h
 *
 * Description:
 *  This module provides methods to interface with the HMC5883L 3-axis
 *  magnetometer. All required register configurations are set during
 *  initialization. Reading and writing of registers is provided as well as the
 *  reading of magnetometer data.
 *
 * Functions:
 *  error_t hmc5883l_init(void);
 *  error_t hmc5883l_reg_read(uint8_t reg, uint8_t* value);
 *  error_t hmc5883l_reg_write(uint8_t reg, uint8_t value);
 *  error_t hmc5883l_rawdata_read(vector_int16_t* magnetic_field);
 *  error_t hmc5883l_data_read(vector_float_t* magnetic_field);
 *  error_t hmc5883l_int_status(uint8_t* state);
 *
 * Note:
 *  This module requires Kinetis KL25Z128 I2C library.
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "error.h"
#include "mkl25z128xxx4.h"
#include "types.h"

/* Uncomment this preprocessor directive to enable MPU6050 interrupt support. */
/* #define HMC5883L_USE_INT */

#define HMC5883L_CONF_REG_A         0x00
#define HMC5883L_CONF_REG_B         0x01
#define HMC5883L_MODE_REG           0x02
#define HMC5883L_DATA_X_MSB         0x03
#define HMC5883L_DATA_X_LSB         0x04
#define HMC5883L_DATA_Z_MSB         0x05
#define HMC5883L_DATA_Z_LSB         0x06
#define HMC5883L_DATA_Y_MSB         0x07
#define HMC5883L_DATA_Y_LSB         0x08
#define HMC5883L_STATUS_REG         0x09
#define HMC5883L_ID_REG_A           0x0A
#define HMC5883L_ID_REG_B           0x0B
#define HMC5883L_ID_REG_C           0x0C

error_t hmc5883l_init(void);
error_t hmc5883l_reg_read(uint8_t reg, uint8_t* value);
error_t hmc5883l_reg_write(uint8_t reg, uint8_t value);
error_t hmc5883l_rawdata_read(vector_int16_t* magnetic_field);
error_t hmc5883l_data_read(vector_float_t* magnetic_field);
#ifdef HMC5883L_USE_INT
error_t hmc5883l_int_status(uint8_t* state);
#endif

#endif /* HMC5883L_H_ */
