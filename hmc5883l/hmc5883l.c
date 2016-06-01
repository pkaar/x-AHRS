/*******************************************************************************
 * hmc5883l.c
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

#include "hmc5883l.h"

#include "gpio.h"
#include "i2c.h"

#define HMC5883L_CALIBRATION    1

#define HMC5883L_SLAVE_ADDR     0x1E

#define HMC5883L_INT            PTB3

/* HMC5883L register configuration array. */
const uint8_t hmc5883l_config[][2] = {
        {HMC5883L_CONF_REG_A,   0x18},  /* sample rate 75 Hz */
        {HMC5883L_CONF_REG_B,   0x60},  /* magnetometer full scale ±2.5 Ga */
        {HMC5883L_MODE_REG,     0x00}   /* continuous measurement mode */
};

#if (HMC5883L_CALIBRATION == 1)
/* Hard iron correction. */
const float hard_iron_correction[3] = {
         -91.550547,
         -66.992152,
           8.899025,
};
#else
const float hard_iron_correction[3] = {
           0.000000,
           0.000000,
           0.000000,
};
#endif

#if (HMC5883L_CALIBRATION == 1)
/* Soft iron correction. */
const float soft_iron_correction[3][3] = {
        { 1.458055, -0.006956,  0.004659},
        {-0.006956,  1.440539, -0.000296},
        { 0.004659, -0.000296,  1.672037}
};
#else
const float soft_iron_correction[3][3] = {
        { 1.000000,  0.000000,  0.000000},
        { 0.000000,  1.000000,  0.000000},
        { 0.000000,  0.000000,  1.000000}
};
#endif

/*******************************************************************************
 * error_t hmc5883l_init(void)
 *
 * Description:
 *  Initialize HMC5883L magnetometer. All required register configurations are
 *  done in this initialization routine. Possible errors are reported to the
 *  calling function.
 *
 * Parameters:
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_init();
 *
 * Note:
 *  Register configuration values are stored in hmc5883l_config field.
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
error_t hmc5883l_init(void)
{
    uint8_t reg_cnt;
    uint8_t cnt;

    /* Initialize I2C hardware module and report error if initialization
     * failed. */
    if (i2c_init(I2C_FAST_MODE) != ERR_OK)
    {
        return ERR_FAIL;
    }

#ifdef HMC5883L_USE_INT
    if (gpio_init(HMC5883L_INT, GPIO_INPUT, GPIO_FLOAT) != ERR_OK)
    {
        return ERR_FAIL;
    }
#endif

    /* Calculate number of registers to configure. */
    reg_cnt = sizeof(hmc5883l_config) / 2;

    for (cnt = 0; cnt < reg_cnt; cnt++)
    {
        if (hmc5883l_reg_write(hmc5883l_config[cnt][0],
                hmc5883l_config[cnt][1]) != ERR_OK)
        {
            return ERR_FAIL;
        }
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t hmc5883l_reg_read(uint8_t reg, uint8_t* value)
 *
 * Description:
 *  Read data from HMC5883L register. Only single byte reads from registers are
 *  supported. Possible errors are reported to the calling function.
 *
 * Parameters:
 *  reg         register to be read
 *  value       memory address of register value
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_reg_read(HMC5883L_CONF_REG_A, &value);
 *
 * Note:
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
error_t hmc5883l_reg_read(uint8_t reg, uint8_t* value)
{
    /* Transmit START condition and slave address + WRITE. Write register
     * address and generate STOP condition. */
    if (i2c_transmit(HMC5883L_SLAVE_ADDR, &reg, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Transmit START condition and slave address + READ. Read register value
     * and generate STOP condition. */
    if (i2c_receive(HMC5883L_SLAVE_ADDR, value, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t hmc5883l_reg_write(uint8_t reg, uint8_t value)
 *
 * Description:
 *  Write data to HMC5883L register. Only single byte writes to registers are
 *  supported. Possible errors are reported to the calling function.
 *
 * Parameters:
 *  reg         address of register to be written
 *  value       value to be written to specified register
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_reg_write(HMC5883L_CONF_REG_A, 0x18);
 *
 * Note:
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
error_t hmc5883l_reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tmp[2];

    /* Prepare data array to be written to I2C bus. */
    tmp[0] = reg;
    tmp[1] = value;

    /* Transmit START condition and slave address + WRITE. Write register
     * address followed by register value and generate STOP condition. */
    if (i2c_transmit(HMC5883L_SLAVE_ADDR, tmp, 2) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t hmc5883l_rawdata_read(vector_int16_t* magnetic_field)
 *
 * Description:
 *  Read HMC5883L raw magnetic field data. Magnetic field data is stored as a
 *  3-dimensional 16-bit unsigned integer vector where only the lowest 12-bits
 *  contain data. Possible errors are reported to the calling function.
 *
 * Parameters:
 *  magnetic_field      memory address of 3-dimensional magnetic field vector
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_rawdata_read(&magnetic_field);
 *
 * Note:
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
error_t hmc5883l_rawdata_read(vector_int16_t* magnetic_field)
{
    uint8_t tmp[6];

    tmp[0] = HMC5883L_DATA_X_MSB;

    /* Transmit START condition and slave address + WRITE. Write register
     * address and generate STOP condition. */
    if (i2c_transmit(HMC5883L_SLAVE_ADDR, tmp, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Transmit START condition and slave address + READ. Read register value
     * and generate STOP condition. */
    if (i2c_receive(HMC5883L_SLAVE_ADDR, tmp, 6) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Create 3-dimensional magnetic field vector. Note: The HMC5883L provides
     * raw data in the order X, Z, Y. */
    magnetic_field->x = (tmp[0] << 8) | tmp[1];
    magnetic_field->y = (tmp[4] << 8) | tmp[5];
    magnetic_field->z = (tmp[2] << 8) | tmp[3];

    return ERR_OK;
}


/*******************************************************************************
 * error_t hmc5883l_data_read(vector_float_t* magnetic_field)
 *
 * Description:
 *  Read HMC5883L calibrated magnetic field data. Magnetic field data is stored
 *  as a 3-dimensional float vector. Possible errors are reported to the
 *  calling function.
 *
 * Parameters:
 *  magnetic_field      memory address of 3-dimensional magnetic field vector
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_data_read(&magnetic_field);
 *
 * Note:
 *  Calibrated magnetic field data is returned in mG.
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
error_t hmc5883l_data_read(vector_float_t* magnetic_field)
{
    vector_int16_t raw_magnetic_field;

    /* Read magnetometer raw data. */
    if (hmc5883l_rawdata_read(&raw_magnetic_field) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Correct hard iron effects. */
    magnetic_field->x = raw_magnetic_field.x - hard_iron_correction[0];
    magnetic_field->y = raw_magnetic_field.y - hard_iron_correction[1];
    magnetic_field->z = raw_magnetic_field.z - hard_iron_correction[2];

    /* Correct soft iron effects. */
    magnetic_field->x =
            soft_iron_correction[0][0] * magnetic_field->x +
            soft_iron_correction[0][1] * magnetic_field->y +
            soft_iron_correction[0][2] * magnetic_field->z;
    magnetic_field->y =
            soft_iron_correction[1][0] * magnetic_field->x +
            soft_iron_correction[1][1] * magnetic_field->y +
            soft_iron_correction[1][2] * magnetic_field->z;
    magnetic_field->z =
            soft_iron_correction[2][0] * magnetic_field->x +
            soft_iron_correction[2][1] * magnetic_field->y +
            soft_iron_correction[2][2] * magnetic_field->z;

    return ERR_OK;
}


/*******************************************************************************
 * error_t hmc5883l_int_status(uint8_t* state)
 *
 * Description:
 *  Get HMC5883L interrupt pin status. Depending on the configuration of the
 *  HMC5883L the logical level of this pin indicates a data complete interrupt.
 *  Possible errors are reported to the calling function.
 *
 * Parameters:
 *  state               memory address of HMC5883L interrupt state
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = hmc5883l_int_status(&state);
 *
 * Note:
 *  Use the preprocessor directive #define HMC5883L_USE_INT to enable HMC5883L
 *  interrupt support. The interrupt handling is not covered by this module.
 *
 * History:
 *  pka, 03/SEP/2015, initial code
 ******************************************************************************/
#ifdef HMC5883L_USE_INT
error_t hmc5883l_int_status(uint8_t* state)
{
    if (gpio_get(HMC5883L_INT, state) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}
#endif
