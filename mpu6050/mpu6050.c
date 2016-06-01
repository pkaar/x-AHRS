/*******************************************************************************
 * mpu6050.c
 *
 * Description:
 *  This module provides methods to interface with the MPU6050 6-axis inertial
 *  measurement unit. All required register configurations are set during
 *  initialization. Reading and writing of registers is provided as well as the
 *  reading of acceleration and angular velocity.
 *
 * Functions:
 *  error_t mpu6050_init(void);
 *  error_t mpu6050_reg_read(uint8_t reg, uint8_t* value);
 *  error_t mpu6050_reg_write(uint8_t reg, uint8_t value);
 *  error_t mpu6050_rawdata_read(vector_int16_t* acceleration, vector_int16_t* angular_velocity);
 *  error_t mpu6050_data_read(vector_float_t* acceleration, vector_float_t* angular_velocity);
 *  error_t mpu6050_int_status(uint8_t* state);
 *
 * Note:
 *  This module requires Kinetis KL25Z128 I2C library.
 *
 * History:
 *  pka, 30/OCT/2015, changed angular velocity units to rad/s
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/

#include "mpu6050.h"

#include "gpio.h"
#include "i2c.h"

#define MPU6050_CALIBRATION     1

#define MPU6050_SLAVE_ADDR      0x68

#define MPU6050_INT             PTB2

#ifdef MPU6050_USE_INT
const uint8_t mpu6050_config[][2] = {
        {MPU6050_PWR_MGMT_1,    0x09},
        {MPU6050_SMPLRT_DIV,    0x01},  /* sample rate 500 Hz */
        {MPU6050_CONFIG,        0x01},  /* bandwidth 188 Hz */
        {MPU6050_ACCEL_CONFIG,  0x08},  /* accelerometer full scale ± 4 g */
        {MPU6050_GYRO_CONFIG,   0x18},  /* gyroscope full scale ± 2000 °/s */
        {MPU6050_INT_PIN_CFG,   0xF0},  /* int pin active low, open-drain */
        {MPU6050_INT_ENABLE,    0x01}   /* enable data ready interrupt */
};
#else
const uint8_t mpu6050_config[][2] = {
        {MPU6050_PWR_MGMT_1,    0x09},
        {MPU6050_SMPLRT_DIV,    0x01},  /* sample rate 500 Hz */
        {MPU6050_CONFIG,        0x01},  /* bandwidth 188 Hz */
        {MPU6050_ACCEL_CONFIG,  0x08},  /* accelerometer full scale ± 4 g */
        {MPU6050_GYRO_CONFIG,   0x18},  /* gyroscope full scale ± 2000 °/s */
};
#endif

/* Acceleration scaling factor. The scaling factor is calculated as follows:
 * scaling = (full scale range) / (resolution) * 1000, where full scale range
 * is 8 g and resolution is given by 2 ^ 16. Multiplication with 1000 is done
 * to get acceleration in mg. */
const float acceleration_scaling = 0.12207030;

#if (MPU6050_CALIBRATION == 1)
/* Acceleration offset. */
const float acceleration_offset[3] = {
          19.6447,
          -1.1062,
         -66.4384
};
#else
const float acceleration_offset[3] = {
           0.0000,
           0.0000,
           0.0000
};
#endif

/* Angular velocity scaling factor. The scaling factor is calculated as follows:
 * scaling = (full scale range) / (resolution) * pi / 180, where full scale
 * range is 2000 °/s and resolution is given by 2 ^ 16. Multiplication with
 * pi / 180 is done to get angular velocity in rad/s. */
const float angular_velocity_scaling = 0.00106526;

#if (MPU6050_CALIBRATION == 1)
/* Angular velocity offset. */
const float angular_velocity_offset[3] = {
           0.040030,
           0.011660,
           0.005506
};
#else
const float angular_velocity_offset[3] = {
           0.000000,
           0.000000,
           0.000000
};
#endif

/*******************************************************************************
 * error_t mpu6050_init(void)
 *
 * Description:
 *  Initialize MPU6050 inertial measurement unit. All required register
 *  configurations are done in this initialization routine. Possible errors are
 *  reported to the calling function.
 *
 * Parameters:
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = mpu6050_init();
 *
 * Note:
 *  Register configuration values are stored in mpu6050_config field.
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
error_t mpu6050_init(void)
{
    uint8_t reg_cnt;
    uint8_t cnt;

    /* Initialize I2C hardware module and report error if initialization
     * failed. */
    if (i2c_init(I2C_FAST_MODE)!= ERR_OK)
    {
        return ERR_FAIL;
    }

#ifdef MPU6050_USE_INT
    if (gpio_init(MPU6050_INT, GPIO_INPUT, GPIO_PULLUP) != ERR_OK)
    {
        return ERR_FAIL;
    }
#endif

    /* Calculate number of registers to configure. */
    reg_cnt = sizeof(mpu6050_config) / 2;

    for (cnt = 0; cnt < reg_cnt; cnt++)
    {
        if (mpu6050_reg_write(mpu6050_config[cnt][0],
                mpu6050_config[cnt][1]) != ERR_OK)
        {
            return ERR_FAIL;
        }
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t mpu6050_reg_read(uint8_t reg, uint8_t* value)
 *
 * Description:
 *  Read data from MPU6050 register. Only single byte reads from registers are
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
 *  error = mpu6050_reg_read(MPU6050_GYRO_CONFIG, &value);
 *
 * Note:
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
error_t mpu6050_reg_read(uint8_t reg, uint8_t* value)
{
    /* Transmit START condition and slave address + WRITE. Write register
     * address and generate STOP condition. */
    if (i2c_transmit(MPU6050_SLAVE_ADDR, &reg, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Transmit START condition and slave address + READ. Read register value
     * and generate STOP condition. */
    if (i2c_receive(MPU6050_SLAVE_ADDR, value, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t mpu6050_reg_write(uint8_t reg, uint8_t value)
 *
 * Description:
 *  Write data to MPU6050 register. Only single byte writes to registers are
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
 *  error = mpu6050_reg_write(MPU6050_GYRO_CONFIG, 0x18);
 *
 * Note:
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
error_t mpu6050_reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tmp[2];

    /* Prepare data array to be written to I2C bus. */
    tmp[0] = reg;
    tmp[1] = value;

    /* Transmit START condition and slave address + WRITE. Write register
     * address followed by register value and generate STOP condition. */
    if (i2c_transmit(MPU6050_SLAVE_ADDR, tmp, 2) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t mpu6050_rawdata_read(vector_int16_t* acceleration, vector_int16_t* angular_velocity)
 *
 * Description:
 *  Read MPU6050 acceleration and angular velocity data. Acceleration and
 *  angular velocity data are stored as a 3-dimensional 16-bit vectors. Possible
 *  errors are reported to the calling function.
 *
 * Parameters:
 *  acceleration        memory address of 3-dimensional acceleration vector
 *  angular_velocity    memory address of 3-dimensional angular velocity vector
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = mpu6050_rawdata_read(&acceleration, &angular_velocity);
 *
 * Note:
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
error_t mpu6050_rawdata_read(vector_int16_t* acceleration, vector_int16_t* angular_velocity)
{
    uint8_t tmp[6];

    tmp[0] = MPU6050_ACCEL_XOUT_H;

    /* Transmit START condition and slave address + WRITE. Write register
     * address and generate STOP condition. */
    if (i2c_transmit(MPU6050_SLAVE_ADDR, tmp, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Transmit START condition and slave address + READ. Read register value
     * and generate STOP condition. */
    if (i2c_receive(MPU6050_SLAVE_ADDR, tmp, 6) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Create 3-dimensional acceleration vector. */
    acceleration->x = (tmp[0] << 8) | tmp[1];
    acceleration->y = (tmp[2] << 8) | tmp[3];
    acceleration->z = (tmp[4] << 8) | tmp[5];

    tmp[0] = MPU6050_GYRO_XOUT_H;

    /* Transmit START condition and slave address + WRITE. Write register
     * address and generate STOP condition. */
    if (i2c_transmit(MPU6050_SLAVE_ADDR, tmp, 1) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Transmit START condition and slave address + READ. Read register value
     * and generate STOP condition. */
    if (i2c_receive(MPU6050_SLAVE_ADDR, tmp, 6) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Create 3-dimensional angular velocity vector. */
    angular_velocity->x = (tmp[0] << 8) | tmp[1];
    angular_velocity->y = (tmp[2] << 8) | tmp[3];
    angular_velocity->z = (tmp[4] << 8) | tmp[5];

    return ERR_OK;
}


/*******************************************************************************
 * error_t mpu6050_data_read(vector_float_t* acceleration, vector_float_t* angular_velocity)
 *
 * Description:
 *  Read MPU6050 calibrated acceleration and angular velocity data. Acceleration
 *  and angular velocity data are stored as a 3-dimensional float vectors.
 *  Possible errors are reported to the calling function.
 *
 * Parameters:
 *  acceleration        memory address of 3-dimensional acceleration vector
 *  angular_velocity    memory address of 3-dimensional angular velocity vector
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = mpu6050_rawdata_read(&acceleration, &angular_velocity);
 *
 * Note:
 *  Calibrated acceleration data is returned in mg, calibrated angular velocity
 *  data in rad/s.
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
error_t mpu6050_data_read(vector_float_t* acceleration, vector_float_t* angular_velocity)
{
    vector_int16_t raw_acceleration;
    vector_int16_t raw_angular_velocity;

    /* Read accelerometer and gyroscope raw data. */
    if (mpu6050_rawdata_read(&raw_acceleration, &raw_angular_velocity) != ERR_OK)
    {
        return ERR_FAIL;
    }

    /* Scale acceleration raw data. */
    acceleration->x = raw_acceleration.x * acceleration_scaling -
            acceleration_offset[0];
    acceleration->y = raw_acceleration.y * acceleration_scaling -
            acceleration_offset[1];
    acceleration->z = raw_acceleration.z * acceleration_scaling -
            acceleration_offset[2];

    /* Scale angular velocity raw data. */
    angular_velocity->x = raw_angular_velocity.x * angular_velocity_scaling -
            angular_velocity_offset[0];
    angular_velocity->y = raw_angular_velocity.y * angular_velocity_scaling -
            angular_velocity_offset[1];
    angular_velocity->z = raw_angular_velocity.z * angular_velocity_scaling -
            angular_velocity_offset[2];

    return ERR_OK;
}


/*******************************************************************************
 * error_t mpu6050_int_status(uint8_t* state)
 *
 * Description:
 *  Get MPU6050 interrupt pin status. Depending on the configuration of the
 *  MPU6050 the logical level of this pin indicates a data complete interrupt.
 *  Possible errors are reported to the calling function.
 *
 * Parameters:
 *  state               memory address of MPU6050 interrupt state
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write register
 *
 * Example:
 *  error = mpu6050_int_status(&state);
 *
 * Note:
 *  Use the preprocessor directive #define MPU6050_USE_INT to enable MPU6050
 *  interrupt support. The interrupt handling is not covered by this module.
 *
 * History:
 *  pka, 17/SEP/2015, initial code
 ******************************************************************************/
#ifdef MPU6050_USE_INT
error_t mpu6050_int_status(uint8_t* state)
{
    if (gpio_get(MPU6050_INT, state) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}
#endif
