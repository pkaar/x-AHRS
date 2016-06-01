/*******************************************************************************
 * i2c.c
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

#include "i2c.h"

#define I2C_READ                0x01
#define I2C_WRITE               0x00

/*******************************************************************************
 * error_t i2c_start(void)
 *
 * Description:
 *  Send START signal to I2C bus. The START signal is only generated if the bus
 *  is idle.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_I2C_BUSY            - bus busy, START signal not generated
 *
 * Example:
 *  error = i2c_start();
 *
 * Note:
 *  The I2C module has to be initialized before calling this routine.
 *  This routine is designed to be called only in this module. Therefore it is
 *  not accessible outside of this module.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_start(void)
{
    /* Check if bus is busy. */
    if (I2C0_S & I2C_S_BUSY_MASK)
    {
        return ERR_I2C_BUSY;
    }

    /* Set mode to transmit. */
    I2C0_C1 |= I2C_C1_TX_MASK;

    /* Generate START signal. */
    I2C0_C1 |= I2C_C1_MST_MASK;

    /* Wait until START signal was detected. */
    while (!(I2C0_S & I2C_S_BUSY_MASK));

    return ERR_OK;
}


/*******************************************************************************
 * error_t i2c_stop(void)
 *
 * Description:
 *  Send STOP signal to I2C bus.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = i2c_stop();
 *
 * Note:
 *  The I2C module has to be initialized before calling this routine.
 *  This routine is designed to be called only in this module. Therefore it is
 *  not accessible outside of this module.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_stop(void)
{
    /* Generate STOP signal. */
    I2C0_C1 &= ~I2C_C1_MST_MASK;

    /* Set mode to transmit. */
    I2C0_C1 &= ~I2C_C1_TX_MASK;

    /* Wait until STOP signal was detected. */
    while (I2C0_S & I2C_S_BUSY_MASK);

    return ERR_OK;
}


/*******************************************************************************
 * error_t i2c_restart(void)
 *
 * Description:
 *  Send repeated START signal to I2C bus.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = i2c_restart();
 *
 * Note:
 *  The I2C module has to be initialized before calling this routine.
 *  This routine is designed to be called only in this module. Therefore it is
 *  not accessible outside of this module.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_restart(void)
{
    uint8_t reg;

    /* Save current content of I2C frequency divider register. According to
     * errata e6070 a repeated START condition cannot be generated if the
     * IC2x_F[MULT] field is set to a non-zero value. */
    reg = I2C0_F;

    /* Clear I2Cx_F[MULT] field. */
    I2C0_F = reg & ~I2C_F_MULT_MASK;

    /* Generate repeated START signal and set mode to transmit. */
    I2C0_C1 |= I2C_C1_RSTA_MASK;

    /* Check if I2C bus is busy and wait until it becomes idle. */
    while (!(I2C0_S & I2C_S_BUSY_MASK));

    /* Restore I2C frequency divider register. */
    I2C0_F = reg;

    return ERR_OK;
}


/*******************************************************************************
 * error_t i2c_init(uint8_t mode)
 *
 * Description:
 *  Initialize I2C0 module. The module uses PTB0 and PTB1 as SCL and SDA lines
 *  and can be configured to work in standard mode (100 kHz) or high speed mode
 *  (400 kHz).
 *
 * Parameters:
 *  mode        I2C mode
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_I2C_SPEED           - invalid I2C speed
 *
 * Example:
 *  error = i2c_init(I2C_STANDARD_MODE);
 *
 * Note:
 *  Make sure I2C0_CLK is 24000000 Hz.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_init(uint8_t mode)
{
    /* Enable I2C0 module clock. */
    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;

    /* Set selected I2C speed. */
    switch (mode)
    {
        case (I2C_STANDARD_MODE):
            I2C0_F |= I2C_F_MULT(0x00) | I2C_F_ICR(0x1F);
            break;

        case (I2C_FAST_MODE):
            I2C0_F |= I2C_F_MULT(0x01) | I2C_F_ICR(0x05);
            break;

        default:
            /* Disable I2C0 module clock. */
            SIM_SCGC4 &= ~SIM_SCGC4_I2C0_MASK;

            return ERR_I2C_SPEED;
    }

    /* Enable I2C master mode. */
    I2C0_C1 |= I2C_C1_IICEN_MASK;

    /* Enable PORTE module clock. */
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    /* Set multiplexing mode of PORTB, pin 0 and pin 1 to operate in I2C0_SCL
     * and I2C0_SDA mode. */
    PORTB_PCR(0) |= PORT_PCR_MUX(2);
    PORTB_PCR(1) |= PORT_PCR_MUX(2);

    return ERR_OK;
}


/*******************************************************************************
 * error_t i2c_transmit(uint8_t addr, uint8_t* data, uint32_t size)
 *
 * Description:
 *  Write data to I2C bus. I2C transmissions starts with the writing of the
 *  slave address and the write bit. After receiving ACK (generated by slave)
 *  the master writes one data bytes followed by the other. Valid bytes are
 *  confirmed by ACK (generated by slave), invalid bytes are indicated by NACK.
 *
 * Parameters:
 *  addr        slave address
 *  data        memory address of data to be transmitted
 *  size        number of bytes to be transmitted
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_I2C_ACK             - NACK received from slave
 *
 * Example:
 *  error = i2c_transmit(0x1A, data, sizeof(data));
 *
 * Note:
 *  The I2C module has to be initialized before calling this routine.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_transmit(uint8_t addr, uint8_t* data, uint32_t size)
{
    uint32_t cnt;

    /* Generate START signal. */
    i2c_start();

    /* Transmit slave address and write bit. */
    I2C0_D = (addr << 1) | I2C_WRITE;

    /* Wait for transfer to complete. */
    while (!(I2C0_S & I2C_S_IICIF_MASK));

    /* Clear interrupt flag. */
    I2C0_S |= I2C_S_IICIF_MASK;

    /* Check slave acknowledge. If NACK was received stop I2C transmission and
     * return an error. */
    if (I2C0_S & I2C_S_RXAK_MASK)
    {
        /* Generate STOP signal. */
        i2c_stop();

        return ERR_I2C_ACK;
    }

    /* Loop through all data bytes. */
    for (cnt = 0; cnt < size; cnt++)
    {
        /* Transmit data byte. */
        I2C0_D = data[cnt];

        /* Wait for transfer to complete. */
        while (!(I2C0_S & I2C_S_IICIF_MASK));

        /* Clear interrupt flag. */
        I2C0_S |= I2C_S_IICIF_MASK;

        /* Check slave acknowledge. If NACK was received stop I2C transmission
         * and return an error. */
        if (I2C0_S & I2C_S_RXAK_MASK)
        {
            /* Generate STOP signal. */
            i2c_stop();

            return ERR_I2C_ACK;
        }
    }

    /* Generate STOP signal. */
    i2c_stop();

    return ERR_OK;
}


/*******************************************************************************
 * error_t i2c_receive(uint8_t addr, uint8_t* data, uint32_t size)
 *
 * Description:
 *  Read data from I2C bus. I2C receptions starts the writing of the slave
 *  address and the read bit. After receiving ACK (generated by slave) the
 *  master provides a clock signal on SCL. The slave shifts data to SDA. Each
 *  received byte is confirmed by the master with ACK if an additional byte
 *  should be received or NACK if no additional byte should be received.
 *
 * Parameters:
 *  addr        slave address
 *  data        memory address of data to be received
 *  size        number of bytes to be received
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_I2C_ACK             - NACK received from slave
 *
 * Example:
 *  error = i2c_receive(0x1A, data, 8);
 *
 * Note:
 *  The I2C module has to be initialized before calling this routine.
 *
 * History:
 *  pka, 27/AUG/2015, initial code
 ******************************************************************************/
error_t i2c_receive(uint8_t addr, uint8_t* data, uint32_t size)
{
    uint32_t cnt;

    /* Generate START signal. */
    i2c_start();

    /* Transmit slave address and read bit. */
    I2C0_D = (addr << 1) | I2C_READ;

    /* Wait for transfer to complete. */
    while (!(I2C0_S & I2C_S_IICIF_MASK));

    /* Clear interrupt flag. */
    I2C0_S |= I2C_S_IICIF_MASK;

    /* Check slave acknowledge. If NACK was received stop I2C transmission and
     * return an error. */
    if (I2C0_S & I2C_S_RXAK_MASK)
    {
        /* Generate STOP signal. */
        i2c_stop();

        return ERR_I2C_ACK;
    }

    /* Check data size. The remaining data bytes determine the value of the
     * I2Cx_C1[TXAK] field. The I2Cx_C1[TXAK] field determines the value of the
     * master acknowledge. */
    if (size == 1)
    {
        /* Generate NACK on following receiving byte. */
        I2C0_C1 |= I2C_C1_TXAK_MASK;
    }
    else
    {
        /* Generate ACK on following receiving byte. */
        I2C0_C1 &= ~I2C_C1_TXAK_MASK;
    }

    /* Set mode to receive. */
    I2C0_C1 &= ~I2C_C1_TX_MASK;

    /* Dummy read. */
    data[0] = I2C0_D;

    /* Loop through all data bytes. */
    for (cnt = 0; cnt < size; cnt++)
    {
        /* Wait for transfer to complete. */
        while (!(I2C0_S & I2C_S_IICIF_MASK));

        /* Clear interrupt flag. */
        I2C0_S |= I2C_S_IICIF_MASK;

        /* Check if last loop iteration is reached. */
        if (cnt == size - 1)
        {
            /* Generate STOP signal. */
            i2c_stop();
        }
        /* Check if second last loop iteration is reached. */
        else if (cnt == size - 2)
        {
            /* Generate NACK on following receiving byte. */
            I2C0_C1 |= I2C_C1_TXAK_MASK;
        }

        /* Read received data. */
        data[cnt] = I2C0_D;

        /* Wait for transfer to complete. */
        if (I2C0_S & I2C_S_IICIF_MASK)
        {
            /* Clear interrupt flag. */
            I2C0_S |= I2C_S_IICIF_MASK;
        }
    }

    return ERR_OK;
}
