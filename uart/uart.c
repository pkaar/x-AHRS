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

#include "uart.h"

#include "cpu.h"
#include "clock.h"

/*******************************************************************************
 * error_t uart_init(uint32_t baud)
 *
 * Description:
 *  Initialize UART0 module with 8 data bits, no parity bit and 1 stop bit. The
 *  module uses PTA1 and PTA2 as RX and TX lines and can be configured to work
 *  with user defined baud rates.
 *
 * Parameters:
 *  baud        UART baud rate
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = uart_init(115200);
 *
 * Note:
 *  This method enables UART0 interrupts in ARM Cortex-M0 core.
 *
 * History:
 *  pka, 30/SEP/2015, initial code
 ******************************************************************************/
error_t uart_init(uint32_t baud)
{
    uint8_t tmp_reg;        /* temporary register */
    uint16_t sbr;           /* baud rate module clock divider */

    /* Calculate UART baud rate clock divider value. */
    sbr = (uint16_t) ((uint32_t) (UART0_CLK + 4 * baud) / (8 * baud));

    /* Check for valid UART clock divider value. */
    if (sbr > 0x1FFF)
    {
        return ERR_UART_SPEED;
    }

    /* Enable port A clock. */
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

    /* Set multiplexing mode of PORTA, pin 1 and pin 2 to work as UART0_RX and
     * UART0_TX. */
    PORTA_PCR(1) |= PORT_PCR_MUX(2);
    PORTA_PCR(2) |= PORT_PCR_MUX(2);

    /* Enable UART0 module clock. */
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

    /* Enable UART0 receiver and transmitter clock. */
    SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1) | SIM_SOPT2_PLLFLLSEL_MASK;

    /* Make sure that transmitter and receiver are disabled while changing UART
     * settings. */
    UART0_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);

    /* Configure UART for 8-bit mode with no parity bit. */
    UART0_C1 &= ~(UART0_C1_M_MASK | UART0_C1_PE_MASK);

    /* Set one stop bit. */
    UART0_BDH &= ~(UART0_BDH_SBNS_MASK);

    /* Set UART over sampling ratio to 8. Adjusting the oversampling ratio
     * allows to use higher baud rates which would not be possible due to
     * rounding errors. */
    UART0_C4 = UART0_C4_OSR(0x07);

    /* Get current value of UART_BDH register without reading SBR bits. */
    tmp_reg = UART0_BDH & ~UART0_BDH_SBR_MASK;

    /* Set UART baud rate module clock divider in UART_BDH register. The saved
     * value of UART_BDH register as well as the 5 most significant bits of sbr
     * have to be written to this register. To get the 5 most significant bits a
     * right shift of 8 digits is used. */
    UART0_BDH = tmp_reg | UART0_BDH_SBR(sbr >> 8);

    /* Set UART baud rate module clock divider in UART_BDL register. The 8 least
     * significant bits of sbr have to be written to this register. */
    UART0_BDL = (uint8_t) sbr;

    /* Enable receiver full interrupt. An interrupt will be generated when Rx
     * watermark is reached or exceeded. */
    UART0_C2 |= UART0_C2_RIE_MASK;

    /* Enable UART0 interrupts. */
    enable_int(INT_UART0);

    /* Enable transmitter and receiver. */
    UART0_C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;

    return ERR_OK;
}


/*******************************************************************************
 * error_t uart_transmit(uint8_t* data, uint16_t size)
 *
 * Description:
 *  Write data to UART interface. Data is transmitted byte wise until the
 *  specified amount of data is written.
 *
 * Parameters:
 *  data        memory address of data to be transmitted
 *  size        number of bytes to be transmitted
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = uart_transmit(data, sizeof(data));
 *
 * Note:
 *  The UART module has to be initialized before calling this routine. This
 *  routine is a blocking call.
 *
 * History:
 *  pka, 30/SEP/2015, initial code
 ******************************************************************************/
error_t uart_transmit(uint8_t* data, uint16_t size)
{
    uint16_t cnt;

    /* Loop through all data bytes. */
    for (cnt = 0; cnt < size; cnt++)
    {
        /* Wait until byte has been transmitted. */
        while (!(UART0_S1 & UART0_S1_TDRE_MASK));

        /* Send data byte. */
        UART0_D = (uint8_t) *data++;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t uart_receive(uint8_t* data, uint16_t size)
 *
 * Description:
 *  Read data to UART interface. Data is received byte wise until the
 *  specified amount of data is read.
 *
 * Parameters:
 *  data        memory address of data to be received
 *  size        number of bytes to be received
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = uart_receive(data, sizeof(data));
 *
 * Note:
 *  The UART module has to be initialized before calling this routine. This
 *  routine is a blocking call.
 *
 * History:
 *  pka, 30/SEP/2015, initial code
 ******************************************************************************/
error_t uart_receive(uint8_t* data, uint16_t size)
{
    uint16_t cnt;

    /* Loop through all data bytes. */
    for (cnt = 0; cnt < size; cnt++)
    {
        /* Wait until byte has been received. */
        while (!(UART0_S1 & UART0_S1_RDRF_MASK));

        *data++ = UART0_D;
    }

    return ERR_OK;
}


/*******************************************************************************
 * void UART0_IRQHandler(void)
 *
 * Description:
 *  UART interrupt service routine.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *
 * Note:
 *
 * History:
 *  pka, 30/SEP/2015, initial code
 ******************************************************************************/
void UART0_IRQHandler(void)
{
    if (UART0_S1 & UART0_S1_RDRF_MASK)
    {
        uart_rx_callback();
    }
}
