/*******************************************************************************
 * pit.c
 *
 * Description:
 *  This module is used to initialize Kinetis K-series periodic interrupt timer
 *  (PIT) as described below.
 *
 * Functions:
 *  void pit_init(uint32_t pit, uint32_t frq)   initialize specified PIT
 *  void pit_start(uint32_t pit)                start specified PIT
 *  void pit_stop(uint32_t pit)                 stop specified PIT
 *  void pit_restart(uint32_t pit)              restart specifeid PIT
 *  void pit_set(uint32_t pit, uint32_t frq)    set frequency of specified PIT
 *  void PIT0_IRQHandler(void)                  PIT0 ISR
 *  void PIT1_IRQHandler(void)                  PIT1 ISR
 *
 * Callbacks:
 *  void pit0_callback(void)                    PIT0 callback function
 *  void pit1_callback(void)                    PIT1 callback function
 *
 * Note:
 *  This library is intended to be used for the Kinetis K21 microcontroller.
 *
 * History:
 *  pka, 11/SEP/2014, - added function pit_set(...)
 *  pka, 27/AUG/2014, - removed automatic start of PIT module in function
 *                      pit_init(...)
 *                    - added function pit_start(...)
 *                    - added function pit_stop(...)
 *                    - added function pit_restart(...)
 *  pka, 25/MAR/2014, - initial code
 ******************************************************************************/

#include "pit.h"

#include "clock.h"
#include "cpu.h"

/*******************************************************************************
 * void pit_init(uint32_t pit, uint32_t frq, uint32_t interrupt)
 *
 * Description:
 *  Initialize specified periodic interrupt timer (PIT) with specified
 *  repetition frequency.
 *
 * Parameters:
 *  pit             PIT channel to be initialized (PIT0, PIT1, PIT2, PIT3)
 *  frq             PIT repetition frequency in Hz
 *  interrupt       PIT interrupt generation (ENABLE_INT, DISABLE_INT)
 *
 * Return:
 *  none
 *
 * Example:
 *  pit_init(PIT0, 1000000, ENABLE_INT);
 *  Initialize PIT0 with a repetition frequency of 1000000 Hz. After the
 *  expiration of a period, PIT0 interrupt service routine calls PIT0 callback
 *  function.
 *
 * Notes:
 *  PIT module is clocked by bus clock.
 *
 * History:
 *  pka, 27/AUG/2014, removed automatic start of PIT module
 *  pka, 25/MAR/2014, initial code
*******************************************************************************/
void pit_init(uint32_t pit, uint32_t frq, uint32_t interrupt)
{
    /* Enable PIT module clock. */
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

    /* Enable PIT module. PIT_MCR[MDIS] bit MUST be enabled before any other
     * bit manipulation is done. */
    PIT_MCR &= ~PIT_MCR_MDIS_MASK;

    /* Set timer start value. The timer counts down until it reaches 0. When 0
     * is reached an interrupt will be generated. */
    PIT_LDVAL(pit) = PIT_CLK / frq - 1;

    /* Check if interrupts are enabled. */
    if (interrupt)
    {
        /* Enable timer interrupt. */
        PIT_TCTRL(pit) |= PIT_TCTRL_TIE_MASK;

        /* Enable PIT interrupt. */
        enable_int(INT_PIT);
    }
}


/*******************************************************************************
 * void pit_start(uint32_t pit)
 *
 * Description:
 *  Start specified periodic interrupt timer (PIT).
 *
 * Parameters:
 *  pit             PIT channel to be started
 *
 * Return:
 *  none
 *
 * Example:
 *  pit_start(PIT0);
 *  Start PIT0 module. After the expiration of a period, PIT0 interrupt service
 *  routine calls PIT0 callback function.
 *
 * Notes:
 *  Start only PIT modules which have been properly initialized. Starting a PIT
 *  module which has not been initialized may cause unexpected errors.
 *  Starting a PIT module which has been stopped earlier, causes the timer value
 *  to be set to its default starting value.
 *
 * History:
 *  pka, 27/AUG/2014, initial code
*******************************************************************************/
void pit_start(uint32_t pit)
{
    PIT_TCTRL(pit) |= PIT_TCTRL_TEN_MASK;
}


/*******************************************************************************
 * void pit_stop(uint32_t pit)
 *
 * Description:
 *  Stop specified periodic interrupt timer (PIT).
 *
 * Parameters:
 *  pit             PIT channel to be stopped
 *
 * Return:
 *  none
 *
 * Example:
 *  pit_stop(PIT0);
 *  Stop PIT0 module.
 *
 * Notes:
 *  Stop only PIT modules which have been properly initialized. Stopping a PIT
 *  module which has not been initialized may cause unexpected errors.
 *
 * History:
 *  pka, 27/AUG/2014, initial code
*******************************************************************************/
void pit_stop(uint32_t pit)
{
    PIT_TCTRL(pit) &= ~PIT_TCTRL_TEN_MASK;
}


/*******************************************************************************
 * void pit_restart(uint32_t pit)
 *
 * Description:
 *  Restart specified periodic interrupt timer (PIT).
 *
 * Parameters:
 *  pit             PIT channel to be stopped
 *
 * Return:
 *  none
 *
 * Example:
 *  pit_restart(PIT0);
 *  Restart PIT0 module.
 *
 * Notes:
 *  Restart only PIT modules which have been properly initialized. Restarting a
 *  PIT module which has not been initialized may cause unexpected errors.
 *  Restarting a PIT module causes the timer value to be set to its default
 *  starting value.
 *
 * History:
 *  pka, 27/AUG/2014, initial code
*******************************************************************************/
void pit_restart(uint32_t pit)
{
    PIT_TCTRL(pit) &= ~PIT_TCTRL_TEN_MASK;
    PIT_TCTRL(pit) |= PIT_TCTRL_TEN_MASK;
}


/*******************************************************************************
 * void pit_set(uint32_t pit, uint32_t frq)
 *
 * Description:
 *  Set repetition frequency of specified periodic interrupt timer (PIT).
 *
 * Parameters:
 *  pit             PIT channel to be set
 *  frq             PIT repetition frequency in Hz
 *
 * Return:
 *  actual PIT repetition frequency
 *
 * Example:
 *  pit0_frq = pit_set(PIT0, 1000000);
 *  Set PIT0 repetition frequency and get actual frequency value.
 *
 * Notes:
 *  According to the specified PIT frequency the timer start value is
 *  calculated. The calculation of this start value contains a division. Since
 *  the start value is loaded to a 32-bit register this may cause rounding
 *  errors and subsequently a wrong PIT repetition frequency.
 *
 * History:
 *  pka, 11/SEP/2014, initial code
*******************************************************************************/
void pit_set(uint32_t pit, uint32_t frq)
{
    /* Set timer start value. */
    PIT_LDVAL(pit) = PIT_CLK / frq - 1;
}


/*******************************************************************************
 * void PIT0_IRQHandler(void)
 *
 * Description:
 *  Interrupt service routine (ISR) when PIT0 reaches 0 and PIT0 interrupt is
 *  enabled.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Notes:
 *  Use pit0_callback() to add code to this ISR.
 *
 * History:
 *  pka, 25/MAR/2014, initial code
 ******************************************************************************/
void PIT_IRQHandler(void)
{
    /* Check if PIT0 triggered interrupt. */
    if (PIT_TFLG0 & PIT_TFLG_TIF_MASK)
    {
        /* Clear timer interrupt flag by setting PIT_TFLG0[TIF] bit. */
        PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

        pit0_callback();
    }

    /* Check if PIT1 triggered interrupt. */
    else if (PIT_TFLG1 & PIT_TFLG_TIF_MASK)
    {
        /* Clear timer interrupt flag by setting PIT_TFLG0[TIF] bit. */
        PIT_TFLG1 |= PIT_TFLG_TIF_MASK;

        pit1_callback();
    }
}
