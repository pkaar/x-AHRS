/*******************************************************************************
 * cpu.c
 *
 * Description:
 *  This module is used to initialize ARM Cortex-M0 core and provides generic
 *  high level routines.
 *
 * Functions:
 *  void enable_interrupts(void)
 *  void disable_interrupts(void)
 *  void enable_int(uint8_t interrupt)
 *  void disable_int(uint8_t interrupt)
 *  void set_int_priority(uint8_t interrupt, uint8_t priority)
 *
 * Note:
 *  This library is intended to be used for Kinetis MKL25Z128 microcontrollers.
 *
 * History:
 *  pka, 17/NOV/2015, added function 'void set_int_priority(uint8_t, uint8_t)'
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/

#include "cpu.h"

/*******************************************************************************
 * void enable_interrupts(void)
 *
 * Description:
 *  Enable all interrupts in ARM Cortx-M0 core.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  enable_interrupts();
 *
 * Note:
 *  This function is implemented as a macro.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
/* Implemented as macro in cpu.h.
void enable_interrupts(void)
{
    asm("cpsie i");
}
*/


/*******************************************************************************
 * void disable_interrupts(void)
 *
 * Description:
 *  Disable all interrupts in ARM Cortex-M0 core.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  disable_interrupts();
 *
 * Note:
 *  This function is implemented as a macro.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
/* Implemented as macro in cpu.h.
void disable_interrupts(void)
{
    asm("cpsid i");
}
*/


/*******************************************************************************
 * void enable_int(uint8_t interrupt)
 *
 * Description:
 *  Initialize nested vectored interrupt controller (NVIC) to enable the
 *  specified interrupt request (IRQ).
 *
 * Parameters:
 *  interrupt       interrupt to be enabled
 *
 * Return:
 *  none
 *
 * Example:
 *  enable_int(INT_PIT0);
 *  Enable periodic interrupt timer (PIT) channel 0 interrupt.
 *
 * Note:
 *  The function only initializes the NVIC to enable a single IRQ. Interrupts
 *  will also need to be enabled in the ARM core. This has to be done using
 *  enable_interrupts().
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
void enable_int(uint8_t interrupt)
{
    NVIC_ICPR |= (1 << ((interrupt - 16) % 32));
    NVIC_ISER |= (1 << ((interrupt - 16) % 32));
}


/*******************************************************************************
 * void disable_int(uint8_t interrupt)
 *
 * Description:
 *  Initialize NVIC to disable the specified IRQ.
 *
 * Parameters:
 *  interrupt       interrupt to be disabled
 *
 * Return:
 *  none
 *
 * Example:
 *  disable_int(INT_PIT0);
 *  Disable PIT channel 0 interrupt.
 *
 * Note:
 *  The function only initializes the NVIC to disable a single IRQ. If you want
 *  to disable all interrupts, use disable_interrupts().
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
void disable_int(uint8_t interrupt)
{
    NVIC_ICER |= (1 << ((interrupt - 16) % 32));
}


/*******************************************************************************
 * void set_int_priority(uint8_t interrupt, uint8_t priority)
 *
 * Description:
 *  Set interrupt priority for specified for specified IRQ. There are 4
 *  different priority levels, where 0 indicates the highest priority and 3 the
 *  lowest.
 *
 * Parameters:
 *  interrupt       interrupt whose priority is changed
 *  priority        interrupt priority
 *
 * Return:
 *  none
 *
 * Example:
 *  set_int_priority(PIT0, 3);
 *
 * Note:
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
void set_int_priority(uint8_t interrupt, uint8_t priority)
{
    uint8_t* priority_reg;

    /* Check interrupt priority and make sure the priority level does not exceed
     * the lowest possible level of 3. */
    if (priority > 3)
    {
        priority = 3;
    }

    /* Determine which of the NVICIPs corresponds to the interrupt number. */
    priority_reg = (uint8_t*) (((uint32_t) &NVIC_IPR0) + (interrupt - 16));

    /* Assign priority to specified IRQ. */
    *priority_reg = ((priority & 0x03) << 6);
}
