/*******************************************************************************
 * startup.c
 *
 * Description:
 *  Startup code for initializing Kinetis MKL25Z128 after hardware reset.
 *
 * Note:
 *  This library is intended to be used for Kinetis MKL25Z4 microcontrollers.
 *
 * History:
 *  pka, 01/SEP/2014, added disabling of all exceptions before starting main()
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/

#include "mkl25z128xxx4.h"

#include "cpu.h"

extern unsigned long __data_load;
extern unsigned long __data_start;
extern unsigned long __data_end;
extern unsigned long __bss_start;
extern unsigned long __bss_end;

#if defined (__cplusplus)
extern "C" {
#endif

extern int main(void);
extern void __StackTop(void);

#if defined (__cplusplus)
}
#endif

void Default_Handler(void);
void Reset_Handler(void);

/* Weak handler definitions point to Default_Handler if not implemented. */
void NMI_Handler() __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler() __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias("Default_Handler")));
void DMA0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void MCM_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTFL_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PMC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LLW_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void SPI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void ADC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void CMP0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTM0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTM1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTM2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_Seconds_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USBOTG_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DAC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TSI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void MCG_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LPTimer_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));

/* Interrupt vector table (see reference manual section 3.2.2, p. 69ff). */
__attribute__ ((section(".vector_table")))
void (* const InterruptVectorTable[])(void) = {
    &__StackTop,                    /* Initial Stack Pointer */
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,
    DMA0_IRQHandler,
    DMA1_IRQHandler,
    DMA2_IRQHandler,
    DMA3_IRQHandler,
    MCM_IRQHandler,
    FTFL_IRQHandler,
    PMC_IRQHandler,
    LLW_IRQHandler,
    I2C0_IRQHandler,
    I2C1_IRQHandler,
    SPI0_IRQHandler,
    SPI1_IRQHandler,
    UART0_IRQHandler,
    UART1_IRQHandler,
    UART2_IRQHandler,
    ADC0_IRQHandler,
    CMP0_IRQHandler,
    FTM0_IRQHandler,
    FTM1_IRQHandler,
    FTM2_IRQHandler,
    RTC_Alarm_IRQHandler,
    RTC_Seconds_IRQHandler,
    PIT_IRQHandler,
    Default_Handler,
    USBOTG_IRQHandler,
    DAC0_IRQHandler,
    TSI0_IRQHandler,
    MCG_IRQHandler,
    LPTimer_IRQHandler,
    Default_Handler,
    PORTA_IRQHandler,
    PORTD_IRQHandler
};

/* Flash configuration field. Program flash memory contains a 16-byte flash
 * configuration filed that stores default protection settings and security
 * information that allows the MCU to restrict access to the FTFE module (see
 * reference manual chapter 29, p. 423ff) */
__attribute__ ((section(".flash_config")))
const uint8_t FlashConfigurationField[] = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFE,
    0xFF,
    0xFF,
    0xFF
};


/*******************************************************************************
 * void Default_Handler(void)
 *
 * Description:
 *  Default Interrupt Handler.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
void Default_Handler(void)
{
    __asm("BKPT");
}


/*******************************************************************************
 * void Reset_Handler(void)
 *
 * Description:
 *  Reset ISR handler to perform all necessary steps after processor reset.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/
void Reset_Handler(void)
{
    unsigned char *source;
    unsigned char *destination;

    /* Disable COP module (Watchdog) because it may resets the core before
     * entering main(). */
    SIM_COPC = 0x0000;

    /* Copy data values from ROM to RAM. */
    source = (unsigned char *) &__data_load;
    destination = (unsigned char *) &__data_start;

    while (destination < (unsigned char*) &__data_end)
    {
        *(destination++) = *(source++);
    }

    /* Clear .bss section. */
    source = (unsigned char *) &__bss_start;
    destination = (unsigned char *) &__bss_end;

    while (source < destination )
    {
        *source++ = 0;
    }

    /* Disable activation of all exceptions with configurable priority, since
     * exception handling is activated by default in ARM Cortex-M0 cores. */
    disable_interrupts();

    /* Call main(). */
    main();

    while(1);
}
