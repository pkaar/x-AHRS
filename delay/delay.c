/*******************************************************************************
 * delay.c
 *
 * Description:
 *  This module provides software delay functionality. For achieving desired
 *  delays, program execution is blocked for the specified time. This module
 *  does not provide precise timing.
 *
 * Functions:
 *  void delay_ms(uint32_t delay)
 *
 * Note:
 *  This module is intended to be used on Freescale Kinetis MK22FN512xxx12 and
 *  Kinetis MKL25Z128xxx4 processors.
 *
 * History:
 *  pka, 28/APR/2016, initial code
 ******************************************************************************/

#include "delay.h"

#include "clock.h"

#define MIN_CLOCK_FRQ   10000

#if (CORE_CLK < MIN_CLOCK_FRQ)
#error Error: Core frequency deceeds minimum required value.
#endif

/*******************************************************************************
 * void delay_ms(uint32_t delay)
 *
 * Description:
 *  Blocking delay routine. Call this function to block program execution for
 *  the specified amount of milliseconds. The controller stays in a busy loop
 *  until the delay is reached.
 *
 * Parameters:
 *  delay       blocking time
 *
 * Return:
 *  none
 *
 * Example:
 *  delay_ms(100);
 *
 * Note:
 *  This module blocks program execution. Do not use this function for precise
 *  timings, since timing error is arround 0.5 %.
 *
 * History:
 *  pka, 28/APR/2016, initial code
 ******************************************************************************/
__attribute__((optimize("O0"))) void delay_ms(uint32_t delay)
{
    uint32_t loop_count;    /* number of loops to achieve specified delay */
    uint32_t reg;           /* register to store loop count */

    loop_count = delay * (CORE_CLK / 10000);

    asm volatile(
            ".syntax unified    \n\t"
            "mov %0, %1         \n\t"   /* load loop count */
            "loop:              \n\t"
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "nop                \n\t"   /* 1 cycle */
            "subs %0, #1        \n\t"   /* 1 cycle */
            "bne loop           \n\t"   /* 2 cycles if taken, 1 cycle if not */
            ".syntax divided    \n\t"
            : "=&r" (reg)
            : "r" (loop_count)
    );
}
