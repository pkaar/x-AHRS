/*******************************************************************************
 * cpu.c
 *
 * Description:
 *  This module is used to initialize ARM Cortex-M0 core and provides generic
 *  high level routines.
 *
 * Functions:
 *  void enable_interrupts(void)            enable all interrupts in ARM core
 *  void disable_interrupts(void)           disable all interrupts in ARM core
 *  void enable_int(uint8_t interrupt)      enable specified interrupt
 *  void disable_int(uint8_t interrupt)     disable specified interrupt
 *
 * Note:
 *  This library is intended to be used for Kinetis MKL25Z4 microcontrollers.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/

#ifndef CPU_H_
#define CPU_H_

#include "mkl25z128xxx4.h"

#define enable_interrupts()                                                 \
    asm("cpsie i")

#define disable_interrupts()                                                \
    asm("cpsid i")

#define critical_init()                                                     \
    uint8_t primask

#define critical_enter()                                                    \
    asm("mrs r0, primask");                                                 \
    asm("cpsid i");                                                         \
    asm("strb r0, %0\n" : "=m" (primask))

#define critical_exit()                                                     \
    asm("ldrb r0, %0\n" :: "m" (primask));                                  \
    asm("msr primask, r0;")

/* Provides functions. */
void enable_int(uint8_t interrupt);
void disable_int(uint8_t interrupt);
void set_int_priority(uint8_t interrupt, uint8_t priority);

#endif /* ARM_H_ */
