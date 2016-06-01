/*******************************************************************************
 * pit.c
 *
 * Description:
 *  This module is used to initialize Kinetis K-series periodic interrupt timer
 *  (PIT) as described below.
 *
 * Functions:
 *  void pit_init(uint8_t pit, uint32_t frq)    initialize specified PIT
 *  void pit_start(uint8_t pit)                 start specified PIT
 *  void pit_stop(uint8_t pit)                  stop specified PIT
 *  void pit_restart(uint8_t pit)               restart specifeid PIT
 *  void pit_set(uint8_t pit, uint32_t frq)     set frequency of specified PIT
 *  void PIT0_IRQHandler(void)                  PIT0 ISR
 *  void PIT1_IRQHandler(void)                  PIT1 ISR
 *  void PIT2_IRQHandler(void)                  PIT2 ISR
 *  void PIT3_IRQHandler(void)                  PIT3 ISR
 *
 * Callbacks:
 *  void pit0_callback(void)                    PIT0 callback function
 *  void pit1_callback(void)                    PIT1 callback function
 *  void pit2_callback(void)                    PIT2 callback function
 *  void pit3_callback(void)                    PIT3 callback function
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

#ifndef PIT_H_
#define PIT_H_

#include "mkl25z128xxx4.h"

#define PIT0                0
#define PIT1                1

#define PIT_DISABLE_INT     0
#define PIT_ENABLE_INT      1

/* Provided functions. */
void pit_init(uint32_t pit_idx, uint32_t frq, uint32_t interrupt);
void pit_start(uint32_t pit_idx);
void pit_stop(uint32_t pit_idx);
void pit_restart(uint32_t pit_idx);
void pit_set(uint32_t pit, uint32_t frq);

/* Required functions. */
void pit0_callback(void) __attribute__((weak));
void pit1_callback(void) __attribute__((weak));

#endif /* PIT_H_ */
