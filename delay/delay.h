/*******************************************************************************
 * delay.h
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

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>

void delay_ms(uint32_t delay);

#endif /* DELAY_H_ */
