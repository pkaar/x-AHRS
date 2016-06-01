/*******************************************************************************
 * clock.c
 *
 * Description:
 *  This module is used to initialize Kinetis KL-series clocks as described
 *  below.
 *
 * Functions:
 *  void clock_init(void)       initialize multipurpose clock generator
 *  void clock_output(void)     set port C, pin 3 as Flash clock output
 *
 * Note:
 *  This library is intended to be used for Kinetis MKL25Z4 microcontrollers.
 *
 * History:
 *  pka, 31/AUG/2014, initial code
 ******************************************************************************/

#ifndef CLOCK_H_
#define CLOCK_H_

#include "mkl25z128xxx4.h"

#define CORE_CLK        48000000UL
#define SYSTEM_CLK      48000000UL
#define BUS_CLK         24000000UL
#define FLASH_CLK       24000000UL

#define I2C0_CLK        BUS_CLK
#define I2C1_CLK        BUS_CLK
#define PIT_CLK         BUS_CLK
#define SPI0_CLK        BUS_CLK
#define SPI1_CLK        SYSTEM_CLK
#define TPM_CLK         BUS_CLK
#define UART0_CLK       BUS_CLK
#define UART1_CLK       BUS_CLK
#define UART2_CLK       BUS_CLK

/* Provided functions. */
void clock_init(void);
void clock_output(void);

#endif /* CLOCK_H_ */
