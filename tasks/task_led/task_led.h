/*******************************************************************************
 * task_led.h
 *
 * Description:
 *  This module initializes LED functionality to indicate device states. The
 *  module uses standard GPIOs to control LEDs. A low priority task performs
 *  LED control according the current task state.
 *
 * Functions:
 *  error_t task_led_init(void);
 *  error_t task_led_state(uint8_t state);
 *
 * Note:
 *  This module was intended to be used on Kinetis MKL25Z128 microcontrollers.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/

#ifndef TASK_LED_H_
#define TASK_LED_H_

#include "error.h"
#include "mkl25z128xxx4.h"

#define TASK_LED_STATE_BLINK        0x00
#define TASK_LED_STATE_PAUSE        0x01

error_t task_led_init(void);
error_t task_led_state(uint8_t state);

#endif /* TASK_LED_H_ */
