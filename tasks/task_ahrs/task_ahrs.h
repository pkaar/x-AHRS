/*
 * task_ahrs.h
 *
 *  Created on: Oct 5, 2015
 *      Author: pkaar
 */

#ifndef TASK_AHRS_H_
#define TASK_AHRS_H_

#include "error.h"
#include "mkl25z128xxx4.h"

#define TASK_AHRS_STATE_START       0x00
#define TASK_AHRS_STATE_PAUSE       0x01

error_t task_ahrs_init(void);
error_t task_ahrs_state(uint8_t state);

#endif /* TASK_AHRS_H_ */
