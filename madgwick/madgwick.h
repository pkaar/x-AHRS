/*******************************************************************************
 * madgwick.h
 *
 * Description:
 *  This module is used to calculate orientation relative to earth magnetic
 *  field by using Sebastian Madgwick AHRS algorithm. The algorithm uses 3-axis
 *  accelerations, angular velocities and magnetic field data for calculation
 *  the orientation.
 *
 * Functions:
 * error_t madgwick_init(float smpl_frq, float b_start, float b_end, float b_step);
 * error_t madgwick_reset(void);
 * error_t madgwick_update(vector_float_t a, vector_float_t w, vector_float_t m, quat_t* q)
 *
 * Note:
 *  See http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/ for more
 *  information.
 *
 * History:
 *  pka, 27/OCT/2015, changed data types of passed parameters
 *  smw, 19/FEB/2012, added magnetometer measurement normalization
 *  smw, 02/OCT/2011, optimized for reduced CPU load
 *  smw, 29/SEP/2011, initial code
 ******************************************************************************/

#ifndef MADGWICK_H_
#define MADGWICK_H_

#include "error.h"
#include "types.h"
#include "quat.h"

error_t madgwick_init(float smpl_frq, float b_start, float b_end, float b_step);
error_t madgwick_reset(void);
error_t madgwick_update(vector_float_t a, vector_float_t w, vector_float_t m, quat_t* q);

#endif /* MADGWICK_H_ */
