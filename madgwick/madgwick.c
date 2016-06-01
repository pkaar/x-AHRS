/*******************************************************************************
 * madgwick.c
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

#include "madgwick.h"

#include <math.h>

volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

volatile float smpl_period;

volatile float beta_start;
volatile float beta_end;
volatile float beta_step;

volatile float beta;


/*******************************************************************************
 * float inv_sqrt(float x)
 *
 * Description:
 *  Calculate inverse square root of specified 32-bit floating point value
 *  using fast inverse square root algorithm.
 *
 * Parameters:
 *  x           32-bit floating point value
 *
 * Return:
 *  y           inverse square root of specified value
 *
 * Example:
 *  inv_sqrt(x);
 *
 * Note:
 *  See https://en.wikipedia.org/wiki/Fast_inverse_square_root for further
 *  information. This function is not available outside of this module.
 *
 * History:
 *  pka, 27/OCT/2015, initial code
 ******************************************************************************/
float inv_sqrt(float x)
{
    int i;

    float x_half;

    x_half = 0.5f * x;

    /* Convert bits to integer. */
    i = *(int*) &x;

    /* Make an initial guess for Newton-Raphson approximation. */
    i = 0x5f3759df - (i >> 1);

    /* Convert bits back to float. */
    x = *(float*) &i;

    /* Perform two Newton-Raphson iteration steps. */
    x = x * (1.5f - x_half * x * x);
    x = x * (1.5f - x_half * x * x);

    return x;
}


/*******************************************************************************
 * error_t madgwick_init(float smpl_frq, float b_start, float b_end, float b_step)
 *
 * Description:
 *  Initialize Madgwick AHRS algorithm and set algorithm gain to default values.
 *  Possible errors are reported to the calling function.
 *
 * Parameters:
 *  smpl_frq    sampling frequency of AHRS data
 *  b_start     algorithm gain starting value
 *  b_end       algorithm gain end value
 *  b_step      algorithm gain decrement step size
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - invalid parameters
 *
 * Example:
 *  error = madgwick_init(250.0f, 5.0f, 0.01f, 0.01f);
 *
 * Note:
 *
 * History:
 *  pka, 27/OCT/2015, initial code
 ******************************************************************************/
error_t madgwick_init(float smpl_frq, float b_start, float b_end, float b_step)
{
    /* Check for valid parameters. */
    if ((smpl_frq < 0.0f) || (b_start < 0.0f) || (b_end < 0.0f) ||
            (b_step < 0.0f))
    {
        return ERR_FAIL;
    }

    /* Check for valid beta settings. */
    if (b_start < b_end)
    {
        return ERR_FAIL;
    }

    /* Calculate sampling period. When calculating orientation this saves some
     * time, since floating point divisions can be prevented. */
    smpl_period = 1.0f / smpl_frq;

    beta_start = b_start;
    beta_end = b_end;
    beta_step = b_step;

    beta = beta_start;

    return ERR_OK;
}

/*******************************************************************************
 * error_t madgwick_reset(void)
 *
 * Description:
 *  Reset Madgwick AHRS algorithm and set orientation and algorithm gain to
 *  default values. Possible errors are reported to the calling function.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = madgwick_reset();
 *
 * Note:
 *
 * History:
 *  pka, 27/OCT/2015, initial code
 ******************************************************************************/
error_t madgwick_reset(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    beta = beta_start;

    return ERR_OK;
}


/*******************************************************************************
 * error_t madgwick_update(vector_float_t a, vector_float_t w, vector_float_t m)
 *
 * Description:
 *  Calculate orientation using acceleration, angular velocity and magnetic
 *  field data. Acceleration and magnetic field data is passed to the method
 *  is user specific units. Angular velocity has to be passed in rad/s. Possible
 *  errors are reported to the calling function.
 *
 * Parameters:
 *  a           acceleration vector
 *  w           angular velocity vector
 *  m           magnetic field vector
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - invalid magnetometer data
 *
 * Example:
 *  error = madgwick_update(acceleration, angular_velocity, magnetic_field);
 *
 * Note:
 *  Passing angular velocity in units different to rad/s results in incorrect
 *  orientations. The algorithm gain is automatically decremented each time
 *  this method is called.
 *
 * History:
 *  pka, 27/OCT/2015, changed data types of passed parameters
 *  smw, 19/FEB/2012, added magnetometer measurement normalization
 *  smw, 02/OCT/2011, optimized for reduced CPU load
 *  smw, 29/SEP/2011, initial code
 ******************************************************************************/
error_t madgwick_update(vector_float_t a, vector_float_t w, vector_float_t m, quat_t* q)
{
    float ax, ay, az;
    float wx, wy, wz;
    float mx, my, mz;

    float recip_norm;
    float s0, s1, s2, s3;
    float dq1, dq2, dq3, dq4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    ax = a.x;
    ay = a.y;
    az = a.z;

    wx = w.x;
    wy = w.y;
    wz = w.z;

    mx = m.x;
    my = m.y;
    mz = m.z;

    /* Check for valid magnetometer data. */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        return ERR_FAIL;
    }

    /* Check if beta has reached its specified end value or if it has to be
     * decremented. */
    if (beta > beta_end)
    {
        /* Decrement beta only if it does not fall below the specified end
         * value. */
        if ((beta - beta_step) > beta_end)
        {
            beta -= beta_step;
        }
        else
        {
            beta = beta_end;
        }
    }

    /* Calculate quaternion rate of change of from angular velocity. */
    dq1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq2 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq3 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq4 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    /* Calculate feedback only if accelerometer measurement is valid. This
     * prevents NaNs in acceleration normalization. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalize accelerometer measurement. */
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalize magnetometer measurement. */
        recip_norm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        /* Auxiliary variables to avoid repeated arithmetic and therefore
         * improve performance. */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* Reference direction of earth magnetic field. */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step. */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalize step magnitude. */
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step. */
        dq1 -= beta * s0;
        dq2 -= beta * s1;
        dq3 -= beta * s2;
        dq4 -= beta * s3;
    }

    /* Integrate quaternion rate of change to get quaternion describing the
     * current orientation. */
    q0 += dq1 * smpl_period;
    q1 += dq2 * smpl_period;
    q2 += dq3 * smpl_period;
    q3 += dq4 * smpl_period;

    /* Normalize quaternion. */
    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

    /* Write quaternion to memory address. */
    q->w = q0;
    q->x = q1;
    q->y = q2;
    q->z = q3;

    /* Calculate conjugate to get orientation relative to earth. */
    *q = quat_conj(*q);

    return ERR_OK;
}
