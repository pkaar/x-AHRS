/*******************************************************************************
 * quat.c
 *
 * Description:
 *  This module provides basic quaternion mathematics. Processed quaternions
 *  have to match the standard quaternion form q = w + xi + yj + zk.
 *
 * Functions:
 *  void quat_conj(quat_t q, quat_t* q_conj);
 *  void quat_mult(quat_t q1, quat_t q2, quat_t* q);
 *
 * Note:
 *  Quaternions have to be in the standard quaternion form q = w + xi + yj + zk.
 *
 * History:
 *  pka, 28/OCT/2015, initial code
 ******************************************************************************/

#include "quat.h"

/*******************************************************************************
 * quat_t quat_conj(quat_t q)
 *
 * Description:
 *  This method calculates the conjugate of the specified quaternion.
 *
 * Parameters:
 *  q           quaternion to be conjugated
 *
 * Return:
 *  q           conjugated quaternion
 *
 * Example:
 *  q = quat_conj(q);
 *
 * Note:
 *  The quaternion to be conjugated has to be in the standard quaternion form
 *  q = w + xi + yj + zk.
 *
 * History:
 *  pka, 28/OCT/2015, initial code
 ******************************************************************************/
quat_t quat_conj(quat_t q)
{
    q.x = (-q.x);
    q.y = (-q.y);
    q.z = (-q.z);

    return q;
}


/*******************************************************************************
 * quat_t quat_mult(quat_t q1, quat_t q2)
 *
 * Description:
 *  This method calculates the product of quaternion 1 and quaternion 2. Where
 *  the sequence of multiplication is according to the order of passed
 *  parameters (q = q1 * q2).
 *
 * Parameters:
 *  q1          quaternion 1
 *  q2          quaternion 2
 *
 * Return:
 *  q           product of quaternion 1 and quaternion 2
 *
 * Example:
 *  q = quat_mult(q1, q2);
 *
 * Note:
 *  The quaternions to be multiplied have to be in the standard quaternion form
 *  q = w + xi + yj + zk.
 *
 * History:
 *  pka, 28/OCT/2015, initial code
 ******************************************************************************/
quat_t quat_mult(quat_t q1, quat_t q2)
{
    quat_t q;

    q.w  = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q.x  = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q.y  = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q.z  = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

    return q;
}
