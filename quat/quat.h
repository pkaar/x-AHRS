/*******************************************************************************
 * quat.h
 *
 * Description:
 *  This module provides basic quaternion mathematics. Processed quaternions
 *  have to match the standard quaternion form q = w + xi + yj + zk.
 *
 * Functions:
 *  quat_t quat_conj(quat_t q);
 *  quat_t quat_mult(quat_t q1, quat_t q2);
 *
 * Note:
 *  Quaternions have to be in the standard quaternion form q = w + xi + yj + zk.
 *
 * History:
 *  pka, 28/OCT/2015, initial code
 ******************************************************************************/

#ifndef QUAT_H_
#define QUAT_H_

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

quat_t quat_conj(quat_t q);
quat_t quat_mult(quat_t q1, quat_t q2);

#endif /* QUAT_H_ */
