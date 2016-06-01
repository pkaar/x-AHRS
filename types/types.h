/*
 * types.h
 *
 *  Created on: Sep 3, 2015
 *      Author: pkaar
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_int16_t;

typedef struct {
    float x;
    float y;
    float z;
} vector_float_t;

#endif /* TYPES_H_ */
