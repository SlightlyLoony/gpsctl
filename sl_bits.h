//
// Created by Tom Dilatush on 10/24/17.
//

#ifndef GPSCTL_SL_BITS_H
#define GPSCTL_SL_BITS_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

uint64_t getBitField_slBits( uint64_t value, uint64_t mask );
bool isBitSet_slBits( uint64_t value, int bitNum );
uint64_t setBit_slBits( uint64_t value, int bitNum, bool state );

#endif //GPSCTL_SL_BITS_H
