//
// Created by Tom Dilatush on 10/24/17.
//

#ifndef GPSCTL_SL_BUFFER_H
#define GPSCTL_SL_BUFFER_H

#include <memory.h>
#include <stdlib.h>
#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdarg.h>
#include <fcntl.h>
#include <stdint.h>
#include <termios.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>
#include <ctype.h>
#include "sl_general.h"

struct slBuffer;
typedef struct slBuffer slBuffer;

slBuffer* init_slBuffer( size_t size, Endiness endiness, ... );

slBuffer* create_slBuffer( size_t size, Endiness );

#define init_slBuffer( Endiness, ... ) init_slBuffer_int(  NUMARGS(__VA_ARGS__), Endiness, __VA_ARGS__ )
slBuffer* init_slBuffer_int( size_t size, Endiness, ... );

slBuffer* copy_slBuffer( size_t size, Endiness, byte *buffer );

void print_slBuffer( slBuffer *buffer );

size_t size_slBuffer( slBuffer *buffer );

byte* buffer_slBuffer( slBuffer *buffer );

void put_uint32_slBuffer( slBuffer *buffer, size_t offset, uint32_t value );
uint32_t get_uint32_slBuffer( slBuffer *buffer, size_t offset );
void put_int32_slBuffer( slBuffer *buffer, size_t offset, int32_t value );
int32_t get_int32_slBuffer( slBuffer *buffer, size_t offset );
void put_uint16_slBuffer( slBuffer *buffer, size_t offset, uint16_t value );
uint16_t get_uint16_slBuffer( slBuffer *buffer, size_t offset );
void put_int16_slBuffer( slBuffer *buffer, size_t offset, int16_t value );
int16_t get_int16_slBuffer( slBuffer *buffer, size_t offset );
void put_uint8_slBuffer( slBuffer *buffer, size_t offset, uint8_t value );
uint8_t get_uint8_slBuffer( slBuffer *buffer, size_t offset );
void put_int8_slBuffer( slBuffer *buffer, size_t offset, int8_t value );
int8_t get_int8_slBuffer( slBuffer *buffer, size_t offset );

#endif //GPSCTL_SL_BUFFER_H
