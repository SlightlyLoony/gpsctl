//
// Created by Tom Dilatush on 10/19/17.
//

#ifndef GPSCTL_SLIGHTLY_LOONY_H
#define GPSCTL_SLIGHTLY_LOONY_H

#define ARRAY_SIZE(x) (sizeof (x) / sizeof *(x))

typedef unsigned char byte;

typedef struct sl_buffer {
    byte *buffer;  // the actual buffer
    size_t size;            // the number of bytes in the buffer
    bool allocated;         // true if the buffer was allocated with malloc et al
} sl_buffer;

sl_buffer allocateSlBuffer( size_t size );

void freeSlBuffer( sl_buffer buffer );

void printSlBuffer( sl_buffer buffer );

void stackDump( void );

void *safeMalloc( size_t size );

char* concat( const char *s1, const char *s2 );

void sleep_ns( int ns );

long long currentTimeMs();

long long max_ll( long long a, long long b );

int hex2int( char c );

#endif //GPSCTL_SLIGHTLY_LOONY_H
