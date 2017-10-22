//
// Created by Tom Dilatush on 10/19/17.
//

#ifndef GPSCTL_SLIGHTLY_LOONY_H
#define GPSCTL_SLIGHTLY_LOONY_H

#define ARRAY_SIZE(x) (sizeof (x) / sizeof *(x))

void stackDump( void );

void *safeMalloc( size_t size );

char* concat( const char *s1, const char *s2 );

void sleep_ns( int ns );

long long currentTimeMs();

long long max_ll( long long a, long long b );

int hex2int( char c );

#endif //GPSCTL_SLIGHTLY_LOONY_H
