//
// Created by Tom Dilatush on 10/17/17.
//

#define _XOPEN_SOURCE
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <getopt.h>
#include <time.h>

/*
Simple program to read configuration of U-Blox GPS serial port.
*/

// TODO: get remote debugging working
// TODO: investigate stationary mode, add option to properly put it in that mode
// TODO: try turning NMEA data off before changing baud rates, then back on afterwards
// TODO: add function to save configuration


/////  begin ARGP stuff

const char *gpsctl_version = "gpscfg 0.1\n";
const char *gpsctl_doc     = "gpscfg - control and configure U-Blox GPS on Raspberry Pi 3 port serial0\n";

typedef struct ubxMsg {
    size_t size;
    unsigned char* msg;
    int noMsg;
    int valid;
    int class;
    int id;
    size_t bodySize;
    unsigned char* body;
} UBXMsg;

const struct timespec u50 = { 0, 50000 };
const struct timespec u10 = { 0, 10000 };

unsigned char save[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};

UBXMsg createUbxMsg( unsigned char class, unsigned char id, unsigned char* body, size_t bodySize ) {
    UBXMsg result;
    result.msg = calloc((bodySize + 8), 1 );
    result.msg[0] = 0xB5;
    result.msg[1] = 0x62;
    result.msg[2] = class;
    result.msg[3] = id;
    result.msg[4] = (unsigned char) (bodySize & 0xFF);
    result.msg[5] = (unsigned char) (bodySize >> 8);
    memcpy( result.msg + 6, body, bodySize );

    // calculate Fletcher checksum for this message...
    unsigned char ck_a = 0;
    unsigned char ck_b = 0;
    int i;
    for( i = 2; i < bodySize + 6; i++ ) {
        ck_a += result.msg[i];
        ck_b += ck_a;
    }
    result.msg[bodySize + 6] = ck_a;
    result.msg[bodySize + 7] = ck_b;

    result.size = bodySize + 8;
    result.bodySize = bodySize;
    result.id = id;
    result.class = class;
    result.valid = 1;
    result.noMsg = 0;
    result.body = body;
    return result;
}

UBXMsg getPortQueryMsg() {
    unsigned char body[] = { 0x01 };
    return createUbxMsg( 0x06, 0x00, body, 1 );
}



// returns character if available, otherwise -1
int readUART( int fd ) {

    // Read a character from the port, if there is one
    unsigned char rx_buffer[1];
    ssize_t rx_length = read( fd, (void*)rx_buffer, 1 );		//Filestream, buffer to store in, number of bytes to read (max)
    if (rx_length < 0)
    {
        if( rx_length < -1 ) {
            printf( "Read error, %zi\n", rx_length );
            return -2;
        }
        return -1;
    }
    else if (rx_length == 0)
    {
        //No data waiting
        return -1;
    }
    else
    {
        //Bytes received
        return rx_buffer[0];
    }
}


// echoes characters received from GPS onto console; never exits
void echo( int fd ) {

    int done = 0;
    while( !done ) {
        nanosleep( &u50, 0 );
        int c = readUART( fd );
        if( c >= 0 ) printf( "%c", c );
        if( c > 255 ) done = 1;  // this will never happen; we're just fooling the endless loop detector
    }
}


// waits up to five seconds for the given string to be read
// returns 0 for failure, 1 for success
int waitForString( int fd, const char* str, int size ) {
    int attempts = 0;
    int gotIt = 0;
    int index = 0;
    while( (attempts < 100000) && (gotIt == 0) ) {
        nanosleep( &u50, 0 );
        int x = readUART( fd );
        if( x >= 0 ) {
            //printf("%c", x);
            if( x == str[index] ) {
                index++;
                if( index == size ) {
                    gotIt = 1;
                }
            }
            else {
                index = 0;
            }
        }
        attempts++;
    }
    return gotIt;
}


// Waits up to five seconds for a known message ("$GNGGA") to appear,
// returning 1 if synchronized, 0 otherwise.
// this is needed because upon opening the port the UART's receiver may
// not be synchronized to the data stream, and will return garbage data.
// Once synchronized, it should stay synchronized.
int syncRx( int fd ) {
    return (waitForString( fd, "$GNGGA,", 7 ) == 0 ) ? 0 : 1;
}


void setTermOptions( int fd, int baud ) {

    struct termios options;
    tcgetattr( fd, &options );
    options.c_cflag = (tcflag_t) (baud | CS8 | CLOCAL | CREAD);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush( fd, TCIFLUSH );
    tcsetattr( fd, TCSANOW, &options );
}


// returns file descriptor or -1 if failure...
// hint is 1 to try 115,200 baud first
int openUART( int quiet, int hint ) {

    int fd;
    fd = open( "/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY );		//Open in non blocking read/write mode

    if (fd == -1) {
        printf( "Can't open serial0 - possibly it's in use by another application\n" );
        return fd;
    }

    // we try twice...
    int try = (hint ? B115200 : B9600);
    int i;
    for( i = 0; i < 2; i++ ) {
        setTermOptions( fd, try );
        if( syncRx( fd ) ) {
            char* baud = ((try == B9600) ? "9,600" : "115,200" );
            if( !quiet ) printf( "Synchronized at %s baud...\n", baud );
            return fd;
        }
        try = ((try == B9600) ? B115200 : B9600);
    }

    // we failed, so close the port...
    close( fd );
    return -1;
}

// Waits up to five seconds for a UBX message to be received.
// Entire message is received, validated, and decoded.
// If no message is received, the noMsg flag will be set.
// If a valid message is received, the valid flag will be set.
UBXMsg rcvUbxMsg( int fd ) {
    struct ubxMsg result;
    result.noMsg = 1;
    result.valid = 0;

    int attempts = 0;
    int gotIt = 0;
    int index = 0;

    unsigned char header[6];

    // first we read the six byte header starting with 0xB5 0x62...
    while( (attempts < 100000) && (gotIt == 0) ) {
        nanosleep( &u50, 0 );
        int x = readUART( fd );
        if( x >= 0 ) {
            header[index] = (unsigned char) x;
            switch( index ) {
                case 0: if( x == 0xB5 ) index++; break;
                case 1: if( x == 0x62 ) index++; else index = 0; break;
                default: index++; if( index >= 6 ) gotIt = 1; break;
            }
        }
        attempts++;
    }
    if( !gotIt )
        return result;  // noMsg flag is set by default...

    // decode the header...
    result.bodySize = header[4] | (header[5] << 8);
    result.class = header[2];
    result.id = header[3];

    // then we read the body, however long it is, plus the two byte checksum...
    gotIt = 0;
    index = 0;
    result.body = calloc( result.bodySize + 2, 1 );

    while( (attempts < 100000) && (gotIt == 0) ) {
        nanosleep( &u50, 0 );
        int x = readUART( fd );
        if( x >= 0 ) {
            result.body[index++] = (unsigned char) x;
            if( index >= result.bodySize + 2 )
                gotIt = 1;
        }
        attempts++;
    }
    if( !gotIt )
        return result;  // noMsg flag is set by default...

    // we got the message, but we're not sure it's valid yet...
    result.noMsg = 0;

    // construct the raw message buffer...
    result.msg = calloc( result.bodySize + 8, 1 );
    memcpy( result.msg, header, 6 );
    memcpy( result.msg + 6, result.body, result.bodySize + 2 );

    // now we check to see if the message is valid (checksum matches)...
    // calculate Fletcher checksum for this message...
    unsigned char ck_a = 0;
    unsigned char ck_b = 0;
    int i;
    for( i = 2; i < result.bodySize + 6; i++ ) {
        ck_a += result.msg[i];
        ck_b += ck_a;
    }
    if( (result.msg[result.bodySize + 6] == ck_a) && (result.msg[result.bodySize + 7] == ck_b) ) {
        result.valid = 1;
    }
    return result;
}

// return 0 if error, 1 if succeeded
int sendUbxMsg( int fd, UBXMsg msg ) {

    // first we blast the message out...
    ssize_t c = write(fd, msg.msg, msg.size );
    if( c < 0 ) return 0;

    free( msg.msg );
    return 1;
}

// prints the body of the given UBX message in hex, or the raw message if the body is invalid
void printUbxMsg( UBXMsg msg ) {
    size_t size = msg.body ? msg.bodySize : msg.size;
    unsigned char * buf = msg.body ? msg.body : msg.msg;
    printf( msg.body ? "body:" : "msg:" );
    int i;
    for( i = 0; i < size; i++ ) {
        printf( "%02x:", buf[i] );
    }
    printf( "\n" );
}

// interprets the given port configuration message and prints the result
void printUbxPortConfiguration( UBXMsg msg ) {
    if( msg.noMsg || !msg.valid ) {
        printf( "Missing or invalid port configuration message!\n" );
        return;
    }

    // get the baud rate...
    int baud = msg.body[8] | (msg.body[9] << 8) | (msg.body[10] << 16) | (msg.body[11] << 24);
    printf( "%d baud\n", baud );

    // get the mode...
    int mode = msg.body[4] | (msg.body[5] << 8) | (msg.body[6] << 16) | (msg.body[7] << 24);
    int bits = 0x03 & (mode >> 6);
    int parity = 0x07 & (mode >> 9);
    int stop = 0x03 & (mode >> 12);
    printf( "%d bits\n", bits + 5 );
    switch( parity ) {
        case 0: printf( "even parity\n" );       break;
        case 1: printf( "odd parity\n" );        break;
        case 4: case 5: printf( "no parity\n" ); break;
        default: printf( "invalid parity\n" );   break;
    }
    switch( stop ) {
        case 0: printf( "1 stop bit\n" );    break;
        case 1: printf( "1.5 stop bits\n" ); break;
        case 2: printf( "2 stop bits\n" );   break;
        case 3: printf( "0.5 stop bits\n" ); break;
        default: break;
    }
}


// toggles the UART's baud rate between 9,600 baud and 115,200 baud
void toggle( int fd, int quiet ) {

    // first we get the current GPS port configuration
    int result = sendUbxMsg( fd, getPortQueryMsg() );
    if( !result ) {
        printf( "Failed to send port query message!\n" );
        exit( 1 );
    }
    UBXMsg cc = rcvUbxMsg( fd );
    if( cc.noMsg || !cc.valid ) {
        printf( "Port query response missing or invalid!\n" );
        exit( 1 );
    }

    // now we construct our new port configuration message...
    int curBaud = cc.body[8] | (cc.body[9] << 8) | (cc.body[10] << 16) | (cc.body[11] << 24);
    int newBaud = (curBaud == 9600) ? 115200: 9600;
    cc.body[8] = (unsigned char) newBaud;
    cc.body[9] = (unsigned char) newBaud >> 8;
    cc.body[10] = (unsigned char) newBaud >> 16;
    cc.body[11] = (unsigned char) (newBaud >> 24);
    UBXMsg setCfg = createUbxMsg( 0x06, 0x00, cc.body, 20 );

    // wait until we've had no rx data for at least three characters
    // we're hoping to hit a pause between the GPS sending bursts of characters
    int wt = (curBaud == 9600) ? 2813 : 234;
    int ct = 0;
    while( ct < wt ) {
        nanosleep( &u10, 0 );
        int c = readUART( fd );
        ct = (( c >= 0 ) ? 0 : ct + 10);
    }

    // send the configuration message...
    result = sendUbxMsg( fd, setCfg );
    if( !result ) {
        printf( "Failed to send port configuration message!\n" );
        return;
    }

    tcdrain( fd ); // wait for the preceding message to finish sending...

    // we ignore the expected ACK, because we don't know what baud rate it's going
    // to come back in
    if( !quiet ) printf( "Port configuration changed to %d baud\n", newBaud );
}

#define ARG_ERROR   0
#define ARG_USAGE   1
#define ARG_HELP    2
#define ARG_VERSION 3
#define ARG_BAUD    4
#define ARG_READ    5
#define ARG_TOGGLE  6
#define ARG_ECHO    7

static int mode = 0;
static int quiet = 0;
static int high = 0;

char *gpsctl_usage = "usage: gpsctl [-qh] -b | -r | -t | -e | -? | -V\n";
char *gpsctl_help = "Options:\n" \
                    "  -?, --help      display this help message and exit\n"\
                    "  -V, --version   display gpsctl version number\n"\
                    "  -b, --baud      report the GPS's baud rate\n" \
                    "  -e, --echo      echo GPS NMEA data to stdout\n"\
                    "  -h, --high      try synchronizing to 115,200 baud first, otherwise 9,600 baud\n"\
                    "  -q, --quiet     suppress non-error messages\n"\
                    "  -r, --read      read GPS port configuration\n"\
                    "  -t, --toggle    toggle GPS between 9,600 baud and 115,200 baud\n"\
                    "  --usage         display short message on usage of gpsctl\n";

static struct option options[] = {
    { "usage",   no_argument, &mode,  ARG_USAGE   },
    { "help",    no_argument, &mode,  ARG_HELP    },
    { "version", no_argument, &mode,  ARG_VERSION },
    { "baud",    no_argument, &mode,  ARG_BAUD    },
    { "read",    no_argument, &mode,  ARG_READ    },
    { "toggle",  no_argument, &mode,  ARG_TOGGLE  },
    { "echo",    no_argument, &mode,  ARG_ECHO    },
    { "quiet",   no_argument, &quiet, 1           },
    { "high",    no_argument, &high,  1           },
    { 0, 0, 0, 0 }
};


// get program options...
void getOptions( int argc, char *argv[] ) {
    int option_index;
    int c;
    int mc = 0;
    while( ( c = getopt_long( argc, argv, "breqth?V", options, &option_index) ) != -1 ) {

        if( c == -1 ) break;  // detect end of options...

        switch( c ) {
            case 0:
                if( options[option_index].flag == &mode ) mc++;
                break;
            case 'b': mc++; mode = ARG_BAUD;    break;
            case 'r': mc++; mode = ARG_READ;    break;
            case 't': mc++; mode = ARG_TOGGLE;  break;
            case 'e': mc++; mode = ARG_ECHO;    break;
            case 'V': mc++; mode = ARG_VERSION; break;
            case 'q': quiet = 1;                break;
            case 'h': high = 1;                 break;
            case '?': mc++; mode = ARG_HELP;    break;
            default:                            break;
        }
    }
    if( mc != 1 ) mode = ARG_ERROR;
}

int main( int argc, char *argv[] ) {

    getOptions( argc, argv );

    if( mode != ARG_ERROR ) {

        int result;
        int fd;

        switch( mode ) {

            case ARG_USAGE:
                puts( gpsctl_usage );
                break;

            case ARG_HELP:
            case ARG_ERROR:
                puts( gpsctl_doc );
                puts( gpsctl_usage );
                puts( gpsctl_help );
                break;

            case ARG_VERSION:
                puts( gpsctl_version );
                break;

            case ARG_BAUD:
                if( (fd = openUART( quiet, high )) < 0 ) exit( 1 );
                close( fd );
                exit(0);

            case ARG_TOGGLE:
                if( (fd = openUART( quiet, high )) < 0 ) exit( 1 );
                toggle( fd, quiet );
                break;

            case ARG_READ:
                if( (fd = openUART( quiet, high )) < 0 ) exit( 1 );
                result = sendUbxMsg( fd, getPortQueryMsg() );
                if (result) {
                    UBXMsg answer = rcvUbxMsg(fd);
                    printUbxPortConfiguration(answer);
                }
                break;

            case ARG_ECHO:
                if( (fd = openUART( quiet, high )) < 0 ) exit( 1 );
                echo( fd );
                break;

            default:
                break;
        }
        exit( 0 );
    }
    else {
        exit( 1 );
    }
}