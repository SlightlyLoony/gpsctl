//
// Created by Tom Dilatush on 10/20/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/termios.h>
#include "sl_general.h"
#include "ublox.h"
#include "sl_serial.h"

typedef enum ubxState    { Valid, NotValid, NotPresent } ubxState;
typedef enum ubxResponse { Ok, TxFail, InvalidPollResponse, NoPollResponse, UnexpectedPollResponse,
                           NAK, UnexpectedAck, InvalidAck, NoAck, None } ubxResponse;

typedef struct ubxChecksum {
    byte ck_a;
    byte ck_b;
} ubxChecksum;

typedef struct ubxType {
    byte class;
    byte id;
} ubxType;
#define UBX_ACK 0x05
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00
#define UBX_CFG 0x06
#define UBX_MON 0x0A
#define UBX_MON_VER 0x04
static ubxType ubxTypeNAK = { UBX_ACK, UBX_ACK_NAK };

typedef struct ubxMsg {
    ubxState state;
    ubxType type;
    sl_buffer body;
    ubxChecksum checksum;
} ubxMsg;


typedef struct ubxPollResponse {
    ubxMsg response;
    ubxResponse state;
} ubxPollResponse;


static void updateUbxChecksum( byte b, ubxChecksum *checksum ) {
    checksum->ck_a += b;
    checksum->ck_b += checksum->ck_a;
}


// Calculate and return the Fletcher checksum for the given UBX message, without modifying the message at all.
static ubxChecksum calcFletcherChecksum( const ubxMsg msg ) {

    ubxChecksum result = {0, 0};

    // first we get the class and id...
    updateUbxChecksum( msg.type.class, &result );
    updateUbxChecksum( msg.type.id, &result );

    // then the length, little-endian...
    updateUbxChecksum( (byte) (msg.body.size), &result );
    updateUbxChecksum( (byte) (msg.body.size >> 8), &result );

    // and finally the body itself...
    for( int i = 0; i < msg.body.size; i++ ) {
        updateUbxChecksum( msg.body.buffer[i], &result );
    }

    return result;
}


ubxMsg createUbxMsg( ubxType type, sl_buffer body ) {

    ubxMsg result;

    result.type     = type;
    result.body     = body;
    result.checksum = calcFletcherChecksum( result );
    result.state    = Valid;

    return result;
}


#define TC ((currentTimeMs() - startTime) < 1000)  // TimeCheck
static bool readUbxBytes( int fdPort, byte *buffer, int *count, size_t max, long long startTime, int charWaitNs ) {

    *count = 0;

    while( TC && (*count < max) ) {
        int c = readSerialChar( fdPort, charWaitNs );
        if( c >= 0 ) {
            buffer[*count] = (byte) c;
            (*count)++;
        }
    }
    return *count >= max;
}


// Waits up to one second for a UBX message to be received.  The state of the resulting message indicates the following:
//    Valid:      the entire message was received and the checksum was correct.
//    NotValid:   the entire message was received, but the checksum was not correct.
//    NotPresent: the message was not received.
#define SYNC1 (0xB5)
#define SYNC2 (0x62)
static ubxMsg rcvUbxMsg( int fdPort ) {

    ubxMsg result = { NotPresent, {0, 0}, {NULL, 0, false}, {0, 0}};

    long long startTime = currentTimeMs();
    speedInfo si = getSpeedInfo( fdPort );
    int charWaitNs = si.nsChar >> 1;

    while( TC ) {

        // first we synchronize on the 0xB5, 0x62 byte pair that starts every UBX message...
        int count = 0;
        while( TC && (count < 2) ) {
            int c = readSerialChar( fdPort, charWaitNs );
            if( c >= 0 ) {
                if ((count == 0) && (c == SYNC1)) {
                    count++;
                    continue;
                }
                if ((count == 1) && (c == SYNC2)) {
                    count++;
                    continue;
                }
                count = 0;
            }
        }
        if( count < 2 ) continue;

        // we're synchronized, so it's time to get the message type...
        byte buff[2];
        readUbxBytes( fdPort, buff, &count, 2, startTime, charWaitNs );
        if( count < 2 ) continue;
        ubxType type = {buff[0], buff[1]};
        result.type = type;

        // get the body length, little-endian...
        readUbxBytes( fdPort, buff, &count, 2, startTime, charWaitNs );
        if( count < 2 ) continue;
        size_t length = buff[0] | (buff[1] << 8);

        // now get the body...
        result.body = allocateSlBuffer( length );
        readUbxBytes( fdPort, result.body.buffer, &count, length, startTime, charWaitNs );
        result.checksum = calcFletcherChecksum( result );  // this calculates what the checksum SHOULD be...
        if( count >= length ) {

            // finally, get the checksum...
            readUbxBytes( fdPort, buff, &count, 2, startTime, charWaitNs );
            if( count >= 2 ) {
                bool valid = ( (buff[0] == result.checksum.ck_a) && (buff[1] == result.checksum.ck_b) );
                result.state = valid ? Valid : NotValid;
                break;
            }
        }
        freeSlBuffer( result.body );
    }

    return result;
}

// Transmits the given message to the UBX GPS, waiting for completion.  Returns 0 for success, -1 for failure.
int sendUbxMsg( int fdPort, ubxMsg msg ) {

    // first we send out our sync bytes, message type, and body length...
    byte buff[6] = {SYNC1, SYNC2, msg.type.class, msg.type.id, (byte) msg.body.size, (byte)(msg.body.size >> 8)};
    if( 6 != write( fdPort, buff, 6 ) ) return -1;

    // then we send out the body itself...
    if( msg.body.size != write( fdPort, msg.body.buffer, msg.body.size ) ) return -1;

    // and finally, out goes the checksum...
    byte buffChk[2] = {msg.checksum.ck_a, msg.checksum.ck_b};
    if( 2 != write( fdPort, buffChk, 2 ) ) return -1;

    // now we wait for everything to actually be sent...
    if( tcdrain( fdPort ) ) return -1;

    return 0;
}


// Returns true if the two given ubxType arguments are equal.
static bool cmpUbxType( ubxType a, ubxType b ) {
    return (a.class == b.class) && (a.id == b.id);
}


// Polls the GPX with the given message, then waits for the response and possibly an ACK/NAK.
ubxPollResponse pollUbx( int fdPort, ubxMsg pollMsg ) {

    ubxPollResponse result = { {}, None };

    // send the poll message...
    int txResult = sendUbxMsg( fdPort, pollMsg );
    if( txResult == 0 ) {

        // now get the poll response message...
        ubxMsg respMsg = rcvUbxMsg( fdPort );
        if( respMsg.state == Valid ) {

            // make sure it's the response type we expected...
            if( cmpUbxType( pollMsg.type, respMsg.type ) ) {

                // certain poll messages (notably configuration) ALSO send an ACK-ACK; for those, get it...
                if( pollMsg.type.class == UBX_CFG ) {

                    // get our ack, if our class of messages so requires...
                    ubxMsg ackMsg = rcvUbxMsg( fdPort );
                    if( ackMsg.state == Valid ) {

                        // make sure it as an ACK or NAK...
                        if( ackMsg.type.class == UBX_ACK ) {

                            // ok, now we're done except for recording our result...
                            result.response = respMsg;
                            result.state = (ackMsg.type.id == UBX_ACK_ACK) ? Ok : NAK;
                        }
                        else
                            result.state = UnexpectedAck;
                    }
                    else
                        result.state = (respMsg.state == NotValid) ? InvalidAck : NoAck;

                }
                else {

                    // we have a winner!
                    result.state = Ok;
                    result.response = respMsg;
                }
            }
            else {

                // we might have received a NAK (in the case of an invalid poll message being sent),
                // or maybe we got complete garbage, so we distinguish between them...
                if( cmpUbxType( respMsg.type, ubxTypeNAK ) )
                    result.state = NAK;
                else
                    result.state = UnexpectedPollResponse;
            }
        }
        else
            result.state = (respMsg.state == NotValid) ? InvalidPollResponse : NoPollResponse;
    }
    else
        result.state = TxFail;
    return result;
}


extern void test( int fdPort ) {

    ubxType type = { UBX_MON, UBX_MON_VER };
    sl_buffer body = allocateSlBuffer( 0 );
    ubxMsg msg = createUbxMsg( type, body );
    ubxPollResponse result = pollUbx( fdPort, msg );
    if( result.state == Ok ) {

        // decode the message and print the results...
        char* buf = (char *) result.response.body.buffer;

        printf( "SW Version: %s\n", buf );
        printf( "HW Version: %s\n", buf + 30 );
        for( int i = 40; i < (int) result.response.body.size; i += 30 ) {
            printf( "Extra info: %s\n", buf + i );
        }
    }
}

