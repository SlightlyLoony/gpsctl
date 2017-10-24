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
#include "sl_buffer.h"
#include "sl_bits.h"


static ubxType ut_ACK_NAK = { UBX_ACK, UBX_ACK_NAK };
static ubxType ut_CFG_PRT = { UBX_CFG, UBX_CFG_PRT };
static ubxType ut_NAV_PVT = { UBX_NAV, UBX_NAV_PVT };


typedef struct ubxMsg {
    ubxState state;
    ubxType type;
    slBuffer *body;
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
    updateUbxChecksum( (byte) (size_slBuffer( msg.body )), &result );
    updateUbxChecksum( (byte) (size_slBuffer( msg.body ) >> 8), &result );

    // and finally the body itself...
    for( int i = 0; i < size_slBuffer( msg.body ); i++ ) {
        updateUbxChecksum( buffer_slBuffer( msg.body )[i], &result );
    }

    return result;
}


ubxMsg createUbxMsg( ubxType type, slBuffer *body ) {

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

    ubxMsg result = { NotPresent, {0, 0}, NULL, {0, 0}};

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
        result.body = create_slBuffer( length, LittleEndian );
        readUbxBytes( fdPort, buffer_slBuffer( result.body ), &count, length, startTime, charWaitNs );
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
        free( result.body );
    }

    return result;
}

// Transmits the given message to the UBX GPS, waiting for completion.  Returns 0 for success, -1 for failure.
int sendUbxMsg( int fdPort, ubxMsg msg ) {

    // first we send out our sync bytes, message type, and body length...
    byte buff[6] = { SYNC1,
                     SYNC2,
                     msg.type.class,
                     msg.type.id,
                     (byte) size_slBuffer( msg.body ),
                     (byte)(size_slBuffer( msg.body ) >> 8)
    };

    if( 6 != write( fdPort, buff, 6 ) ) return -1;

    // then we send out the body itself...
    if( size_slBuffer( msg.body ) != write( fdPort, buffer_slBuffer( msg.body ), size_slBuffer( msg.body ) ) )
        return -1;

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
                if( cmpUbxType( respMsg.type, ut_ACK_NAK ) )
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


extern changeBaudResponse changeBaudRate( int fdPort, unsigned int newBaudRate, bool verbose ) {

    // first we poll for the current configuration...
    ubxPollResponse current
            = pollUbx( fdPort, createUbxMsg( ut_CFG_PRT, init_slBuffer( LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */ ) ) );

    // update the baud rate in the configuration...
    put_uint32_slBuffer( current.response.body, 8, newBaudRate );

    // send out the message to change the baud rate...
    int result = sendUbxMsg( fdPort, createUbxMsg( ut_CFG_PRT, current.response.body ) );

    // change our host's baud rate to the new one...
    result = setTermOptionsBaud( fdPort, newBaudRate );

    // now get our response message...
    ubxMsg resp = rcvUbxMsg( fdPort );

    printf( "Received message status: %d\n", resp.state );

    return ChangeBaudOk;
}


// Corrects the raw time as reported by the GPS.
static void correctTime( pvt_fix *fix ) {

    // no matter what, we're gonna add the nanoseconds correction to the seconds...
    fix->second += ((int64_t) fix->nanoseconds_correction) * 1e-9;

    // if the result is positive, we're done...
    if( fix->second >= 0.0 ) return;

    // otherwise, the time was rounded up and now we have to undo that...
    fix->second += 60;  // fix the seconds wrap-around - note this is wrong for leap-seconds, but how to fix that?

    // now fix the minutes...
    fix->minute--;
    if( fix->minute >= 0 ) return;
    fix->minute += 60;

    // then the hours...
    fix->hour--;
    if( fix->hour >= 0 ) return;
    fix->hour += 24;

    // we rounded to the next day - sheesh, fix it...
    fix->day--;
    if( fix->day >= 1 ) return;

    // oh oh - we rounded up from the preceding month!
    // now this starts to get a little complicated - we have to know the days in a month, which means leap years...
    fix->month--;
    if( fix->month >= 1) {
        fix->day = daysInMonth((unsigned) fix->year, (unsigned) fix->month);
        return;
    }

    // sheesh - we even rounded up the year!
    fix->year--;
    fix->month = 12;
    fix->day = daysInMonth( (unsigned) fix->year, (unsigned) fix->month );
}


extern int getFix( int fdPort, bool verbose, pvt_fix *fix ) {

    // get a fix from the GPS...
    int result = sendUbxMsg( fdPort, createUbxMsg( ut_NAV_PVT, create_slBuffer( 0, LittleEndian ) ) );
    ubxMsg resp = rcvUbxMsg( fdPort );

    if( (resp.state == Valid) && (size_slBuffer( resp.body ) >= 92) ) {

        fix->year                      = get_uint16_slBuffer( resp.body, 4 );
        fix->month                     = get_uint8_slBuffer( resp.body, 6 );
        fix->day                       = get_uint8_slBuffer( resp.body, 7 );
        fix->hour                      = get_uint8_slBuffer( resp.body, 8);
        fix->minute                    = get_uint8_slBuffer( resp.body, 9 );
        fix->second                    = get_uint8_slBuffer( resp.body, 10);
        fix->nanoseconds_correction    = get_int32_slBuffer( resp.body, 16);
        fix->time_accuracy_ns          = get_uint32_slBuffer( resp.body, 12 );
        fix->date_valid                = isBitSet_slBits( get_uint8_slBuffer( resp.body, 11 ), 0);
        fix->time_valid                = isBitSet_slBits( get_uint8_slBuffer( resp.body, 11 ), 1);
        fix->time_resolved             = isBitSet_slBits( get_uint8_slBuffer( resp.body, 11 ), 2);
        fix->fix_is_3d                 = (get_uint8_slBuffer( resp.body, 20) == 3);
        fix->fix_valid                 = isBitSet_slBits( get_uint8_slBuffer( resp.body, 21 ), 0);
        fix->number_of_satellites_used = get_uint8_slBuffer( resp.body, 23);
        fix->latitude_deg              = get_int32_slBuffer( resp.body, 28 ) * 1e-7;
        fix->longitude_deg             = get_int32_slBuffer( resp.body, 24 ) * 1e-7;
        fix->height_above_ellipsoid_mm = get_int32_slBuffer( resp.body, 32 );
        fix->height_above_sea_level_mm = get_int32_slBuffer( resp.body, 36 );

        correctTime( fix );

        return 0;
    }

    return -1;
}


extern void test( int fdPort ) {

    ubxType type = { UBX_MON, UBX_MON_VER };
    slBuffer* body = create_slBuffer( 0, LittleEndian );
    ubxMsg msg = createUbxMsg( type, body );
    ubxPollResponse result = pollUbx( fdPort, msg );
    if( result.state == Ok ) {

        // decode the message and print the results...
        char* buf = (char *) buffer_slBuffer( result.response.body );

        printf( "SW Version: %s\n", buf );
        printf( "HW Version: %s\n", buf + 30 );
        for( int i = 40; i < (int) size_slBuffer( result.response.body ); i += 30 ) {
            printf( "Extra info: %s\n", buf + i );
        }
    }
}

