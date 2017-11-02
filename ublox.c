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

typedef enum ackResponse { ackACK, ackNAK, ackMissing, ackUnexpectedMessage, ackError } ackResponse;


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


static bool readUbxBytes( int fdPort, byte *buffer, int *count, size_t max, long long startTime, long long charWaitMs ) {

    *count = 0;

    while( ((currentTimeMs() - startTime) < 1000) && (*count < max) ) {
        int c = readSerialChar( fdPort, charWaitMs );
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
// The body (an slBuffer) is only allocated if the message returned Valid.
#define SYNC1 (0xB5)
#define SYNC2 (0x62)
static ubxMsg rcvUbxMsg( int fdPort ) {

    ubxMsg result = { NotPresent, {0, 0}, NULL, {0, 0}};

    long long startTime = currentTimeMs();

    while( ((currentTimeMs() - startTime) < 1000) ) {

        // first we synchronize on the 0xB5, 0x62 byte pair that starts every UBX message...
        int count = 0;
        while( ((currentTimeMs() - startTime) < 1000) && (count < 2) ) {
            int c = readSerialChar( fdPort, startTime + 1000 - currentTimeMs() );
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
        readUbxBytes( fdPort, buff, &count, 2, startTime, startTime + 1000 - currentTimeMs() );
        if( count < 2 ) continue;
        ubxType type = {buff[0], buff[1]};
        result.type = type;

        // get the body length, little-endian...
        readUbxBytes( fdPort, buff, &count, 2, startTime, startTime + 1000 - currentTimeMs() );
        if( count < 2 ) continue;
        size_t length = buff[0] | (buff[1] << 8);

        // now get the body...
        result.body = create_slBuffer( length, LittleEndian );
        readUbxBytes( fdPort, buffer_slBuffer( result.body ), &count, length, startTime, startTime + 1000 - currentTimeMs() );
        result.checksum = calcFletcherChecksum( result );  // this calculates what the checksum SHOULD be...
        if( count >= length ) {

            // finally, get the checksum...
            readUbxBytes( fdPort, buff, &count, 2, startTime, startTime + 1000 - currentTimeMs() );
            if( count >= 2 ) {
                bool valid = ( (buff[0] == result.checksum.ck_a) && (buff[1] == result.checksum.ck_b) );
                result.state = valid ? Valid : NotValid;
                if( !valid ) free( result.body );
                break;
            }
        }
        free( result.body );
    }

    return result;
}


// Waits for an ACK/NAK message.  Returns appropriate ackResponse value.
static ackResponse ubxAckWait( int fdPort ) {

    ubxMsg ackMsg = rcvUbxMsg( fdPort );
    if( ackMsg.state == Valid ) {
        free(ackMsg.body);
        return ( ackMsg.type.class != UBX_ACK ) ? ackUnexpectedMessage : (ackMsg.type.id == UBX_ACK_ACK) ? ackACK : ackNAK;
    }
    else
        return (ackMsg.state == NotValid) ? ackError : ackMissing;
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


// Send the given ubxMsg, then wait for an ACK and return the result.
static ackResponse sendUbxAckedMsg( int fdPort, ubxMsg msg ) {
    int result = sendUbxMsg( fdPort, msg );
    if( result != 0 ) return ackError;
    return ubxAckWait( fdPort );
}


// Returns true if the two given ubxType arguments are equal.
static bool cmpUbxType( ubxType a, ubxType b ) {
    return (a.class == b.class) && (a.id == b.id);
}


// Polls the GPX with the given message, then waits for the response and possibly an ACK/NAK.  The response message
// body is only allocated if the response state is "Ok".
static ubxPollResponse pollUbx( int fdPort, ubxMsg pollMsg ) {

    ubxPollResponse result = { {}, None };

    // send the poll message...
    int txResult = sendUbxMsg( fdPort, pollMsg );
    free( pollMsg.body );
    if( txResult == 0 ) {

        // now get the poll response message...
        ubxMsg respMsg = rcvUbxMsg( fdPort );
        if( respMsg.state == Valid ) {

            // make sure it's the response type we expected...
            if( cmpUbxType( pollMsg.type, respMsg.type ) ) {

                // certain poll messages (notably configuration) ALSO send an ACK-ACK; for those, get it...
                if( pollMsg.type.class == UBX_CFG ) {

                    // get our ack, if our class of messages so requires...
                    ackResponse ackResp = ubxAckWait( fdPort );
                    switch( ackResp ) {
                        case ackACK:               result.state = Ok; result.response = respMsg;       break;
                        case ackNAK:               result.state = NAK; free( respMsg.body );           break;
                        case ackUnexpectedMessage: result.state = UnexpectedAck; free( respMsg.body ); break;
                        case ackMissing:           result.state = NoAck; free( respMsg.body );         break;
                        case ackError:             result.state = InvalidAck; free( respMsg.body );    break;
                    }
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

                free( respMsg.body );
            }
        }
        else
            result.state = (respMsg.state == NotValid) ? InvalidPollResponse : NoPollResponse;

    }
    else
        result.state = TxFail;
    return result;
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


// Fills the fix structure at the given pointer with information about a time and position fix obtained from the
// GPS system.  Returns the appropriate reportResponse value.
extern reportResponse getFix( int fdPort, int verbosity, pvt_fix *fix ) {

    // get a fix from the GPS...
    ubxPollResponse pollResp = pollUbx( fdPort, createUbxMsg( ut_NAV_PVT, create_slBuffer( 0, LittleEndian ) ) );

    // convenience variable...
    slBuffer *body = pollResp.response.body;

    if( (pollResp.state == Ok) && (size_slBuffer( body ) >= 92) ) {

        fix->year                              = get_uint16_slBuffer( body, 4 );
        fix->month                             = get_uint8_slBuffer( body, 6 );
        fix->day                               = get_uint8_slBuffer( body, 7 );
        fix->hour                              = get_uint8_slBuffer( body, 8);
        fix->minute                            = get_uint8_slBuffer( body, 9 );
        fix->second                            = get_uint8_slBuffer( body, 10);
        fix->nanoseconds_correction            = get_int32_slBuffer( body, 16);
        fix->time_accuracy_ns                  = get_uint32_slBuffer( body, 12 );
        fix->date_valid                        = isBitSet_slBits( get_uint8_slBuffer( body, 11 ), 0);
        fix->time_valid                        = isBitSet_slBits( get_uint8_slBuffer( body, 11 ), 1);
        fix->time_resolved                     = isBitSet_slBits( get_uint8_slBuffer( body, 11 ), 2);
        fix->fix_is_3d                         = (get_uint8_slBuffer( body, 20) == 3);
        fix->fix_valid                         = isBitSet_slBits( get_uint8_slBuffer( body, 21 ), 0);
        fix->number_of_satellites_used         = get_uint8_slBuffer( body, 23);
        fix->latitude_deg                      = get_int32_slBuffer( body, 28 ) * 1e-7;
        fix->longitude_deg                     = get_int32_slBuffer( body, 24 ) * 1e-7;
        fix->height_above_ellipsoid_mm         = get_int32_slBuffer( body, 32 );
        fix->height_above_sea_level_mm         = get_int32_slBuffer( body, 36 );
        fix->height_accuracy_mm                = get_uint32_slBuffer( body, 44 );
        fix->horizontal_accuracy_mm            = get_uint32_slBuffer( body, 40 );
        fix->ground_speed_mm_s                 = get_int32_slBuffer( body, 60 );
        fix->ground_speed_accuracy_mm_s        = get_uint32_slBuffer( body, 68 );
        fix->heading_deg                       = get_int32_slBuffer( body, 64 ) * 1e-5;
        fix->heading_accuracy_deg              = get_uint32_slBuffer( body, 72 ) * 1e-5;
        #undef body
        correctTime( fix );

        return reportOK;
    }

    return reportError;
}


// Fills the ubxVersion structure at the given pointer with information about the GPS's version.  Returns the
// appropriate reportResponse value.
extern reportResponse getVersion(int fdPort, int verbosity, ubxVersion *version ) {

    ubxType type = { UBX_MON, UBX_MON_VER };
    slBuffer* body = create_slBuffer( 0, LittleEndian );
    ubxMsg msg = createUbxMsg( type, body );
    ubxPollResponse result = pollUbx( fdPort, msg );
    if( result.state == Ok ) {

        // decode the message and print the results...
        char* buf = (char *) buffer_slBuffer( result.response.body );

        version->software = getAllocatedStringCopy( buf );
        version->hardware = getAllocatedStringCopy( buf + 30 );

        // calculate the number of extension entries we have...
        int ne = ((int) size_slBuffer( result.response.body ) - 40) / 30;
        version->number_of_extensions = ne;
        if( ne == 0 )
            version->extensions = NULL;
        else {
            version->extensions = safeMalloc( (size_t) (sizeof(buf) * ne) );
            char** ptr = version->extensions;
            for (int i = 40; i < (int) size_slBuffer(result.response.body); i += 30) {
                *ptr = getAllocatedStringCopy( buf + i );
                ptr++;
            }
            *ptr = NULL;
        }

        return reportOK;
    } else
        return reportError;
}


extern bool ubxSynchronize( const int fdPort, const int verbosity ) {

    // we're gonna try this up to 10 times...
    for( int i = 0; i < 10; i++ ) {
        // send the request message and wait for the response...
        ubxType type = {UBX_MON, UBX_MON_VER};
        slBuffer *body = create_slBuffer(0, LittleEndian);
        ubxMsg msg = createUbxMsg(type, body);
        ubxPollResponse result = pollUbx(fdPort, msg);
        if (result.state == Ok) return true;
        if( verbosity >= 3 ) printf( "." );
    }
    return false;
}


// Returns the current state of NMEA data on the GPS's UART.  Returns 0 if NMEA is off, 1 if NMEA is on, and -1
// if there was an error.
static int isNmeaOn( int fdPort ) {

    // poll for the current configuration...
    ubxPollResponse current
            = pollUbx( fdPort, createUbxMsg( ut_CFG_PRT, init_slBuffer( LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */ ) ) );

    if( current.state != Ok ) return -1;

    return isBitSet_slBits( get_uint16_slBuffer( current.response.body, 14 ), 1 ) ? 1 : 0;
}


// TODO: better error handling here...
// Configures whether the GPS transmits NMEA periodically on the UART.  Returns success or failure.
extern configResponse setNMEAData(int fdPort, int verbosity, bool nmeaOn) {

    char* onoff = nmeaOn ? "on" : "off";

    // first we poll for the current configuration...
    if( verbosity >= 3 ) printf( "Querying to see if GPS UART already has NMEA data turned %s...\n", onoff );
    int cn = isNmeaOn( fdPort );
    if( cn < 0 ) {
        printf( "Querying for state of GPS UART failed!\n" );
        return configError;
    }

    // if we're already in the state we want, just leave...
    if( cn == nmeaOn ) {
        if( verbosity >= 3 ) printf( "GPS UART already has NMEA data turned %s...\n", onoff );
        return configOK;
    }

    // change the configuration to the way we want it, starting with polling it again...
    if( verbosity >= 3 ) printf( "Polling GPS UART configuration...\n" );
    ubxPollResponse current
            = pollUbx( fdPort, createUbxMsg( ut_CFG_PRT, init_slBuffer( LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */ ) ) );
    if( current.state != Ok ) return configError;

    // now change the bit we need to diddle...
    slBuffer *bdy = current.response.body;
    put_uint16_slBuffer( bdy, 14, (uint16_t) setBit_slBits( get_uint16_slBuffer( bdy, 14 ), 1, nmeaOn ) );

    // send out the message to actually make the change...
    if( verbosity >= 3 ) printf( "Sending new configuration to GPS with NMEA data turned %s for the UART...\n", onoff );
    ackResponse ackResp = sendUbxAckedMsg( fdPort, createUbxMsg( ut_CFG_PRT, bdy ) );
    if( ackResp == ackACK ) {
        if( verbosity >= 1 ) printf( "Successfully set new configuration to turn NMEA data %s for GPS UART...\n", onoff );
        return configOK;
    }
    else {
        if( verbosity >= 1 ) printf( "Failed to change GPS UART configuration to turn NMEA data %s...\n", onoff );
        return configError;
    }
}


static configResponse checkNMEA( int fdPort, int nmea, bool verbose ) {
    if( nmea != 1 ) return configOK;
    if( verbose ) printf( "Turning NMEA data back on...\n" );
    return setNMEAData(fdPort, verbose, true);
}


extern changeBaudResponse changeBaudRate( int fdPort, unsigned int newBaudRate, bool verbose ) {

    // if NMEA data is on, turn it off and remember...
    int nmea = isNmeaOn( fdPort );
    if( nmea < 0 ) {
        printf( "Could not determine if NMEA data was turned on!\n" );
        return ChangeBaudFailure;
    }
    if( nmea == 1 ) {
        if( verbose ) printf( "NMEA data is turned on, so turning it off...\n" );

        // turn NMEA data off, wait a second for the accumulated data to be transmitted, then flush everything...
        configResponse ans = setNMEAData(fdPort, verbose, false);
        if( ans != configOK ) return ChangeBaudFailure;
        sleep( 1 );
        tcflush( fdPort, TCIOFLUSH );
    }

    // first we poll for the current configuration...
    ubxPollResponse current
            = pollUbx( fdPort, createUbxMsg( ut_CFG_PRT, init_slBuffer( LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */ ) ) );
    if( current.state != Ok ) {
        checkNMEA( fdPort, nmea, verbose );
        printf( "Problem polling for current configuration before changing baud rate!\n" );
        return ChangeBaudFailure;
    }

    // update the baud rate in the configuration...
    put_uint32_slBuffer( current.response.body, 8, newBaudRate );

    // send out the message to change the baud rate...
    int sendResp = sendUbxMsg( fdPort, createUbxMsg( ut_CFG_PRT, current.response.body ) );
    if( sendResp != 0 ) {
        checkNMEA( fdPort, nmea, verbose );
        printf( "Problem sending message to change baud rate!\n" );
        return ChangeBaudFailure;
    }

    // change our host's baud rate to the new one...
    int stoResp = setTermOptionsBaud( fdPort, newBaudRate );
    if( stoResp != 0 ) {
        checkNMEA( fdPort, nmea, verbose );
        printf( "Problem changing the host baud rate!\n" );
        return ChangeBaudFailure;
    }

    // now get our ACK...
    ackResponse ackResp = ubxAckWait( fdPort );

    // sometimes we don't get the ACK, likely hosed by the baud rate change...
    // so in that case we'll query the port and see what we actually have...
    if( ackResp == ackMissing ) {

        // poll the current configuration again...
        if( verbose ) printf( "ACK missing, repolling for status...\n" );
        ubxPollResponse repoll = pollUbx( fdPort, createUbxMsg( ut_CFG_PRT,
                                                 init_slBuffer( LittleEndian, 1 /* UBX_CFG_PRT_UART_ID */ ) ) );
        if( repoll.state == Ok ) {
            ackResp = ackACK;
        }
    }

    // if we turned off NMEA, now turn it back on...
    if( checkNMEA( fdPort, nmea, verbose ) != configOK ) return ChangeBaudFailure;

    switch( ackResp ) {
        case ackACK:
            if( verbose ) printf( "Successfully changed baud rate...\n" );
            return ChangeBaudOk;
        case ackNAK:
            printf( "Received NAK after setting new baud rate!\n" );
            return ChangeBaudFailure;
        case ackMissing:
            printf( "Never received ACK after setting new baud rate!\n" );
            return ChangeBaudFailure;
        case ackError:
            printf( "Error when waiting for ACK after setting new baud rate!\n" );
            return ChangeBaudFailure;
        case ackUnexpectedMessage:
            printf( "Received an unexpected message type when waiting for ACK!\n" );
            return ChangeBaudFailure;
        default:
            printf( "Unknown ACK response (code %d)!\n", ackResp );
            return ChangeBaudFailure;
    }
}
