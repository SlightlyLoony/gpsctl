cmake_minimum_required(VERSION 3.8)
project(gpsctl)

set(CMAKE_C_STANDARD 11)

include_directories( /usr/local/include )

set(SOURCE_FILES gpsctl.c sl_general.c sl_general.h ublox.c ublox.h sl_serial.c sl_serial.h sl_buffer.c sl_buffer.h sl_bits.c sl_bits.h cJSON.h cJSON.c sl_options.c sl_options.h sl_return.c sl_return.h)
add_executable(gpsctl ${SOURCE_FILES})