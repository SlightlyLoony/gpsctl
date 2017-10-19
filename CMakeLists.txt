cmake_minimum_required(VERSION 3.8)
project(gpsctl)

set(CMAKE_C_STANDARD 11)

set(SOURCE_FILES gpsctl.c slightly_loony.c slightly_loony.h)
add_executable(gpsctl ${SOURCE_FILES})