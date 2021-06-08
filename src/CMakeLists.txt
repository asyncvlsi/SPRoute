cmake_minimum_required(VERSION 3.16)

FILE(GLOB CPP_SOURCES "*.cpp")
FILE(GLOB C_SOURCES "*.c")
add_library(sproute ${CPP_SOURCES} ${c_SOURCES})
