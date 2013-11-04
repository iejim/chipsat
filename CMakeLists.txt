cmake_minimum_required(VERSION 2.8)
project(chipsat)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    set(BUILD_POSTFIX "_d")
else (CMAKE_BUILD_TYPE STREQUAL Debug)
    set(BUILD_POSTFIX "")
endif (CMAKE_BUILD_TYPE STREQUAL Debug)

include_directories(include)

# collect header files
FILE(GLOB chipsat_HEADER include/*.h)

add_subdirectory(src)

# Build the examples only in Debug-builds
if(CMAKE_BUILD_TYPE STREQUAL Debug)
    add_subdirectory(examples)
endif(CMAKE_BUILD_TYPE STREQUAL Debug)



