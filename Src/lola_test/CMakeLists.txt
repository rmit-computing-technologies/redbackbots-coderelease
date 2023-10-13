cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(lola_test)

## Only continue if OFFNAO build
if(NOT RBB_BUILD_NAO)
   return()
endif(NOT RBB_BUILD_NAO)

find_package(Boost COMPONENTS 
        system 
    REQUIRED)
message(STATUS "BOOST=${Boost_INCLUDE_DIR}")
include_directories(${Boost_INCLUDE_DIR})

# All source files for lola test
SET(LOLA_TEST_SRCS
    battery.h
    fsr.h
    imu.h
    joints.h
    leds.cpp
    leds.h
    lola_connector.cpp
    lola_connector.h
    lola_frame.cpp
    lola_frame.h
    point_3d.h
    stl_ext.h
)

# RedBackBots Executable
add_executable(${LOLA_TEST_EXECUTEABLE}
    ${LOLA_TEST_SRCS}
)
target_link_libraries(${LOLA_TEST_EXECUTEABLE}
    Boost::system
    ${MSGPACK_LIB}
)
target_include_directories(${LOLA_TEST_EXECUTEABLE}
    PUBLIC .
)
