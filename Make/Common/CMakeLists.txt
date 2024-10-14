cmake_minimum_required(VERSION 3.16)

if(BUILD_NAO)
  message(STATUS "Build_NAO = TRUE")
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../CMake/NaoToolchain.cmake")
elseif(CMAKE_HOST_UNIX)
  message(STATUS "Build_HOST_UNIX = TRUE")
  set(CMAKE_CXX_COMPILER clang++)
endif()

if(NOT BUILD_NAO)
  set(BUILD_DESKTOP ON)
endif()

project(redbackbots LANGUAGES CXX)

set(CMAKE_CONFIGURATION_TYPES Debug Develop Release CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Develop CACHE STRING "" FORCE)
endif()

# C++ 14 is required for now due to Boost Python
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# RBB Source Files locations
set(RBB_PREFIX "${PROJECT_SOURCE_DIR}/../..")
set(RBB_SRC_PREFIX "${PROJECT_SOURCE_DIR}/../../Src")
set(OUTPUT_PREFIX "${PROJECT_SOURCE_DIR}/../..")
set(RBB_BUILDCHAIN_DIR "${RBB_PREFIX}/Util/Buildchain")
set(RBB_ROOT_DIR "${RBB_PREFIX}/Src/robot")
set(RBB_INCLUDE_DIR "${RBB_ROOT_DIR}/include")
set(RBB_SRC_DIR "${RBB_ROOT_DIR}/src")

# Uncomment this (locally) to build the test sources
# set(BUILD_RBB_TEST TRUE)

#####
# Setup Build and host platform targets
message(STATUS "CMake System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake Processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Cmake Host Processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

#####
# Configure Platform
add_definitions(-DLINUX)
# Ignore all compile warnings TODO should we keep?
add_definitions(-w)
if (UNIX)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(PLATFORM "LinuxArm")
  else()
    set(PLATFORM "Linux")
  endif()
endif()
message(STATUS "Platform: ${PLATFORM}")
if(NOT PLATFORM)
    message(FATAL_ERROR "Target platform not set")
endif()

#####
# Configure Host OS
if(CMAKE_HOST_UNIX)
    if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(OS "LinuxArm")
    else()
        set(OS "Linux")
    endif()
endif()
message(STATUS "Host OS: ${OS}")
if(NOT OS)
    message(FATAL_ERROR "OS could not be detected")
endif()

#####
set(CMAKE_CXX_COMPILER_LAUNCHER env CCACHE_CPP2=yes CCACHE_SLOPPINESS=pch_defines,time_macros ccache)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER CMake)
set_property(GLOBAL PROPERTY AUTOGEN_SOURCE_GROUP "Generated Files")
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT SimRobot)

set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEVELOP "")
set(CMAKE_CXX_FLAGS_RELEASE "")

set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "")
set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "")
set(CMAKE_EXE_LINKER_FLAGS_DEVELOP "")
set(CMAKE_MODULE_LINKER_FLAGS_DEVELOP "")
set(CMAKE_SHARED_LINKER_FLAGS_DEVELOP "")
set(CMAKE_STATIC_LINKER_FLAGS_DEVELOP "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "")
set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "")

# TODO TW: MVSC is MS Visual Studio Complier - is this required?
if(MSVC)
  add_compile_options(/W3 /GF /GS- /MP /Zc:wchar_t- /EHsc /permissive- /Zc:preprocessor /wd5105)
  add_definitions(-DNOMINMAX -D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS -D_CRT_SECURE_NO_DEPRECATE -D__SSSE3__)

  execute_process(COMMAND bash -c "cat /proc/cpuinfo" OUTPUT_VARIABLE _cpuinfo)
  string(FIND "${_cpuinfo}" "avx2" _has_avx2)
  if(NOT ${_has_avx2} EQUAL -1)
    add_compile_options(/arch:AVX2)
  else()
    add_compile_options(/arch:AVX)
  endif()
else()
  add_compile_options(-pipe
      -Wall
      -Wextra
      -Wconversion
      -Wno-unknown-warning-option
      -Wno-sign-conversion
      -Wno-implicit-int-float-conversion
      -Wunreachable-code
      -Xclang
      -fno-pch-timestamp)
  if(BUILD_DESKTOP)
    if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL arm64) # Option does not work on Apple M1, but "The Internet" says it also causes trouble on multiple other arm platforms.
      #add_compile_options(-march=native)
    endif()
    add_link_options(-fuse-ld=lld)
  endif()
  add_definitions(-D__extern_always_inline=inline)
endif()

set(_is_msvc "$<CXX_COMPILER_ID:MSVC>")
set(_is_clang "$<NOT:$<CXX_COMPILER_ID:MSVC>>")
set(_not_mac "$<NOT:$<PLATFORM_ID:Darwin>>")

# TODO: Investigate purpose of this
# # Library - Flags (develop)
add_library(Flags::ForDevelop INTERFACE IMPORTED)
target_compile_definitions(Flags::ForDevelop INTERFACE
    $<$<CONFIG:Release>:NDEBUG>)
target_compile_options(Flags::ForDevelop INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/bigobj>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/ZI>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Zi>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Release>>:/Od>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ob2>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oi>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ot>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oy>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/Ox>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/wd4189>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Release>>:-g>>
    $<${_is_clang}:$<$<CONFIG:Release>:-O3>>
    $<${_is_clang}:$<$<CONFIG:Release>:-fomit-frame-pointer>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-parameter>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-variable>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-lambda-capture>>)
target_link_options(Flags::ForDevelop INTERFACE
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Release>>:/DEBUG>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/INCREMENTAL:NO>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/SAFESEH:NO>>
    $<$<AND:${_not_mac},${_is_clang}>:$<$<CONFIG:Release>:-s>>)
# Library - Flags (default)
add_library(Flags::Default INTERFACE IMPORTED)
target_compile_definitions(Flags::Default INTERFACE
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>)
target_compile_options(Flags::Default INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/bigobj>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/ZI>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/Od>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/Ox>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/wd4189>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<CONFIG:Debug>:-g>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-O3>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-fomit-frame-pointer>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-parameter>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-variable>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-lambda-capture>>)
target_link_options(Flags::Default INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/DEBUG>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/INCREMENTAL:NO>>
    $<$<AND:${_not_mac},${_is_clang}>:$<$<CONFIG:Release>:-s>>)

if(BUILD_DESKTOP)
  # include("${SIMROBOT_PREFIX}/Make/Common/Box2D.cmake")
  # include("${SIMROBOT_PREFIX}/Make/Common/ODE.cmake")
  # include("${SIMROBOT_PREFIX}/Make/Common/OpenGL.cmake")
  # include("${SIMROBOT_PREFIX}/Make/Common/Qt5.cmake")

  add_library(GTest::GTest STATIC IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_PREFIX}/Util/gtest/include")
  set_target_properties(GTest::GTest PROPERTIES IMPORTED_LOCATION "${RBB_PREFIX}/Util/gtest/lib/${PLATFORM}/libgtest.a")
endif()

set(EIGEN_NAN_INITIALIZATION_CONDITION "$<NOT:$<CONFIG:Release>>")
# include("${SIMROBOT_PREFIX}/Make/Common/Eigen.cmake")

# TODO TW: Move GameController header files out of the main codebase into a separate library
# add_library(GameController::GameController INTERFACE IMPORTED)
# set_target_properties(GameController::GameController PROPERTIES
#     INTERFACE_INCLUDE_DIRECTORIES "${RBB_PREFIX}/Util/GameController/include")


##############
# Common Build Libraries

# Library - ALSA Sound (libasound)
if(BUILD_NAO)
  add_library(Nao::ALSA::ALSA SHARED IMPORTED)
  set_target_properties(Nao::ALSA::ALSA PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/gcc/V6/lib/libasound.so")
else()
  find_package(ALSA REQUIRED)
endif()

# Library - Boost (and components)
if(BUILD_NAO)
  message(STATUS "----- BUILD NAO CONFIGURATION -----")

  # Boost headers
  add_library(BoostInterface INTERFACE)
  set_target_properties(BoostInterface PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")

  # Boost program options
  add_library(Boost::ProgramOptions STATIC IMPORTED)
  set_target_properties(Boost::ProgramOptions PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::ProgramOptions PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/V6/libboost_program_options.a")

  # Boost python3
  add_library(Boost::Python3 STATIC IMPORTED)
  set_target_properties(Boost::Python3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Python3 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/V6/libboost_python38.a")

  # Boost regex
  add_library(Boost::Regex STATIC IMPORTED)
  set_target_properties(Boost::Regex PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Regex PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/V6/libboost_regex.a")

  # Boost system
  add_library(Boost::System STATIC IMPORTED)
  set_target_properties(Boost::System PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::System PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/V6/libboost_system.a")

  # Boost thread
  add_library(Boost::Thread STATIC IMPORTED)
  set_target_properties(Boost::Thread PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Thread PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/V6/libboost_thread.a")

  # Library - libexpat
  add_library(libexpat::libexpat SHARED IMPORTED)
  #set_target_properties(libexpat::libexpat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/expat/include")
  set_target_properties(libexpat::libexpat PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/expat/lib/V6/libexpat.so.1.6.11")

  # Library - libjpeg
  add_library(libjpeg::libjpeg STATIC IMPORTED)
  set_target_properties(libjpeg::libjpeg PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/libjpeg/include")
  set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/libjpeg/lib/V6/libjpeg.a")

  # Library - PNG
  add_library(Png::Png STATIC IMPORTED)
  set_target_properties(Png::Png PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/png/include")
  set_target_properties(Png::Png PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/png/lib/V6/libpng.a")

  # Library - Python 3
  add_library(Python38 STATIC IMPORTED)
  set_target_properties(Python38 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/python3.8/include" )
  set_target_properties(Python38 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/python3.8/lib/V6/libpython3.8.a")

  # Library - fast Fourier transform
  add_library(fftw3 STATIC IMPORTED)
  set_target_properties(fftw3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/fftw3/include")
  set_target_properties(fftw3 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/fftw3/lib/V6/libfftw3.a")

  add_library(CompiledNN STATIC IMPORTED)
  set_target_properties(CompiledNN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/CompiledNN/include")
  set_target_properties(CompiledNN PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/CompiledNN/lib/libCompiledNN.a")
  # set_target_properties(libtensorflow PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/libtensorflow/lib/libtensorflow_framework.so")

  # Library - hdf5
  add_library(hdf5 STATIC IMPORTED)
  set_target_properties(hdf5 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/hdf5/include")
  set_target_properties(hdf5 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/hdf5/lib/V6/libhdf5.a")

  #  Library - opencv
  add_library(opencv STATIC IMPORTED)
  set_target_properties(opencv PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/opencv/include")
  set_target_properties(opencv PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/opencv/lib/libopencv_ts.a")


endif()
if(BUILD_DESKTOP)
  message(STATUS "----- BUILD DESKTOP CONFIGURATION -----")
  # Boost headers
  add_library(BoostInterface INTERFACE)
  set_target_properties(BoostInterface PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")

  # Boost program options
  add_library(Boost::ProgramOptions STATIC IMPORTED)
  set_target_properties(Boost::ProgramOptions PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::ProgramOptions PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/${PLATFORM}/libboost_program_options.a")

  # Boost python3
  add_library(Boost::Python3 STATIC IMPORTED)
  set_target_properties(Boost::Python3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Python3 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/${PLATFORM}/libboost_python38.a")

  # Boost regex
  add_library(Boost::Regex STATIC IMPORTED)
  set_target_properties(Boost::Regex PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Regex PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/${PLATFORM}/libboost_regex.a")

  # Boost system
  add_library(Boost::System STATIC IMPORTED)
  set_target_properties(Boost::System PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::System PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/${PLATFORM}/libboost_system.a")

  # Boost thread
  add_library(Boost::Thread STATIC IMPORTED)
  set_target_properties(Boost::Thread PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/boost/include")
  set_target_properties(Boost::Thread PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/boost/lib/${PLATFORM}/libboost_thread.a")

  # Library - BZip2
  find_package(BZip2 REQUIRED)

  # Library - libexpat
  #add_library(libexpat::libexpat SHARED IMPORTED)
  #set_target_properties(libexpat::libexpat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/expat/include")
  #set_target_properties(libexpat::libexpat PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/expat/lib/${PLATFORM}/libexpat.so.1.6.11")

  # Library - libjpeg
  add_library(libjpeg::libjpeg STATIC IMPORTED)
  set_target_properties(libjpeg::libjpeg PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/libjpeg/include")
  set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/libjpeg/lib/${PLATFORM}/libjpeg.a")

  # Library - PNG
  add_library(Png::Png STATIC IMPORTED)
  set_target_properties(Png::Png PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/png/include")
  set_target_properties(Png::Png PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/png/lib/${PLATFORM}/libpng.a")

  # Library - Python 3
  # NOTE: Ubuntu 22.04 seems to need to use the SHARED library due to 32/64-bit alignment
  #       as inlcuded files from NaoV6 are 32-bit
  add_library(Python38 SHARED IMPORTED)
  set_target_properties(Python38 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/python3.8/include" )
  set_target_properties(Python38 PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/python3.8/lib/${PLATFORM}/libpython3.8.so")

endif()

# Library - Eigen
add_library(Eigen INTERFACE)
set_target_properties(Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/Eigen/include")

# Library - FADBAD++
add_library(FadBad INTERFACE)
set_target_properties(FadBad PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/FADBAD++/include")

# Library - msgpack
add_library(MsgpackInterface INTERFACE)
set_target_properties(MsgpackInterface PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_SRC_PREFIX}/msgpack/include")

# Library - Snappy
include("../CMake/snappy.cmake")

# Library - Tiny DNN
add_library(TinyDNN INTERFACE)
set_target_properties(TinyDNN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_BUILDCHAIN_DIR}/tiny-dnn")

# Common Redbackbots Library
include("../CMake/rbb-common.cmake")

##############
# Nao Source code build
if(BUILD_NAO OR (NOT DISABLE_NAO))
  include("../CMake/LolaTest.cmake")
  include("../CMake/BoostTest.cmake")
  include("../CMake/Nao.cmake")
  set(CMAKE_OSX_ARCHITECTURES "")
endif()

##############
# Desktop source code build
if (BUILD_DESKTOP)
  # Offnao itself
  include("../CMake/Offnao.cmake")
endif()

##############
# source group configuration

source_group(PCH REGULAR_EXPRESSION ".*[ch]xx$")
# source_group(Visualizers REGULAR_EXPRESSION ".*natvis$")
