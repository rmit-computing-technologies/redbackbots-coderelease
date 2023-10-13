cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Snappy root location
set(SNAPPY_ROOT_DIR "${RBB_PREFIX}/Util/Buildchain/snappy")
set(SNAPPY_SRC_DIR "${SNAPPY_ROOT_DIR}/src")

# Snappy Sources
SET(SNAPPY_SRCS
    ${SNAPPY_SRC_DIR}/snappy-sinksource.cc
    ${SNAPPY_SRC_DIR}/snappy-stubs-internal.cc
    ${SNAPPY_SRC_DIR}/snappy.cc
)

# Create a different Library for Nao and Desktop buils
if (BUILD_NAO)
    set(SnappyName "SnappyNao")
elseif (BUILD_DESKTOP)
    set(SnappyName "SnappyDesktop")
else()
    message(FATAL_ERROR "Cannot determine a library type for Snappy")
endif()

# Setup Snappy as a separate library
add_library(${SnappyName}
    STATIC
    ${SNAPPY_SRCS}
)

# Supress warnings from Snappy
target_compile_options(${SnappyName} PRIVATE -Wno-shorten-64-to-32)
target_compile_options(${SnappyName} PRIVATE -Wno-unused-parameter)
target_compile_options(${SnappyName} PRIVATE -Wno-implicit-int-conversion)

# Include dirs
target_include_directories(${SnappyName} PUBLIC "${SNAPPY_ROOT_DIR}/include")
set_target_properties(${SnappyName} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${SNAPPY_ROOT_DIR}/include")
