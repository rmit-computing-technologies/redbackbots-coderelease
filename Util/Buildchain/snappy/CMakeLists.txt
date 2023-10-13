cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Snappy Sources
SET(SNAPPY_SRCS
    snappy-sinksource.cc
    snappy-stubs-internal.cc
    snappy.cc
)

# Setup Snappy as a separate library
add_library(${SNAPPY_LIRBARY}
    STATIC
    ${SNAPPY_SRCS}
)
