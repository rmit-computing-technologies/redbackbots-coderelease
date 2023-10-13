set(WHISTLE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/alsarecorder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/fourier_transform.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/band_pass_filter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/sine_generator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/wavfile.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/whistle_classifier.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/whistle_detector.cpp"
)
add_library(whistle STATIC ${WHISTLE_SRC})
target_link_libraries(whistle asound fftw3f)

set(WHISTLE_DEBUGGER_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/whistle_debugger.cpp"
)
add_executable(whistledebug ${WHISTLE_DEBUGGER_SRC})
target_link_libraries(whistledebug whistle)

set(EXAMPLE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/example.cpp"
)
add_executable(minimal-example ${EXAMPLE_SRC})
target_link_libraries(minimal-example whistle)
