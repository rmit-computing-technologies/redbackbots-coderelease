# Only compile this test file if the BUILD_RBB_TEST flag is set
if (BUILD_RBB_TEST)
  set(LOLA_ROOT_DIR "${RBB_SRC_PREFIX}/lola_test")
  set(LOLA_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/LolaTest/$<CONFIG>")

  # Define source files
  file(GLOB_RECURSE LOLA_SOURCES
      "${LOLA_ROOT_DIR}/*.cpp" "${LOLA_ROOT_DIR}/*.h"    
      )

  if(BUILD_NAO)
    if(NOT (${PLATFORM} STREQUAL Linux AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
      message(ERROR "The target lola_test has to be built for Linux and with clang.")
    endif()

    ##########
    add_executable(LolaTest ${LOLA_SOURCES})
    set_property(TARGET LolaTest PROPERTY RUNTIME_OUTPUT_DIRECTORY "${LOLA_OUTPUT_DIR}")
    set_property(TARGET LolaTest PROPERTY RUNTIME_OUTPUT_NAME lolatest)
    set_property(TARGET LolaTest PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
    target_include_directories(LolaTest PRIVATE "${LOLA_ROOT_DIR}")
    
    # External dependent libraries
    target_link_libraries(LolaTest PRIVATE BoostInterface)
    target_link_libraries(LolaTest PRIVATE Boost::System)
    target_link_libraries(LolaTest PRIVATE Boost::Thread)
    target_link_libraries(LolaTest PRIVATE MsgpackInterface)
    target_link_libraries(LolaTest PRIVATE -lpthread-2.31)

    target_compile_definitions(LolaTest PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

    target_compile_options(LolaTest PRIVATE -Wno-switch)
    target_compile_options(LolaTest PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
    target_compile_options(LolaTest PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

    target_link_libraries(LolaTest PRIVATE Flags::Default)

  else()
    # Define an external project to complete the LolaTest build
    include(ExternalProject)
    ExternalProject_Add(LolaTest
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=$<CONFIG> -DBUILD_NAO=ON
        BUILD_ALWAYS 1
        USES_TERMINAL_BUILD ON
        INSTALL_COMMAND "")
    set_property(TARGET LolaTest PROPERTY FOLDER "")
  endif()
endif()