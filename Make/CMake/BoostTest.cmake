# Only compile this test file if the BUILD_RBB_TEST flag is set
if (BUILD_RBB_TEST)
  set(BOOSTTEST_ROOT_DIR "${RBB_SRC_PREFIX}/boost_test")
  set(BOOSTTEST_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/BoostTest/$<CONFIG>")

  # Define source files
  file(GLOB_RECURSE BOOSTTEST_SOURCES
      "${BOOSTTEST_ROOT_DIR}/*.cpp" "${BOOSTTEST_ROOT_DIR}/*.h"    
      )

  if(BUILD_NAO)
    if(NOT (${PLATFORM} STREQUAL Linux AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
      message(ERROR "The target boost_test has to be built for Linux and with clang.")
    endif()

    ##########
    add_executable(BoostTest ${BOOSTTEST_SOURCES})
    set_property(TARGET BoostTest PROPERTY RUNTIME_OUTPUT_DIRECTORY "${BOOSTTEST_OUTPUT_DIR}")
    set_property(TARGET BoostTest PROPERTY RUNTIME_OUTPUT_NAME boosttest)
    set_property(TARGET BoostTest PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
    target_include_directories(BoostTest PRIVATE "${BOOSTTEST_ROOT_DIR}")
    
    # External dependent libraries
    target_link_libraries(BoostTest PRIVATE BoostInterface)
    target_link_libraries(BoostTest PRIVATE Boost::ProgramOptions)
    target_link_libraries(BoostTest PRIVATE Boost::Python3)
    target_link_libraries(BoostTest PRIVATE Boost::Regex)
    target_link_libraries(BoostTest PRIVATE Boost::System)
    target_link_libraries(BoostTest PRIVATE Boost::Thread)
    target_link_libraries(BoostTest PRIVATE libexpat::libexpat)
    target_link_libraries(BoostTest PRIVATE MsgpackInterface)
    target_link_libraries(BoostTest PRIVATE Python38)
    target_link_libraries(BoostTest PRIVATE -lcrypt)
    target_link_libraries(BoostTest PRIVATE -ldl-2.31)
    target_link_libraries(BoostTest PRIVATE -lpthread-2.31)
    target_link_libraries(BoostTest PRIVATE -lutil)
    target_link_libraries(BoostTest PRIVATE -lz)
    
    # For Boost Python
    # target_link_options(BoostTest PRIVATE -Wl,--export-dynamic)
    # target_link_options(BoostTest PRIVATE -shared)
    # target_link_options(BoostTest PRIVATE -fPIC)
    # target_link_options(BoostTest PRIVATE -rdynamic)
    # target_compile_options(BoostTest PRIVATE --export_dynamic)

    target_compile_definitions(BoostTest PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

    target_compile_options(BoostTest PRIVATE -Wno-switch)
    target_compile_options(BoostTest PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
    target_compile_options(BoostTest PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

    target_link_libraries(BoostTest PRIVATE Flags::Default)

  else()
    # Define an external project to complete the BoostTest build
    include(ExternalProject)
    ExternalProject_Add(BoostTest
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=$<CONFIG> -DBUILD_NAO=ON
        BUILD_ALWAYS 1
        USES_TERMINAL_BUILD ON
        INSTALL_COMMAND "")
    set_property(TARGET BoostTest PROPERTY FOLDER "")
  endif()
endif()