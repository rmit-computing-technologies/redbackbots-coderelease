
#####
# RBB Source Files root location
set(OFFNAO_ROOT_DIR "${RBB_PREFIX}/Src/offnao")
set(OFFNAO_INCLUDE_DIR "${OFFNAO_ROOT_DIR}/include")
set(OFFNAO_SRC_DIR "${OFFNAO_ROOT_DIR}/src")
set(VATNAO_ROOT_DIR "${RBB_PREFIX}/Src/vatnao")
set(VATNAO_INCLUDE_DIR "${VATNAO_ROOT_DIR}")
set(VATNAO_RESOURCE_DIR "${VATNAO_ROOT_DIR}/ui/resources")
set(VATNAO_SRC_DIR "${VATNAO_ROOT_DIR}")
set(VATNAO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Vatnao/$<CONFIG>")

if(BUILD_DESKTOP)
    file(GLOB_RECURSE VATNAO_SOURCES
        "${VATNAO_SRC_DIR}/*.cpp" 
        "${VATNAO_SRC_DIR}/**/*.cpp" 
        "${OFFNAO_SRC_DIR}/serialise.cpp" 
        "${RBB_SRC_DIR}/blackboard/serialise.cpp"
    )

    file(GLOB_RECURSE VATNAO_MOC
        "${VATNAO_INCLUDE_DIR}/*.hpp" 
        "${VATNAO_INCLUDE_DIR}/**/*.hpp" 
        "${OFFNAO_INCLUDE_DIR}/*.hpp" 
        "${OFFNAO_INCLUDE_DIR}/**/*.hpp"
    )

    #####
    # Protobuf Configuration
    set(Protobuf_INCLUDE_DIR ${RBB_BUILDCHAIN_DIR}/protobuf/include/)
    set(Protobuf_LIBRARY ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotobuf.so)
    set(Protobuf_PROTOC_LIBRARIES ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotoc.so)
    set(Protobuf_PROTOC_EXECUTABLE ${RBB_BUILDCHAIN_DIR}/protobuf/bin/${OS}/protoc)
    # Proto files to prevent duplicate linking issues
    # Generate Protobuf files
    include(${RBB_BUILDCHAIN_DIR}/protobuf/cmake/FindProtobuf.cmake)
    protobuf_generate_cpp(PROTO_SRCS_VATNAO PROTO_HDRS_VATNAO
        ${RBB_INCLUDE_DIR}/blackboard/Blackboard.proto
        ${RBB_INCLUDE_DIR}/blackboard/naoData.proto
    )

    # Setup Protobuf library include
    add_library(Google::ProtobufVN STATIC IMPORTED)
    set_target_properties(Google::ProtobufVN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIR}")
    set_target_properties(Google::ProtobufVN PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotobuf.a")

#    #####
#    # Define Vatnao only source files
#    SET(VATNAO_SOURCES
#            # Main
#            ${VATNAO_SRC_DIR}/
#
#            # Protobuf source files
#            # Required both serialise from Robot and Vatnao
##            ${RBB_SRC_DIR}/blackboard/serialise.cpp
##            ${VATNAO_SRC_DIR}/serialise.cpp
##            ${PROTO_SRCS}
##            ${PROTO_HDRS}
#            )

#    set(VATNAO_MOC
#            ${VATNAO_INCLUDE_DIR}/readers/reader.hpp
#            )

    # if(CMAKE_TOOLCHAIN_FILE)
    #   list(APPEND VATNAO_CXX_SRCS ${VATNAO_INCLUDE_DIR}/tabs/cameraTab.cpp)
    #   list(APPEND VATNAO_MOC      ${VATNAO_INCLUDE_DIR}/tabs/cameraTab.hpp)
    # endif(CMAKE_TOOLCHAIN_FILE)

#    set(VATNAO_UI
#            ${VATNAO_SRC_DIR}/visualiser.ui
#            ${VATNAO_SRC_DIR}/ConnectionBar.ui
#            # ${VATNAO_SRC_DIR}/tabs/LogTab.ui
#            # ${VATNAO_SRC_DIR}/tabs/LogsTab.ui
#            )
#
#    set(VATNAO_RES
#            ${VATNAO_RESOURCE_DIR}/visualiser_resources.qrc
#            )

    #####
    # Configure QT5
    # As moc files are generated in the binary dir, tell CMake to always look for includes there
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    # Find QT5 Packages
    find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED
    )

    #SET(QT_USE_QTSVG TRUE) #svg support comes in via plugin when using QImage
    set(QT_USE_QTNETWORK TRUE)
    set(QT_USE_QTOPENGL TRUE)
    set(QT_USE_QTXML TRUE)

    set(QT5_LIBRARIES
        ${Qt5Gui_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
    )

    # build cxx files for resources
#    QT5_ADD_RESOURCES(VATNAO_RES_SRCS ${VATNAO_RES})

    # build ui_XXX files from the XML-style .ui files
#    QT5_WRAP_UI(VATNAO_UI_SRCS ${VATNAO_UI})

    # this moc's the above variable and appends to the cxx sources
    SET(QT5_MOC
            ${VATNAO_INCLUDE_DIR}/ui/vatnaoManager.hpp
            ${VATNAO_INCLUDE_DIR}/ui/uiElements/imageView.hpp
            ${VATNAO_INCLUDE_DIR}/ui/uiElements/option.hpp
            ${VATNAO_INCLUDE_DIR}/ui/uiElements/debugFrame.hpp
            )
    QT5_WRAP_CPP(VATNAO_MOC_SRCS ${QT5_MOC})

    #####
    # Configure OpenGL
    find_package(OpenGL  REQUIRED)

    #####
    # Vatnao Executable
    add_executable(Vatnao
        ${VATNAO_SOURCES}
        # ${VATNAO_RES_SRCS}
        ${VATNAO_MOC_SRCS}
        ${PROTO_SRCS_VATNAO}
        ${PROTO_HDRS_VATNAO}
    )
    set_property(TARGET Vatnao PROPERTY RUNTIME_OUTPUT_DIRECTORY "${VATNAO_OUTPUT_DIR}")
    set_property(TARGET Vatnao PROPERTY RUNTIME_OUTPUT_NAME vatnao)
    set_property(TARGET Vatnao PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
    target_include_directories(Vatnao
        PRIVATE
            ${VATNAO_INCLUDE_DIR}
            ${RBB_INCLUDE_DIR}
            ${OFFNAO_INCLUDE_DIR}
            "${VATNAO_OUTPUT_DIR}"

            # Inlcude Cmake build locations for generated headers
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    # message(STATUS "Qt5Gui_INCLUDES: ${Qt5Gui_INCLUDES}")
    # message(STATUS "Qt5Widgets_INCLUDES: ${Qt5Widgets_INCLUDES}")
    # message(STATUS "QWT_INCLUDE_DIR: ${QWT_INCLUDE_DIR}")

    # The QT slots/signals keywords conflict with Python3 C++ types, so they must be disabled
    target_compile_definitions(Vatnao PRIVATE QT_NO_KEYWORDS)

    # QT5 Libraries
    target_link_libraries(Vatnao PRIVATE Qt5::Core)
    target_link_libraries(Vatnao PRIVATE Qt5::Gui)
    target_link_libraries(Vatnao PRIVATE Qt5::Widgets)

    # RBB Libraries
    target_link_libraries(Vatnao PRIVATE RBBCommonDesktop)

    # External dependent libraries
    target_link_libraries(Vatnao PRIVATE BoostInterface)
    target_link_libraries(Vatnao PRIVATE Boost::ProgramOptions)
    #target_link_libraries(Vatnao PRIVATE Boost::Python)
    target_link_libraries(Vatnao PRIVATE Boost::Python3)
    # target_link_libraries(Vatnao PRIVATE Boost::Regex)
    target_link_libraries(Vatnao PRIVATE Boost::System)
    target_link_libraries(Vatnao PRIVATE Boost::Thread)
    target_link_libraries(Vatnao PRIVATE BZip2::BZip2)
    target_link_libraries(Vatnao PRIVATE Eigen)
    target_link_libraries(Vatnao PRIVATE FadBad)
    target_link_libraries(Vatnao PRIVATE Google::ProtobufVN)
    # target_link_libraries(Vatnao PRIVATE MsgpackInterface)
    target_link_libraries(Vatnao PRIVATE Png::Png)
    #target_link_libraries(Vatnao PRIVATE Python::Python)
    target_link_libraries(Vatnao PRIVATE Python38)
    target_link_libraries(Vatnao PRIVATE SnappyDesktop)
    # target_link_libraries(Vatnao PRIVATE TinyDNN)
    target_link_libraries(Vatnao PRIVATE libjpeg::libjpeg)
    target_link_libraries(Vatnao PRIVATE -lcrypt)
    target_link_libraries(Vatnao PRIVATE -ldl)
    target_link_libraries(Vatnao PRIVATE -lexpat)
    target_link_libraries(Vatnao PRIVATE -lutil)
    target_link_libraries(Vatnao PRIVATE -lz)

    target_compile_definitions(Vatnao PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

    # Some default compile options
    target_compile_options(Vatnao PRIVATE -Wno-switch)
    target_compile_options(Vatnao PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
    target_compile_options(Vatnao PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

    # Supress warnings from Boost (Remove these eventually)
    target_compile_options(Vatnao PRIVATE -Wno-misleading-indentation)
    target_compile_options(Vatnao PRIVATE -Wno-deprecated-declarations)
    target_compile_options(Vatnao PRIVATE -Wno-expansion-to-defined)

    # Supress warnings from Eigen external library
    target_compile_options(Vatnao PRIVATE -Wno-int-in-bool-context)
    target_compile_options(Vatnao PRIVATE -Wno-implicit-int-float-conversion)
    target_compile_options(Vatnao PRIVATE -Wno-deprecated-copy)

    # Supress warnings from TinyDNN
    target_compile_options(Vatnao PRIVATE -Wno-delete-non-abstract-non-virtual-dtor)
    target_compile_options(Vatnao PRIVATE -Wno-shorten-64-to-32)
    target_compile_options(Vatnao PRIVATE -Wno-implicit-int-conversion)

    # Supress additional warnings
    target_compile_options(Vatnao PRIVATE -Wno-implicit-float-conversion)

    # RBB warning temporarily disabled - we should fix these
    target_compile_options(Vatnao PRIVATE -Wno-ignored-qualifiers)
    target_compile_options(Vatnao PRIVATE -Wno-float-conversion)
    target_compile_options(Vatnao PRIVATE -Wno-mismatched-tags)
    target_compile_options(Vatnao PRIVATE -Wno-int-to-void-pointer-cast)
    target_compile_options(Vatnao PRIVATE -Wno-logical-not-parentheses)
    target_compile_options(Vatnao PRIVATE -Wno-pointer-bool-conversion)
    target_compile_options(Vatnao PRIVATE -Wno-dangling-gsl)
    target_compile_options(Vatnao PRIVATE -Wno-tautological-constant-out-of-range-compare)
    target_compile_options(Vatnao PRIVATE -Wno-sizeof-array-div)

    target_link_libraries(Vatnao PRIVATE Flags::Default)

endif()
