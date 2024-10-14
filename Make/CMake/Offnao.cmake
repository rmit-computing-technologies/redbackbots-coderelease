
##### 
# RBB Source Files root location
set(OFFNAO_ROOT_DIR "${RBB_PREFIX}/Src/offnao")
set(OFFNAO_INCLUDE_DIR "${OFFNAO_ROOT_DIR}/include")
set(OFFNAO_RESOURCE_DIR "${OFFNAO_ROOT_DIR}/resources")
set(OFFNAO_SRC_DIR "${OFFNAO_ROOT_DIR}/src")
set(OFFNAO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Offnao/$<CONFIG>")

if(BUILD_DESKTOP)
  #############################
  # Protobuf Configuration
  set(Protobuf_INCLUDE_DIR ${RBB_BUILDCHAIN_DIR}/protobuf/include/)
  set(Protobuf_LIBRARY ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotobuf.so)
  set(Protobuf_PROTOC_LIBRARIES ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotoc.so)
  set(Protobuf_PROTOC_EXECUTABLE ${RBB_BUILDCHAIN_DIR}/protobuf/bin/${OS}/protoc)

  # Include Protobuf cmake
  include(${RBB_BUILDCHAIN_DIR}/protobuf/cmake/FindProtobuf.cmake)
  add_library(Google::Protobuf STATIC IMPORTED)
  set_target_properties(Google::Protobuf PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIR}")
  set_target_properties(Google::Protobuf PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotobuf.a")

  # Generate Protobuf files
  protobuf_generate_cpp(PROTO_SRCS_OFFNAO PROTO_HDRS_OFFNAO
      ${RBB_INCLUDE_DIR}/blackboard/Blackboard.proto
      ${RBB_INCLUDE_DIR}/blackboard/naoData.proto
  )
  # message (STATUS "Generated Protobuf Srcs: ${PROTO_SRCS}")
  # message (STATUS "Generated Protobuf Headers: ${PROTO_HDRS}")
  # message (STATUS "Protobuf include dirs Headers: ${Protobuf_INCLUDE_DIRS}")

  #####
  # Define Offnao only source files
  SET(OFFNAO_SOURCES
    # Main
    ${OFFNAO_SRC_DIR}/main.cpp

    # Panels
    ${OFFNAO_SRC_DIR}/mediaPanel.cpp
    ${OFFNAO_SRC_DIR}/visualiser.cpp
    
    # Readers
    ${OFFNAO_SRC_DIR}/readers/dumpReader.cpp
    ${OFFNAO_SRC_DIR}/readers/networkReader.cpp
    ${OFFNAO_SRC_DIR}/readers/reader.cpp
    ${OFFNAO_SRC_DIR}/readers/record2Reader.cpp
    ${OFFNAO_SRC_DIR}/readers/bbd2Reader.cpp
    
    # Other utilities
    ${OFFNAO_SRC_DIR}/utils/OverlayPainter.cpp
    ${OFFNAO_SRC_DIR}/utils/FieldPainter.cpp
    ${OFFNAO_SRC_DIR}/utils/CPlaneColours.cpp
    ${OFFNAO_SRC_DIR}/utils/classifier.cpp

    # Offnao Tabs
    # ${OFFNAO_SRC_DIR}/tabs/aroundFeetTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/cameraTab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/cameraPoseTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/fieldView.cpp
    # ${OFFNAO_SRC_DIR}/tabs/graphTab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/jointsTab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/LogsTab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/LogTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/overviewTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/PointCloud.cpp
    # ${OFFNAO_SRC_DIR}/tabs/plots.cpp
    # ${OFFNAO_SRC_DIR}/tabs/sensorTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/tab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/temperatureTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/variableView.cpp
    # ${OFFNAO_SRC_DIR}/tabs/visionTab.cpp
    # ${OFFNAO_SRC_DIR}/tabs/walkTab.cpp
    ${OFFNAO_SRC_DIR}/tabs/yuvHistogram.cpp
    # ${OFFNAO_SRC_DIR}/tabs/zmpTab.cpp

    # Protobuf source files
    # Required both serialise from Robot and Offnao
    ${RBB_SRC_DIR}/blackboard/serialise.cpp
    ${OFFNAO_SRC_DIR}/serialise.cpp
    ${PROTO_SRCS_OFFNAO}
    ${PROTO_HDRS_OFFNAO}
  )

  set(OFFNAO_MOC
    ${OFFNAO_INCLUDE_DIR}/readers/reader.hpp
    ${OFFNAO_INCLUDE_DIR}/readers/dumpReader.hpp
    ${OFFNAO_INCLUDE_DIR}/readers/record2Reader.hpp
    ${OFFNAO_INCLUDE_DIR}/readers/bbd2Reader.hpp
    ${OFFNAO_INCLUDE_DIR}/mediaPanel.hpp
    ${OFFNAO_INCLUDE_DIR}/visualiser.hpp
    
    # ${OFFNAO_INCLUDE_DIR}/tabs/aroundFeetTab.hpp
    ${OFFNAO_INCLUDE_DIR}/tabs/cameraTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/cameraPoseTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/graphTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/jointsTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/LogsTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/LogTab.hpp
    ${OFFNAO_INCLUDE_DIR}/tabs/overviewTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/plots.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/sensorTab.hpp
    ${OFFNAO_INCLUDE_DIR}/tabs/tab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/temperatureTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/visionTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/walkTab.hpp
    # ${OFFNAO_INCLUDE_DIR}/tabs/zmpTab.hpp
  )

  # if(CMAKE_TOOLCHAIN_FILE)
  #   list(APPEND OFFNAO_CXX_SRCS ${OFFNAO_INCLUDE_DIR}/tabs/cameraTab.cpp)
  #   list(APPEND OFFNAO_MOC      ${OFFNAO_INCLUDE_DIR}/tabs/cameraTab.hpp)
  # endif(CMAKE_TOOLCHAIN_FILE)

  set(OFFNAO_UI
    ${OFFNAO_SRC_DIR}/visualiser.ui
    ${OFFNAO_SRC_DIR}/ConnectionBar.ui
    # ${OFFNAO_SRC_DIR}/tabs/LogTab.ui
    # ${OFFNAO_SRC_DIR}/tabs/LogsTab.ui
  )

  set(OFFNAO_RES
    ${OFFNAO_RESOURCE_DIR}/visualiser_resources.qrc
  )

  ##### 
  # Configure QT5
  # As moc files are generated in the binary dir, tell CMake to always look for includes there
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  # Find QT5 Packages
  find_package(Qt5 COMPONENTS 
    Core
    Gui
    OpenGL
    Widgets
    Xml
    REQUIRED
  )
  find_library ( QGLVIEWER_LIBRARY NAMES QGLViewer qglviewer-qt5 QGLViewer-qt5 )
  #message(${QGLVIEWER_LIBRARY})

  #SET(QT_USE_QTSVG TRUE) #svg support comes in via plugin when using QImage
  set(QT_USE_QTNETWORK TRUE)
  set(QT_USE_QTOPENGL TRUE)
  set(QT_USE_QTXML TRUE)

  set(QT5_LIBRARIES
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
  )

  # build cxx files for resources
  QT5_ADD_RESOURCES(OFFNAO_RES_SRCS ${OFFNAO_RES})

  # build ui_XXX files from the XML-style .ui files
  QT5_WRAP_UI(OFFNAO_UI_SRCS ${OFFNAO_UI})

  # this moc's the above variable and appends to the cxx sources
  QT5_WRAP_CPP(OFFNAO_MOC_SRCS ${OFFNAO_MOC})


  #####
  # Configure OpenGL
  # find_package(OpenGL  REQUIRED)
  find_package(OpenGL REQUIRED COMPONENTS
    OpenGL
  )

  #####
  # Offnao Executable
  add_executable(Offnao
    ${OFFNAO_SOURCES}
    ${OFFNAO_RES_SRCS}
    ${OFFNAO_MOC_SRCS}
    ${OFFNAO_UI_SRCS}
  )
  set_property(TARGET Offnao PROPERTY RUNTIME_OUTPUT_DIRECTORY "${OFFNAO_OUTPUT_DIR}")
  set_property(TARGET Offnao PROPERTY RUNTIME_OUTPUT_NAME offnao)
  set_property(TARGET Offnao PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
  target_include_directories(Offnao 
    PRIVATE
      ${OFFNAO_INCLUDE_DIR}
      ${RBB_INCLUDE_DIR}
      "${OFFNAO_OUTPUT_DIR}"

      # Inlcude Cmake build locations for generated headers
      ${CMAKE_CURRENT_BINARY_DIR}
  )
  # message(STATUS "Qt5Gui_INCLUDES: ${Qt5Gui_INCLUDES}")
  # message(STATUS "Qt5Widgets_INCLUDES: ${Qt5Widgets_INCLUDES}")
  # message(STATUS "QWT_INCLUDE_DIR: ${QWT_INCLUDE_DIR}")

  # QT5 Libraries
  target_link_libraries(Offnao PRIVATE Qt5::Core)
  target_link_libraries(Offnao PRIVATE Qt5::Gui)
  target_link_libraries(Offnao PRIVATE Qt5::OpenGL)
  target_link_libraries(Offnao PRIVATE Qt5::Widgets)
  target_link_libraries(Offnao PRIVATE Qt5::Xml)
  target_link_libraries(Offnao PRIVATE ${QGLVIEWER_LIBRARY})

  # The QT slots/signals keywords conflict with Python3 C++ types, so they must be disabled
  target_compile_definitions(Offnao PRIVATE QT_NO_KEYWORDS)

  # RBB Libraries
  target_link_libraries(Offnao PRIVATE RBBCommonDesktop)

  # External dependent libraries
  target_link_libraries(Offnao PRIVATE BoostInterface)
  target_link_libraries(Offnao PRIVATE Boost::ProgramOptions)
  #target_link_libraries(Offnao PRIVATE Boost::Python)
  target_link_libraries(Offnao PRIVATE Boost::Python3)
  # target_link_libraries(Offnao PRIVATE Boost::Regex)
  target_link_libraries(Offnao PRIVATE Boost::System)
  target_link_libraries(Offnao PRIVATE Boost::Thread)
  target_link_libraries(Offnao PRIVATE BZip2::BZip2)
  target_link_libraries(Offnao PRIVATE Eigen)
  target_link_libraries(Offnao PRIVATE FadBad)
  target_link_libraries(Offnao PRIVATE Google::Protobuf)
  # target_link_libraries(Offnao PRIVATE MsgpackInterface)
  target_link_libraries(Offnao PRIVATE OpenGL::GL)
  target_link_libraries(Offnao PRIVATE Png::Png)
  #target_link_libraries(Offnao PRIVATE Python::Python)
  target_link_libraries(Offnao PRIVATE Python38)
  target_link_libraries(Offnao PRIVATE SnappyDesktop)
  # target_link_libraries(Offnao PRIVATE TinyDNN)
  target_link_libraries(Offnao PRIVATE libjpeg::libjpeg)
  target_link_libraries(Offnao PRIVATE -lcrypt)
  target_link_libraries(Offnao PRIVATE -ldl)
  target_link_libraries(Offnao PRIVATE -lexpat)
  target_link_libraries(Offnao PRIVATE -lutil)
  # target_link_libraries(Offnao PRIVATE -lz)

  target_compile_definitions(Offnao PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

  # Some default compile options
  target_compile_options(Offnao PRIVATE -Wno-switch)
  target_compile_options(Offnao PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_compile_options(Offnao PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

  # Supress warnings from Boost (Remove these eventually)
  target_compile_options(Offnao PRIVATE -Wno-misleading-indentation)
  target_compile_options(Offnao PRIVATE -Wno-deprecated-declarations)
  target_compile_options(Offnao PRIVATE -Wno-expansion-to-defined)

  # Supress warnings from Eigen external library
  target_compile_options(Offnao PRIVATE -Wno-int-in-bool-context)
  target_compile_options(Offnao PRIVATE -Wno-implicit-int-float-conversion)
  target_compile_options(Offnao PRIVATE -Wno-deprecated-copy)

  # Supress warnings from TinyDNN
  target_compile_options(Offnao PRIVATE -Wno-delete-non-abstract-non-virtual-dtor)
  target_compile_options(Offnao PRIVATE -Wno-shorten-64-to-32)
  target_compile_options(Offnao PRIVATE -Wno-implicit-int-conversion)

  # Supress additional warnings
  target_compile_options(Offnao PRIVATE -Wno-implicit-float-conversion)

  # RBB warning temporarily disabled - we should fix these
  target_compile_options(Offnao PRIVATE -Wno-ignored-qualifiers)
  target_compile_options(Offnao PRIVATE -Wno-float-conversion)
  target_compile_options(Offnao PRIVATE -Wno-mismatched-tags)
  target_compile_options(Offnao PRIVATE -Wno-int-to-void-pointer-cast)
  target_compile_options(Offnao PRIVATE -Wno-logical-not-parentheses)
  target_compile_options(Offnao PRIVATE -Wno-pointer-bool-conversion)
  target_compile_options(Offnao PRIVATE -Wno-dangling-gsl)
  target_compile_options(Offnao PRIVATE -Wno-tautological-constant-out-of-range-compare)
  target_compile_options(Offnao PRIVATE -Wno-sizeof-array-div)

  target_link_libraries(Offnao PRIVATE Flags::Default)

endif()
