cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# RBB Source Files root location
set(RBB_ROOT_DIR "${RBB_PREFIX}/Src/robot")
set(RBB_INCLUDE_DIR "${RBB_ROOT_DIR}/include")
set(RBB_SRC_DIR "${RBB_ROOT_DIR}/src")

#############################
# Add all source files for robot library
# These sources files are configered depending on how much we need to build into librobot
SET(ROBOT_COMMON_SRCS
    ${RBB_SRC_DIR}/soccer.cpp

    # Blackboard
    ${RBB_SRC_DIR}/blackboard/Blackboard.cpp

    # Thread
    ${RBB_SRC_DIR}/thread/Thread.cpp
    ${RBB_SRC_DIR}/thread/ThreadManager.cpp

    # Kinematics
    ${RBB_SRC_DIR}/perception/kinematics/Pose.cpp
    ${RBB_SRC_DIR}/perception/kinematics/Parameters.cpp

    # Vision
    ${RBB_SRC_DIR}/perception/vision/Vision.cpp
    ${RBB_SRC_DIR}/perception/vision/Fovea.cpp
    ${RBB_SRC_DIR}/perception/vision/Region.cpp
    ${RBB_SRC_DIR}/perception/vision/VisionAdapter.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/Camera.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/CameraToRR.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/CombinedCamera.cpp
    ${RBB_SRC_DIR}/perception/vision/other/YUV.cpp
    ${RBB_SRC_DIR}/perception/vision/other/Ransac.cpp
    ${RBB_SRC_DIR}/perception/vision/other/GMM_classifier.cpp
    ${RBB_SRC_DIR}/perception/vision/regionfinder/ColourROI.cpp
    ${RBB_SRC_DIR}/perception/vision/regionfinder/RobotColorROI.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/BallDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/RegionFieldFeatureDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/RobotDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/DNNHelper.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfoprocessor/FieldBoundaryFinder.cpp

    # Types
    ${RBB_SRC_DIR}/types/BehaviourSharedData.cpp
    ${RBB_SRC_DIR}/types/ButtonPresses.cpp
    ${RBB_SRC_DIR}/types/CameraSettings.cpp

    # Misc/utils
    ${RBB_SRC_DIR}/transmitter/OffNao.cpp
    ${RBB_SRC_DIR}/utils/Connection.cpp
    ${RBB_SRC_DIR}/utils/LeastSquaresLine.cpp
    ${RBB_SRC_DIR}/utils/Logger.cpp
    ${RBB_SRC_DIR}/utils/options.cpp
    ${RBB_SRC_DIR}/utils/speech.cpp
    ${RBB_SRC_DIR}/utils/Timer.cpp
)

#############################
# Create a different Library for Nao and Desktop buils
if (BUILD_NAO)
    set(RBBCommonName "RBBCommonNao")
elseif (BUILD_DESKTOP)
    set(RBBCommonName "RBBCommonDesktop")
else()
    message(FATAL_ERROR "Cannot determine a library type for RBBCommon")
endif()

# RebBackBots Common library
add_library(${RBBCommonName}
    STATIC 
        ${ROBOT_COMMON_SRCS}
)
target_include_directories(${RBBCommonName}
    PUBLIC ${RBB_INCLUDE_DIR}
)
# External dependent libraries
target_link_libraries(${RBBCommonName} PRIVATE BoostInterface)
target_link_libraries(${RBBCommonName} PRIVATE Boost::ProgramOptions)
#target_link_libraries(${RBBCommonName} PRIVATE Boost::Python)
target_link_libraries(${RBBCommonName} PRIVATE Boost::Python3)
target_link_libraries(${RBBCommonName} PRIVATE Boost::Regex)
target_link_libraries(${RBBCommonName} PRIVATE Boost::System)
target_link_libraries(${RBBCommonName} PRIVATE Boost::Thread)
target_link_libraries(${RBBCommonName} PRIVATE Eigen)
target_link_libraries(${RBBCommonName} PRIVATE FadBad)
target_link_libraries(${RBBCommonName} PRIVATE MsgpackInterface)
target_link_libraries(${RBBCommonName} PRIVATE Png::Png)
if(BUILD_NAO)
    # target_link_libraries(${RBBCommonName} PRIVATE Python27)
    target_link_libraries(${RBBCommonName} PRIVATE Python38)
    target_link_libraries(${RBBCommonName} PRIVATE SnappyNao)
    target_link_libraries(${RBBCommonName} PRIVATE -ldl-2.31)
    target_link_libraries(${RBBCommonName} PRIVATE -lpthread-2.31)
    target_link_libraries(${RBBCommonName} PRIVATE -lrt-2.31)
elseif(BUILD_DESKTOP)
    #target_link_libraries(${RBBCommonName} PRIVATE Python::Python)
    target_link_libraries(${RBBCommonName} PRIVATE Python38)
    target_link_libraries(${RBBCommonName} PRIVATE SnappyDesktop)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${RBBCommonName} PRIVATE Threads::Threads)
endif()
target_link_libraries(${RBBCommonName} PRIVATE TinyDNN)
target_link_libraries(${RBBCommonName} PRIVATE -lcrypt)
target_link_libraries(${RBBCommonName} PRIVATE -lutil)
target_link_libraries(${RBBCommonName} PRIVATE -lz)

target_compile_definitions(${RBBCommonName} PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

# Some default compile options
target_compile_options(${RBBCommonName} PRIVATE -Wno-switch)
target_compile_options(${RBBCommonName} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
target_compile_options(${RBBCommonName} PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

# Supress warnings from Eigen external library
target_compile_options(${RBBCommonName} PRIVATE -Wno-int-in-bool-context)
target_compile_options(${RBBCommonName} PRIVATE -Wno-implicit-int-float-conversion)
target_compile_options(${RBBCommonName} PRIVATE -Wno-deprecated-copy)

# Supress warnings from TinyDNN
target_compile_options(${RBBCommonName} PRIVATE -Wno-delete-non-abstract-non-virtual-dtor)
target_compile_options(${RBBCommonName} PRIVATE -Wno-shorten-64-to-32)
target_compile_options(${RBBCommonName} PRIVATE -Wno-implicit-int-conversion)

# Supress additional warnings
target_compile_options(${RBBCommonName} PRIVATE -Wno-implicit-float-conversion)

# RBB warning temporarily disabled - we should fix these
target_compile_options(${RBBCommonName} PRIVATE -Wno-ignored-qualifiers)
target_compile_options(${RBBCommonName} PRIVATE -Wno-float-conversion)

target_link_libraries(${RBBCommonName} PRIVATE Flags::Default)

# Include dirs
set_target_properties(${RBBCommonName} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RBB_INCLUDE_DIR}")
