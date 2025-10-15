
# RBB Source Files root location
set(NAO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Nao/$<CONFIG>")

if(BUILD_NAO)
  if(NOT (${PLATFORM} STREQUAL Linux AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
    message(ERROR "The target Nao has to be built for Linux and with clang.")
  endif()

  #####
  # Protobuf Configuration
  set(Protobuf_INCLUDE_DIR ${RBB_BUILDCHAIN_DIR}/protobuf/include/)
  set(Protobuf_LIBRARY ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotobuf.so)
  set(Protobuf_PROTOC_LIBRARIES ${RBB_BUILDCHAIN_DIR}/protobuf/lib/${PLATFORM}/libprotoc.so)
  set(Protobuf_PROTOC_EXECUTABLE ${RBB_BUILDCHAIN_DIR}/protobuf/bin/${OS}/protoc)

  # Include Protobuf cmake
  include(${RBB_BUILDCHAIN_DIR}/protobuf/cmake/FindProtobuf.cmake)

  # Generate Protobuf files
  protobuf_generate_cpp(PROTO_SRCS_NAO PROTO_HDRS_NAO
      ${RBB_INCLUDE_DIR}/communication/protobuf/Blackboard.proto
      ${RBB_INCLUDE_DIR}/communication/protobuf/naoData.proto
  )

  # Setup Protobuf library include
  add_library(Google::Protobuf STATIC IMPORTED)
  set_target_properties(Google::Protobuf PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIR}")
  set_target_properties(Google::Protobuf PROPERTIES IMPORTED_LOCATION "${RBB_BUILDCHAIN_DIR}/protobuf/lib/V6/libprotobuf.a")

  #####
  # Robot Sources
  SET(ROBOT_SRCS
    # Communication
    ${RBB_SRC_DIR}/communication/receiver/NaoReceiver.cpp
    ${RBB_SRC_DIR}/communication/receiver/TeamReceiver.cpp
    ${RBB_SRC_DIR}/communication/transmitter/NaoTransmitter.cpp
    ${RBB_SRC_DIR}/communication/transmitter/TeamTransmitter.cpp
  
    # GameController
    ${RBB_SRC_DIR}/gamecontroller/GameController.cpp
    ${RBB_SRC_DIR}/gamecontroller/RoboCupGameControlData.cpp
    
    # Motion
    ${RBB_SRC_DIR}/motion/generator/ActionGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/GetupGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/ClippedGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/BodyModel.cpp
    ${RBB_SRC_DIR}/motion/generator/DistributedGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/HeadGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/NullGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/RefPickupGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/DeadGenerator.cpp
    ${RBB_SRC_DIR}/motion/generator/Walk2014Generator.cpp
    ${RBB_SRC_DIR}/motion/generator/WalkCycle.cpp
    ${RBB_SRC_DIR}/motion/generator/WalkEnginePreProcessor.cpp
    ${RBB_SRC_DIR}/motion/touch/NullTouch.cpp
    ${RBB_SRC_DIR}/motion/touch/FeetState.cpp
    ${RBB_SRC_DIR}/motion/MotionOdometry.cpp

    # Vision
    ${RBB_SRC_DIR}/perception/vision/Vision.cpp
    ${RBB_SRC_DIR}/perception/vision/VisionAdapter.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/NaoCameraProvider.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/NaoCameraV6.cpp
    ${RBB_SRC_DIR}/perception/vision/camera/NaoCameraDefinitions.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/FieldBoundaryDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/ECImageMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/ScanGridMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/ScanLineMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/LineAndCircleMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/IntersectionCandidatesMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/IntersectionsMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/RelativeFieldColorsMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/PenaltyMarkRegionsMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/PenaltyMarkMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/BallSpotsMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/BOPMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/RobotMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/middleinfo/RobotLowerMiddleInfo.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/BallDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/FieldFeatureDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/detector/RobotDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/other/JerseyClassifier.cpp
    ${RBB_SRC_DIR}/perception/vision/referee/RefereeGestureDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/referee/KeypointsDetector.cpp
    ${RBB_SRC_DIR}/perception/vision/other/KinematicsPoseTester.cpp
    ${RBB_SRC_DIR}/perception/vision/other/WriteImage.cpp

    # State Estimation
    ${RBB_SRC_DIR}/perception/stateestimation/StateEstimationAdapter.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/LocaliserTransitioner.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/Localiser.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKF.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/multimodalcmkf/CMKF.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFParams.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFTransitioner.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/egoballtracker/EgoBallTracker.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/egoballtracker/ballcmkf/BallCMKF.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/egoballtracker/ballcmkf/BallCMKFParams.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/teamballtracker/TeamBallTracker.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/teamballtracker/TeamBallTrackerTransitioner.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/teamballtracker/teamballkf/TeamBallKF.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/teamballtracker/teamballkf/TeamBallKFParams.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/teamballtracker/teamballkf/TeamBallKFTransitioner.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/robotfilter/RobotFilter.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/robotfilter/types/GroupedRobots.cpp
    ${RBB_SRC_DIR}/perception/stateestimation/robotfilter/types/RobotObservation.cpp

    # Kinematics
    ${RBB_SRC_DIR}/perception/kinematics/Kinematics.cpp

    # Behaviour
    ${RBB_SRC_DIR}/perception/behaviour/BehaviourAdapter.cpp
    ${RBB_SRC_DIR}/perception/behaviour/BehaviourHelpers.cpp
    ${RBB_SRC_DIR}/perception/behaviour/SafetySkill.cpp
    ${RBB_SRC_DIR}/perception/behaviour/KickCalibrationSkill.cpp
    ${RBB_SRC_DIR}/perception/behaviour/KinematicsCalibrationSkill.cpp
    ${RBB_SRC_DIR}/perception/behaviour/IMUCalibrationSkill.cpp

    # Python
    ${RBB_SRC_DIR}/perception/behaviour/python/PythonSkill.cpp
    ${RBB_SRC_DIR}/perception/behaviour/python/RobotModule.cpp
    ${RBB_SRC_DIR}/perception/behaviour/python/RegisterConverters.cpp

    # Types
    ${RBB_SRC_DIR}/types/BallInfo.cpp
    ${RBB_SRC_DIR}/types/BroadcastData.cpp
    ${RBB_SRC_DIR}/types/FieldFeatureInfo.cpp
    ${RBB_SRC_DIR}/types/File.cpp
    ${RBB_SRC_DIR}/types/RobotVisionInfo.cpp
    ${RBB_SRC_DIR}/types/SharedStateEstimationBundle.cpp
    ${RBB_SRC_DIR}/types/field/FieldFeatureLocations.cpp
    ${RBB_SRC_DIR}/types/math/Geometry.cpp
    ${RBB_SRC_DIR}/types/LabelImage.cpp

    # Whistle
    # ${RBB_SRC_DIR}/whistle/filt.cpp

    # Misc/utils
    ${RBB_SRC_DIR}/utils/Cluster.cpp
    ${RBB_SRC_DIR}/utils/body.cpp
    ${RBB_SRC_DIR}/utils/ml/PatchUtilities.cpp
    ${RBB_SRC_DIR}/utils/ml/Resize.cpp
    ${RBB_SRC_DIR}/utils/math/IISC.cpp
  )

  #####
  # Define Nao V6 only sources
  SET(NAO_SOURCES
    # Perception
    ${RBB_SRC_DIR}/perception/PerceptionThread.cpp

    # Whistle
    ${RBB_SRC_DIR}/whistle/WhistleThread.cpp

    # Motion
    ${RBB_SRC_DIR}/motion/MotionAdapter.cpp
    ${RBB_SRC_DIR}/motion/effector/LoLAEffector.cpp
    ${RBB_SRC_DIR}/motion/touch/LoLATouch.cpp
    ${RBB_SRC_DIR}/motion/LoLAData.cpp

    # Protobuf sources
    ${RBB_SRC_DIR}/communication/serialisation/deserialise.cpp
    ${RBB_SRC_DIR}/communication/serialisation/serialise.cpp
    ${PROTO_SRCS_NAO}
    ${PROTO_HDRS_NAO}

    # Main
    ${RBB_SRC_DIR}/main.cpp
  )

  #####
  # RedBackBots Executable
  add_executable(Nao
    ${ROBOT_SRCS}
    ${NAO_SOURCES}
  )
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_DIRECTORY "${NAO_OUTPUT_DIR}")
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_NAME redbackbots)
  set_property(TARGET Nao PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
  target_include_directories(Nao PRIVATE
    ${RBB_INCLUDE_DIR}
    "${NAO_ROOT_DIR}"

    # Inlcude Cmake build locations for generated headers
    ${CMAKE_CURRENT_BINARY_DIR}
  )


  #####
  # RBB Libraries
  target_link_libraries(Nao PRIVATE RBBCommonNao)

  # External dependent libraries
  target_link_libraries(Nao PRIVATE Nao::ALSA::ALSA)
  target_link_libraries(Nao PRIVATE BoostInterface)
  target_link_libraries(Nao PRIVATE Boost::ProgramOptions)
  target_link_libraries(Nao PRIVATE Boost::Python3)
  target_link_libraries(Nao PRIVATE Boost::Regex)
  target_link_libraries(Nao PRIVATE Boost::System)
  target_link_libraries(Nao PRIVATE Boost::Thread)
  target_link_libraries(Nao PRIVATE Eigen)
  target_link_libraries(Nao PRIVATE CompiledNN)
  target_link_libraries(Nao PRIVATE CompiledNNasmjit)
  target_link_libraries(Nao PRIVATE CompiledNN::ONNXNao)
  target_link_libraries(Nao PRIVATE FadBad)
  target_link_libraries(Nao PRIVATE hdf5) 
  target_link_libraries(Nao PRIVATE opencv)
  target_link_libraries(Nao PRIVATE kissfft)
  target_link_libraries(Nao PRIVATE Google::Protobuf)
  target_link_libraries(Nao PRIVATE MsgpackInterface)
  target_link_libraries(Nao PRIVATE Png::Png)
  target_link_libraries(Nao PRIVATE PortAudio)
  target_link_libraries(Nao PRIVATE Python38)
  target_link_libraries(Nao PRIVATE SnappyNao)
  target_link_libraries(Nao PRIVATE TinyDNN)
  target_link_libraries(Nao PRIVATE libexpat::libexpat)
  target_link_libraries(Nao PRIVATE libjpeg::libjpeg)
  target_link_libraries(Nao PRIVATE -lcrypt)
  target_link_libraries(Nao PRIVATE -ldl-2.31)
  target_link_libraries(Nao PRIVATE -lpthread-2.31)
  target_link_libraries(Nao PRIVATE -lrt-2.31)
  target_link_libraries(Nao PRIVATE -lutil)
  # target_link_libraries(Nao PRIVATE -lz)

  # Make all symbols exportable to dynamically loading shared libs (.so)
  # For example, this is required by python numpy
  target_link_options(Nao PRIVATE -Bdynamic)
  target_link_options(Nao PRIVATE --export-dynamic)

  target_compile_definitions(Nao PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

  # Some default compile options
  target_compile_options(Nao PRIVATE -Wno-switch)
  target_compile_options(Nao PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_compile_options(Nao PRIVATE $<$<CONFIG:Release>:-Wno-unused>)

  # Supress errors from Eigen external library
  target_compile_options(Nao PRIVATE -Wno-int-in-bool-context)
  target_compile_options(Nao PRIVATE -Wno-implicit-int-float-conversion)
  target_compile_options(Nao PRIVATE -Wno-deprecated-copy)

  # Supress warnings from TinyDNN
  target_compile_options(Nao PRIVATE -Wno-delete-non-abstract-non-virtual-dtor)
  target_compile_options(Nao PRIVATE -Wno-shorten-64-to-32)
  target_compile_options(Nao PRIVATE -Wno-implicit-int-conversion)

  # Supress additional warnings
  target_compile_options(Nao PRIVATE -Wno-implicit-float-conversion)

  # RBB warning temporarily disabled - we should fix these
  target_compile_options(Nao PRIVATE -Wno-ignored-qualifiers)
  target_compile_options(Nao PRIVATE -Wno-float-conversion)
  target_compile_options(Nao PRIVATE -Wno-mismatched-tags)
  target_compile_options(Nao PRIVATE -Wno-int-to-void-pointer-cast)
  target_compile_options(Nao PRIVATE -Wno-missing-field-initializers)
  target_compile_options(Nao PRIVATE -Wno-literal-conversion)
  target_compile_options(Nao PRIVATE -Wno-pointer-bool-conversion)

  target_link_libraries(Nao PRIVATE Flags::Default)

else()
  # Define an external project to complete the Nao build
  include(ExternalProject)
  ExternalProject_Add(Nao
      SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
      CMAKE_ARGS -DCMAKE_BUILD_TYPE=$<CONFIG> -DBUILD_NAO=ON
      BUILD_ALWAYS 1
      USES_TERMINAL_BUILD ON
      INSTALL_COMMAND "")
  set_property(TARGET Nao PROPERTY FOLDER "")
endif()
