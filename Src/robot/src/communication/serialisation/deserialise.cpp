/**
 * @file serialise.cpp
 * 
 * Serialise side of Protobuf communication.
 * This is kept as one file to keep seralisation disentangled from each data structure.
 * 
 * @author rUNSWift
 *         RedbackBots
 */


#include <algorithm>
#include <fstream>
#include <iostream>
#include <jpeglib.h>
#include <type_traits>

#include "blackboard/Blackboard.hpp"

// Blackboard modules
#include "blackboard/modulesList.hpp"

// Additional serialisable objects
#include "types/serialise/CameraSettingsSerialse.hpp"

// Generated file from Protobuf
#include "Blackboard.pb.h"

// Friend classes
#include "communication/serialisation/SerialiseImage.hpp"

// Debugger types
#ifdef TMP_NDEBUG
#else 
   #include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#endif

using namespace std;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Deseralise functions


// Forward Declarations
static void deserialise(RobotObstacle &, const offnao::StateEstimation_RobotObstacle &);

static void deserialise(AbsCoord &, const offnao::AbsCoord &);

static void deserialise(BehaviourRequest &, const offnao::Behaviour_BehaviourRequest &);

static void deserialise(BallInfo &, const offnao::Vision_BallInfo &);

static void deserialise(RobotVisionInfo &, const offnao::Vision_RobotVisionInfo &);

static void deserialise(FieldFeatureInfo &, const offnao::Vision_FieldFeatureInfo &);

static void deserialise(SPLStandardMessage &, const offnao::Receiver_SPLStandardMessage &);

static void deserialise(BroadcastData &, const offnao::Receiver_BroadcastData &);

static void deserialise(BehaviourDebugInfo &, const offnao::BehaviourDebugInfo &);

static void deserialise(BehaviourSharedData &, const offnao::BehaviourSharedData &);

#ifdef TMP_NDEBUG
#else 
static void deserialise(ScanGrid::ScanGridLine &, const offnao::Debugger_VisionDebugger_ScanGrid_ScanGridLine &);
static void deserialise(ScanGrid &, const offnao::Debugger_VisionDebugger_ScanGrid &);
#endif

template<typename T>
static void deserialise(T &cpp, const offnao::RepeatedInt &pb);

template<typename T>
static void deserialise(T &cpp, const offnao::RepeatedFloat &pb);

static void deserialise(boost::numeric::ublas::matrix<float> &cpp, const offnao::FloatMatrix &pb);

template<typename T, int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, const google::protobuf::RepeatedField<T> &pb);

// Friend Serialisation Classes
template <typename T>
void SerialiseImage::deserialise(Image<T> &cpp, const offnao::Vision_CameraImage &pb) {
   if (pb.has_data()) {
      const string &pbField  = pb.data();
      cpp.setResolution(pb.width(), pb.height());

      size_t pbSize  = pbField.size();
      size_t cppSize = sizeof(T[cpp.height * cpp.width]);

      if (cppSize < pbSize) {
         llog(ERROR) << "Image:: trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << std::endl;
      }
      memcpy(cpp.image, pb.data().data(), min(cppSize, pbSize));
      // llog(INFO) << "Image:: deserialise " << cpp.width << " x " << cpp.height 
      //            << " size: " << pbSize << " (" << cppSize << ") " << std::endl;
   }
}

// Implementations
template<typename T>
static void deserialise(T &cpp, const T &pb) {
   cpp = pb;
}

static void deserialise(uint8_t &cpp, const uint32_t &pb) {
   cpp = (uint8_t) pb;
}

static void deserialise(int16_t &cpp, const int32_t &pb) {
   cpp = (int16_t) pb;
}

static void deserialise(time_t &cpp, const int32_t &pb) {
   cpp = (time_t) pb;
}

static void deserialise(uint16_t &cpp, const google::protobuf::uint32 &pb) {
   cpp = (uint16_t) pb;
}

// Generic ones for enums
template<typename T,
      typename std::enable_if<std::is_enum<T>::value>::type * = nullptr>
static void deserialise(T &cpp, const int &pb) {
   cpp = static_cast<T>(pb);
}

template<typename T,
         typename U,
      typename std::enable_if<std::is_enum<T>::value>::type * = nullptr,
      typename std::enable_if<std::is_enum<U>::value>::type * = nullptr>
static void deserialise(T &cpp, const U &pb) {
   cpp = static_cast<T>(pb);
}

template<size_t fixedSize>
static void deserialise(char (&cpp)[fixedSize], const string &pb) {
   size_t size = fixedSize;
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << std::endl;
      size = pb.size();
   }
   pb.copy(cpp, size);
}

template<typename T, typename Container>
static void deserialise(Container &cpp, const ::google::protobuf::RepeatedField<T> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << std::endl;
      size = pb.size();
   }
   for (int i = 0; i < size; ++i) {
      deserialise(cpp[i], pb.Get(i));
   }
}

template<typename T, size_t size>
static void deserialise(T (&cpp)[size], const ::google::protobuf::RepeatedField<T> &pb) {
   deserialise(cpp, pb, size);
}

// different name so it's not used accidentally
template<typename T, typename U, int size>
static void deserialiseWithImplicitCast(T(&cpp)[size], const ::google::protobuf::RepeatedField<U> &pb) {
   deserialise(cpp, pb, size);
}

// vector<bool>::data() returns void because it's a bitset
static void deserialise(vector<bool> &cpp, const ::google::protobuf::RepeatedField<bool> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << std::endl;
      size = pb.size();
   }
   cpp.resize(static_cast<unsigned int>(size));
   for (int i = 0; i < size; ++i) {
      cpp[i] = pb.Get(i);
   }
}

template<typename T>
static void deserialise(vector<T> &vector, const ::google::protobuf::RepeatedField<T> &repeatedField, int size) {
   vector.resize(static_cast<unsigned int>(size));
   T *cpp = vector.data();
   deserialise(cpp, repeatedField, size);
}

template<typename T, std::size_t N>
static void deserialise(std::array<T, N> &array, const ::google::protobuf::RepeatedField<T> &repeatedField, int size) {
   T *cpp = array.data();
   deserialise(cpp, repeatedField, size);
}

template<typename T, int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, const google::protobuf::RepeatedField<T> &pb) {
   T tmp[size1_Rows * size2_Cols];
   deserialise(tmp, pb);
   cpp = Eigen::Map<Eigen::Matrix<T, size1_Rows, size2_Cols> >(tmp);
}

// template<typename T, typename Container>
// static void deserialise(Container &cpp, const ::google::protobuf::RepeatedField<T> &pb, int size) {
//    if (pb.size() < size) {
//       llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
//                   << " elements" << std::endl;
//       size = pb.size();
//    }
//    for (int i = 0; i < size; ++i) {
//       deserialise(cpp[i], pb.Get(i));
//    }
// }

template<typename Container, typename U>
static void deserialise(Container& cpp, const ::google::protobuf::RepeatedPtrField<U> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << std::endl;
      size = pb.size();
   }
   for (int i = 0; i < size; ++i) {
      deserialise(cpp[i], pb.Get(i));
   }
}

template<typename T, typename U>
static void deserialise(T *array, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   deserialise(array, repeatedPtrField, repeatedPtrField.size());
}

// TW: the below comment is from runswift.
// I'm not sure how to overload the one above such that this one takes precendence.  Would be nice if we used it more often, but I'm too lazy to find all the fixed-size arrays.
template<typename T, typename U, int size>
static void deserialiseArray(T(&cpp)[size], const ::google::protobuf::RepeatedPtrField<U> &pb) {
   deserialise(cpp, pb, size);
}

template<typename T, typename U>
static void deserialise(vector<T> &vector, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   vector.resize(static_cast<unsigned int>(repeatedPtrField.size()));
   deserialise(vector.data(), repeatedPtrField);
}

template<typename T, std::size_t N, typename U>
static void deserialise(std::array<T, N> &array, 
                        const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   deserialise(array, repeatedPtrField, array.size());
}

static void deserialise(pair<int, int> &cpp, const offnao::PairIntInt &pb) {
   cpp.first  = pb.first();
   cpp.second = pb.second();
}

template<typename T>
static void deserialise(T &cpp, const offnao::RepeatedInt &pb) {
   deserialise(cpp, pb.values());
}

template<typename T>
static void deserialise(T &cpp, const offnao::RepeatedFloat &pb) {
   deserialise(cpp, pb.values());
}


template<int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<uint8_t, size1_Rows, size2_Cols, Eigen::RowMajor> &cpp, 
                        const offnao::UIntMatrix &pb) {
   if (size1_Rows != pb.size1() || size2_Cols != pb.size2()) {
      llog(ERROR) << "trying to deserialise " << pb.size1() << "x" << pb.size2() << " elements to a field of "
                  << size1_Rows << "x" << size2_Cols << " elements" << std::endl;
   }
   int tmp[size1_Rows * size2_Cols];
   deserialise(tmp, pb.data());
   for (int i = 0; i != size1_Rows * size2_Cols; ++i) {
      cpp(i) = (uint8_t) tmp[i];
   }
}

static void
deserialise(boost::numeric::ublas::matrix<float> &cpp, const offnao::FloatMatrix &pb) {
   cpp.resize(pb.size1(), pb.size2());
   deserialise(cpp.data(), pb.data(), pb.size1() * pb.size2());
}

template<int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<float, size1_Rows, size2_Cols> &cpp, const offnao::FloatMatrix &pb) {
   if (size1_Rows != pb.size1() || size2_Cols != pb.size2()) {
      llog(ERROR) << "trying to deserialise " << pb.size1() << "x" << pb.size2() << " elements to a field of "
                  << size1_Rows << "x" << size2_Cols << " elements" << std::endl;
   }
   float tmp[size1_Rows * size2_Cols];
   deserialise(tmp, pb.data());
   cpp = Eigen::Map<Eigen::Matrix<float, size1_Rows, size2_Cols> >(tmp);
}

static void deserialise(CameraSettings &cpp, const offnao::CameraSettings &pb) {
   deserialise(cpp.settings, pb.settings(), CameraSettings::CameraSetting::NUM_CAMERA_SETTINGS);

   cpp.hflip                 = pb.hflip();
   cpp.vflip                 = pb.vflip();
}

static void deserialise(JointValues &cpp, const offnao::JointValues &pb) {
   deserialise(cpp.angles, pb.angles());
   deserialise(cpp.stiffnesses, pb.stiffnesses());
   deserialise(cpp.temperatures, pb.temperatures());
   deserialise(cpp.currents, pb.currents());
}

static void deserialise(SensorValues &cpp, const offnao::SensorValues &pb) {
   deserialise(cpp.joints, pb.joints());
   deserialise(cpp.sensors, pb.sensors());
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void deserialise(RobotPose &cpp, const offnao::Motion_Pose &pb) {
   deserialise(cpp.topCameraToWorldTransform, pb.topcameratoworldtransform());
   deserialise(cpp.botCameraToWorldTransform, pb.botcameratoworldtransform());
   deserialise(cpp.neckToWorldTransform, pb.necktoworldtransform());
   deserialise(cpp.origin, pb.origin());
   deserialise(cpp.zunit, pb.zunit());
   deserialise(cpp.topCOrigin, pb.topcorigin());
   deserialise(cpp.botCOrigin, pb.botcorigin());
   deserialise(cpp.horizon, pb.horizon());
   deserialiseWithImplicitCast(cpp.topExclusionArray, pb.topexclusionarray());
   deserialiseWithImplicitCast(cpp.botExclusionArray, pb.botexclusionarray());
   if (pb.has_necktoworldtransform()) {
      cpp.makeConstants();
   }
}

void deserialise(Odometry &cpp, const offnao::Motion_Odometry &pb) {
   cpp.forward = pb.forward();
   cpp.left    = pb.left();
   cpp.turn    = pb.turn();
}

static void deserialise(XYZ_Coord &cpp, const offnao::XYZ_Coord &pb) {
   cpp.x = pb.x();
   cpp.y = pb.y();
   cpp.z = pb.z();
}

static void deserialise(ActionCommand::Head &cpp, const offnao::ActionCommandAll_Head &pb) {
   deserialise(cpp.yaw, pb.yaw());
   deserialise(cpp.pitch, pb.pitch());
   deserialise(cpp.isRelative, pb.isrelative());
   deserialise(cpp.yawSpeed, pb.yawspeed());
   deserialise(cpp.pitchSpeed, pb.pitchspeed());
}

static void deserialise(ActionCommand::Body &cpp, const offnao::ActionCommandAll_Body &pb) {
   deserialise(cpp.actionType, pb.actiontype());
   deserialise(cpp.forward, pb.forward());
   deserialise(cpp.left, pb.left());
   deserialise(cpp.turn, pb.turn());
   deserialise(cpp.power, pb.power());
}

static void deserialise(ActionCommand::rgb &cpp, const offnao::ActionCommandAll_LED_rgb &pb) {
   cpp.red   = pb.red();
   cpp.green = pb.green();
   cpp.blue  = pb.blue();
}

static void deserialise(ActionCommand::rgbSegments &cpp, const offnao::ActionCommandAll_LED_rgbSegments &pb) {
   cpp = ActionCommand::rgbSegments(); 
   for (int i = 0; i < pb.segments_size(); ++i) {
      const offnao::ActionCommandAll_LED_rgb &pbSegment = pb.segments(i);

      ActionCommand::rgb segment;

      segment.red = pbSegment.red();
      segment.green = pbSegment.green();
      segment.blue = pbSegment.blue();
      cpp.segments[i] = segment;
   }
}

static void deserialise(ActionCommand::LED &cpp, const offnao::ActionCommandAll_LED &pb) {
   deserialise(cpp.leftEar, pb.leftear());
   deserialise(cpp.rightEar, pb.rightear());
   deserialise(cpp.leftEye, pb.lefteye());
   deserialise(cpp.rightEye, pb.righteye());
   deserialise(cpp.chestButton, pb.chestbutton());
   deserialise(cpp.leftFoot, pb.leftfoot());
   deserialise(cpp.rightFoot, pb.rightfoot());
}

static void deserialise(ActionCommand::All &cpp, const offnao::ActionCommandAll &pb) {
   deserialise(cpp.head, pb.head());
   deserialise(cpp.body, pb.body());
   deserialise(cpp.leds, pb.leds());
}

static void deserialise(FootPosition &cpp, const offnao::Motion_FootPosition &pb) {
   cpp.x = pb.x();
   cpp.y = pb.y();
   cpp.theta = pb.theta();
}

static void deserialise(FeetPosition &cpp, const offnao::Motion_FeetPosition &pb) {
   deserialise(cpp.left, pb.left());
   deserialise(cpp.right, pb.right());
}

static void deserialise(MotionDebugInfo &cpp, const offnao::Motion_MotionDebugInfo &pb) {
   deserialise(cpp.feetPosition, pb.feetposition());
}

static void deserialise(Parameters<float> &cpp, const offnao::Kinematics_FloatParameters &pb) {
   cpp.cameraPitchTop    = pb.camerapitchtop();
   cpp.cameraYawTop      = pb.camerayawtop();
   cpp.cameraRollTop     = pb.camerarolltop();
   cpp.cameraYawBottom   = pb.camerayawbottom();
   cpp.cameraPitchBottom = pb.camerapitchbottom();
   cpp.cameraRollBottom  = pb.camerarollbottom();
   cpp.bodyPitch         = pb.bodypitch();
}

static void deserialise(AbsCoord &cpp, const offnao::AbsCoord &pb) {
   deserialise(cpp.vec, pb.vec());
   deserialise(cpp.var, pb.var());
   cpp.weight = pb.weight();
}

static void deserialise(RRCoord &cpp, const offnao::RRCoord &pb) {
   deserialise(cpp.vec, pb.vec());
   deserialise(cpp.var, pb.var());
}

static void deserialise(Rangei &cpp, const offnao::Rangei &pb) {
   cpp.min = pb.min();
   cpp.max = pb.max();
}

static void deserialise(Boundaryi &cpp, const offnao::Boundaryi &pb) {
   deserialise(cpp.x, pb.x());
   deserialise(cpp.y, pb.y());
}

static void deserialise(SharedStateEstimationBundle &cpp, const offnao::SharedStateEstimationBundle &pb) {
   deserialise(cpp.robotPos, pb.robotpos());
   deserialise(cpp.ballPosRRC, pb.ballposrrc());
   deserialise(cpp.ballVelRRC, pb.ballvelrrc());
   cpp.haveBallUpdate = pb.haveballupdate();
   cpp.haveTeamBallUpdate = pb.haveteamballupdate();
}

static void deserialise(RobotObstacle &cpp, const offnao::StateEstimation_RobotObstacle &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.rrc, pb.rrc());
   deserialise(cpp.pos, pb.pos());
   deserialise(cpp.tangentHeadingLeft, pb.tangentheadingleft());
   deserialise(cpp.tangentHeadingRight, pb.tangentheadingright());
   deserialise(cpp.evadeVectorLeft, pb.evadevectorleft());
   deserialise(cpp.evadeVectorRight, pb.evadevectorright());
}

static void deserialise(BehaviourRequest &cpp, const offnao::Behaviour_BehaviourRequest &pb) {
   deserialise(cpp.actions, pb.actions());
   deserialise(cpp.behaviourSharedData, pb.behaviourshareddata());
   deserialise(cpp.behaviourDebugInfo, pb.behaviourdebuginfo());
}

static void deserialise(BehaviourDebugInfo &cpp, const offnao::BehaviourDebugInfo &pb) {
   deserialise(cpp.bodyBehaviourHierarchy, pb.bodybehaviourhierarchy());
   deserialise(cpp.headBehaviourHierarchy, pb.headbehaviourhierarchy());
   deserialise(cpp.haveBallManoeuvreTarget, pb.haveballmanoeuvretarget());
   deserialise(cpp.ballManoeuvreTargetX, pb.ballmanoeuvretargetx());
   deserialise(cpp.ballManoeuvreTargetY, pb.ballmanoeuvretargety());
   deserialise(cpp.ballManoeuvreHeadingError, pb.ballmanoeuvreheadingerror());
   deserialise(cpp.ballManoeuvreType, pb.ballmanoeuvretype());
   deserialise(cpp.ballManoeuvreHard, pb.ballmanoeuvrehard());
   deserialise(cpp.anticipating, pb.anticipating());
   deserialise(cpp.anticipateX, pb.anticipatex());
   deserialise(cpp.anticipateY, pb.anticipatey());
   deserialise(cpp.anticipateH, pb.anticipateh());
}

static void deserialise(Eigen::Matrix<int, 2, 1> &cpp, const offnao::Point &pb) {
   cpp[0] = pb.x();
   cpp[1] = pb.y();
}

static void deserialise(Eigen::Matrix<float, 2, 1> &cpp, const offnao::PointF &pb) {
   cpp[0] = pb.x();
   cpp[1] = pb.y();
}

static void deserialise(BBox &cpp, const offnao::BBox &pb) {
   deserialise(cpp.a, pb.a());
   deserialise(cpp.b, pb.b());
}

static void deserialise(BallInfo &cpp, const offnao::Vision_BallInfo &pb) {
   deserialise(cpp.status, pb.status());
   deserialise(cpp.rr, pb.rr());
   cpp.radius = pb.radius();
   deserialise(cpp.imageCoords, pb.imagecoords());
   cpp.topCamera = pb.topcamera();
}

static void deserialise(RobotVisionInfo &cpp, const offnao::Vision_RobotVisionInfo &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.cameras, pb.cameras());
   deserialise(cpp.imageCoords, pb.imagecoords());
   deserialise(cpp.topImageCoords, pb.topimagecoords());
   deserialise(cpp.botImageCoords, pb.botimagecoords());
}

static void deserialise(FieldBoundary &cpp, const offnao::Vision_FieldBoundary &pb) {
   deserialise(cpp.boundaryOnField, pb.boundaryonfield());
   deserialise(cpp.boundaryInImage, pb.boundaryinimage());
   deserialise(cpp.boundaryInImageLowerBound, pb.boundaryinimagelowerbound());
   deserialise(cpp.boundaryInImageUpperBound, pb.boundaryinimageupperbound());
   cpp.isValid = pb.isvalid();
   cpp.extrapolated = pb.extrapolated();
   cpp.odd = pb.odd();
}

static void deserialise(FieldFeatureInfo &cpp, const offnao::Vision_FieldFeatureInfo &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.p1, pb.p1());
   deserialise(cpp.p2, pb.p2());
   deserialise(cpp.field1, pb.field1());
   deserialise(cpp.field2, pb.field2());
   cpp.topCamera = pb.topcamera();
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void deserialise(RegionI &cpp, const offnao::Vision_RegionI &pb) {
   deserialise(cpp.is_top_camera_, pb.is_top_camera_());
   deserialise(cpp.bounding_box_rel_, pb.bounding_box_rel_());
   deserialise(cpp.bounding_box_fovea_, pb.bounding_box_fovea_());
   deserialise(cpp.bounding_box_raw_, pb.bounding_box_raw_());
   deserialise(cpp.n_raw_cols_in_region_, pb.n_raw_cols_in_region_());
   deserialise(cpp.n_raw_rows_in_region_, pb.n_raw_rows_in_region_());
   deserialise(cpp.density_to_raw_, pb.density_to_raw_());
   deserialise(cpp.y_offset_raw_, pb.y_offset_raw_());
   deserialise(cpp.x_offset_raw_, pb.x_offset_raw_());
   deserialise(cpp.raw_total_width_, pb.raw_total_width_());
   deserialise(cpp.raw_to_fovea_density_, pb.raw_to_fovea_density_());
   deserialise(cpp.fovea_width_, pb.fovea_width_());
}

static void deserialise(SPLStandardMessage &cpp, const offnao::Receiver_SPLStandardMessage &pb) {
   deserialise(cpp.header, pb.header());
   deserialise(cpp.version, pb.version());
   deserialise(cpp.playerNum, pb.playernum());
   deserialise(cpp.teamNum, pb.teamnum());
   deserialise(cpp.fallen, pb.fallen());
   deserialise(cpp.pose, pb.pose());
   deserialise(cpp.ballAge, pb.ballage());
   deserialise(cpp.ball, pb.ball());
   deserialise(cpp.numOfDataBytes, pb.numofdatabytes());

   const string &pbField = pb.data();
   size_t       pbSize   = pbField.size();
   size_t       cppSize  = sizeof(uint8_t[SPL_STANDARD_MESSAGE_DATA_SIZE]);
   if (cppSize < pbSize) {
      llog(ERROR) << "trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << std::endl;
   }
   if (cpp.numOfDataBytes != pbSize) {
      llog(ERROR) << "trying to deserialise " << pbSize << " bytes but numOfDataBytes is " << cpp.numOfDataBytes
                  << std::endl;
   }
   memcpy(cpp.data, pbField.data(), min(cppSize, pbSize));
}

static void deserialise(BehaviourSharedData &cpp, const offnao::BehaviourSharedData &pb) {
   deserialise(cpp.role, pb.role());
   deserialise(cpp.playingBall, pb.playingball());
   deserialise(cpp.playingBallScore, pb.playingballscore());
   deserialise(cpp.needAssistance, pb.needassistance());
   deserialise(cpp.isAssisting, pb.isassisting());
   deserialise(cpp.secondsSinceLastKick, pb.secondssincelastkick());
   deserialise(cpp.isKickedOff, pb.iskickedoff());
   deserialise(cpp.walkingToX, pb.walkingtox());
   deserialise(cpp.walkingToY, pb.walkingtoy());
   deserialise(cpp.walkingToH, pb.walkingtoh());
}

static void deserialise(BroadcastData &cpp, const offnao::Receiver_BroadcastData &pb) {
   deserialise(cpp.playerNum, pb.playernum());
   deserialise(cpp.robotPos, pb.robotpos());
   deserialise(cpp.ballPosAbs, pb.ballposabs());
   deserialise(cpp.ballPosRR, pb.ballposrr());
   deserialise(cpp.sharedStateEstimationBundle, pb.sharedstateestimationbundle());
   deserialise(cpp.behaviourSharedData, pb.behaviourshareddata());
   deserialise(cpp.acB, pb.acb());
   deserialise(cpp.uptime, pb.uptime());
   deserialise(cpp.gameState, pb.gamestate());
}

static void deserialise(CameraSettingsSerialse& csSettings, const offnao::UpdateCameraSettings &pb) {
   csSettings.whichCamera = (CameraInfo::Camera) pb.whichcamera();
   deserialise(csSettings.settings, pb.settings());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugger deserialisation, compile out everything is NDEBUG is set
#ifdef TMP_NDEBUG
   static void deserialise(DebuggerBlackboard& dbbSerialise, const offnao::Debugger &pb) {
      // do nothing
   }
#else 

   static void deserialise(ScanGrid::ScanGridLine& sgl, const offnao::Debugger_VisionDebugger_ScanGrid_ScanGridLine &pb) {
      sgl.x = pb.x();
      sgl.yMin = pb.ymin();
      sgl.yMax = pb.ymax();
      sgl.lowResYMaxIndex = pb.lowresymaxindex();
      sgl.yMaxIndex = pb.ymaxindex();
   }

   static void deserialise(ScanGrid::ScanGridHorizontalLine& sgl, const offnao::Debugger_VisionDebugger_ScanGrid_ScanGridHorizontalLine &pb) {
      sgl.y = pb.y();
      sgl.left = pb.left();
      sgl.right = pb.right();
   }

   static void deserialise(ScanGrid& sg, const offnao::Debugger_VisionDebugger_ScanGrid &pb) {
      ::deserialise(sg.fullResY, pb.fullresy(), pb.fullresy().size());
      ::deserialise(sg.lowResHorizontalLines, pb.lowreshorizontallines());
      ::deserialise(sg.verticalLines, pb.verticallines());
      sg.fieldLimit = pb.fieldlimit();
      sg.lowResStart = pb.lowresstart();
      sg.lowResStep = pb.lowresstep();
   }

   static void deserialise(ScanLineRange& slrange, const offnao::Debugger_VisionDebugger_ScanLineRange &pb) {
      slrange.from = pb.fromsl();
      slrange.to = pb.tosl();
      slrange.upper = pb.uppersl();
      slrange.lower = pb.lowersl();
      slrange.left = pb.leftsl();
      slrange.right = pb.rightsl();
   }

   static void deserialise(ScanLineRegion& slregion, const offnao::Debugger_VisionDebugger_ScanLineRegion &pb) {
      ::deserialise(slregion.range, pb.range());
      ::deserialise(slregion.color, pb.color());
   }

   static void deserialise(ColorScanLineRegionsHorizontal::ScanLine& hsl, const offnao::Debugger_VisionDebugger_ColorScanLineRegionsHorizontal_HorizontalScanLine &pb) {
      hsl.y = pb.y();
      ::deserialise(hsl.regions, pb.regions());
   }

   static void deserialise(ColorScanLineRegionsHorizontal& hsl, const offnao::Debugger_VisionDebugger_ColorScanLineRegionsHorizontal &pb) {
      ::deserialise(hsl.scanLines, pb.scanlines());
   }

   static void deserialise(ColorScanLineRegionsVertical::ScanLine& vsl, const offnao::Debugger_VisionDebugger_ColorScanLineRegionsVertical_VerticalScanLine &pb) {
      vsl.x = pb.x();
      ::deserialise(vsl.regions, pb.regions());
   }

   static void deserialise(ColorScanLineRegionsVertical& vsl, const offnao::Debugger_VisionDebugger_ColorScanLineRegionsVertical &pb) {
      ::deserialise(vsl.scanLines, pb.scanlines());
      vsl.lowResStart = pb.lowresstart();
      vsl.lowResStep = pb.lowresstep();
   }

   static void deserialise(PenaltyMarkRegions& pmr, const offnao::Debugger_VisionDebugger_PenaltyMarkRegions &pb) {
      ::deserialise(pmr.regions, pb.regions());
   }

   static void deserialise(BallSpots& sg, const offnao::Debugger_VisionDebugger_BallSpots &pb) {
      ::deserialise(sg.ballSpots, pb.ballspots());
      sg.firstSpotIsPredicted = pb.firstspotispredicted();
   }

   static void deserialise(IntersectionCandidates& ics, const offnao::Debugger_VisionDebugger_IntersectionCandidates &pb) {
      ::deserialise(ics.intersections, pb.intersections());
   }

   static void deserialise(IntersectionCandidates::IntersectionCandidate& ic, const offnao::Debugger_VisionDebugger_IntersectionCandidate &pb) {
      // ::deserialise(ic.pos, pb.pos());
      ::deserialise(ic.img, pb.img());
      // ::deserialise(ic.dir1, pb.dir1());
      // ::deserialise(ic.dir2, pb.dir2());
   }

   static void deserialise(Spot& spot, const offnao::Debugger_VisionDebugger_Spot& pb) {
      spot.image.x() = pb.imagex();
      spot.image.y() = pb.imagey();
      spot.field.x() = pb.fieldx();
      spot.field.y() = pb.fieldy();
      spot.candidate = pb.candidate();
   }

   static void deserialise(Candidate& candidate, const offnao::Debugger_VisionDebugger_Candidate& pb) {
      candidate.n0.x() = pb.n0x();
      candidate.n0.y() = pb.n0y();
      candidate.d = pb.d();

      candidate.spots.clear();  // Clear any existing spots
      for (const auto& pbSpot : pb.spots()) {
         Spot spot;
         ::deserialise(spot, pbSpot);
         candidate.spots.push_back(new Spot(spot));  // Allocate with copy constructor
      }
   }

   static void deserialise(CircleCandidate& circleCandidate, const offnao::Debugger_VisionDebugger_CircleCandidate& pb) {
      circleCandidate.center.x() = pb.centerx();
      circleCandidate.center.y() = pb.centery();
      circleCandidate.radius = pb.radius();

      circleCandidate.fieldSpots.clear();  // Clear any existing spots
      for (const auto& pbSpot : pb.fieldspots()) {
         circleCandidate.fieldSpots.emplace_back(pbSpot.x(), pbSpot.y());
      }

      circleCandidate.imageSpots.clear();  // Clear any existing spots
      for (const auto& pbSpot : pb.imagespots()) {
         circleCandidate.imageSpots.emplace_back(pbSpot.x(), pbSpot.y());
      }
   }

   static void deserialise(Candidates& candidates, const offnao::Debugger_VisionDebugger_Candidates& pb) {
      // Deserialize horizontalCandidates
      candidates.horizontalCandidates.clear();  // Clear existing candidates
      for (const auto& pbCandidate : pb.horizontalcandidates()) {
         Candidate candidate;
         ::deserialise(candidate, pbCandidate);
         candidates.horizontalCandidates.push_back(candidate);
      }

      // Deserialize verticalCandidates
      candidates.verticalCandidates.clear();  // Clear existing candidates
      for (const auto& pbCandidate : pb.verticalcandidates()) {
         Candidate candidate;
         ::deserialise(candidate, pbCandidate);
         candidates.verticalCandidates.push_back(candidate);
      }
   }

   static void deserialise(CircleCandidates& circleCandidates, const offnao::Debugger_VisionDebugger_CircleCandidates& pb) {
      circleCandidates.candidates.clear();  // Clear existing candidates
      for (const auto& pbCircleCandidate : pb.candidates()) {
         CircleCandidate circleCandidate;
         ::deserialise(circleCandidate, pbCircleCandidate);
         circleCandidates.candidates.push_back(circleCandidate);
      }
   }
   static void deserialise(LineSpots::Line& line, const offnao::Debugger_VisionDebugger_Line& pb) {
      // Deserialize Geometry::Line
      line.line.base.x() = pb.line().base().x();
      line.line.base.y() = pb.line().base().y();
      line.line.direction.x() = pb.line().direction().x();
      line.line.direction.y() = pb.line().direction().y();

      // Deserialize spotsInField
      line.spotsInField.clear();
      for (const auto& pbSpot : pb.spotsinfield()) {
         line.spotsInField.emplace_back(pbSpot.x(), pbSpot.y());
      }

      // Deserialize spotsInImg
      line.spotsInImg.clear();
      for (const auto& pbSpot : pb.spotsinimg()) {
         line.spotsInImg.emplace_back(pbSpot.x(), pbSpot.y());
      }

      // Deserialize first and last image spots
      line.firstImg.x() = pb.firstimg().x();
      line.firstImg.y() = pb.firstimg().y();
      line.lastImg.x() = pb.lastimg().x();
      line.lastImg.y() = pb.lastimg().y();

      // Deserialize first and last field positions
      line.firstField.x() = pb.firstfield().x();
      line.firstField.y() = pb.firstfield().y();
      line.lastField.x() = pb.lastfield().x();
      line.lastField.y() = pb.lastfield().y();

      // Deserialize other attributes
      line.belongsToCircle = pb.belongstocircle();
      line.length = pb.length();
   }
   static void deserialise(LineSpots& lineSpots, const offnao::Debugger_VisionDebugger_LineSpots& pb) {
      lineSpots.lines.clear();
      for (const auto& pbLine : pb.lines()) {
         LineSpots::Line line;
         deserialise(line, pbLine);
         lineSpots.lines.push_back(line);
      }
   }

   static void deserialise(CentreCircle &cpp, const offnao::Debugger_VisionDebugger_CentreCircle &pb) {
      deserialise(cpp.image, pb.image());
      cpp.wasSeen = pb.wasseen();
   }

   static void deserialise(PlaneSpots& planeSpots, const offnao::Debugger_VisionDebugger_PlaneSpots& pb) {
      planeSpots.spots.clear();  // Clear existing spots
      for (const auto& pbSpot : pb.spots()) {
         Spot spot;
         ::deserialise(spot, pbSpot);
         planeSpots.spots.push_back(new Spot(spot));  // Allocate with copy constructor
      }
   }

   static void deserialise(Comparison& comparison, const offnao::Debugger_VisionDebugger_Comparison& pb) {
      ::deserialise(comparison.pointInImage, pb.pointinimage());
      ::deserialise(comparison.referencePoint, pb.referencepoint());
      comparison.type = pb.type();
   }

   static void deserialise(IsWhiteSpots& isWhiteSpots, const offnao::Debugger_VisionDebugger_IsWhiteSpots& pb) {
      isWhiteSpots.comparisons.clear();  // Clear existing comparisons
      for (const auto& pbComparison : pb.comparisons()) {
         Comparison comparison;
         ::deserialise(comparison, pbComparison);
         isWhiteSpots.comparisons.push_back(comparison);
      }
   }

   static void deserialise(RobotObstaclesImage::Obstacle& obstacle, const offnao::Debugger_VisionDebugger_ObstacleImage& pb) {
      obstacle.top = pb.top();
      obstacle.bottom = pb.bottom();
      obstacle.left = pb.left();
      obstacle.right = pb.right();
      obstacle.bottomFound = pb.bottomfound();
      obstacle.fallen = pb.fallen();
      obstacle.confidence = pb.confidence();
      obstacle.distance = pb.distance();
   }

   static void deserialise(RobotObstaclesImage& robotsImage, const offnao::Debugger_VisionDebugger_RobotObstaclesImage& pb) {
      ::deserialise(robotsImage.obstacles, pb.obstacles());
   }

   static void deserialise(RefereeKeypoints::Point& refereeKeypoint, const offnao::Debugger_VisionDebugger_RefereeKeypoint& pb) {
      refereeKeypoint.position.x() = pb.position().x();
      refereeKeypoint.position.y() = pb.position().y();
      refereeKeypoint.valid = pb.valid();
   }

   static void deserialise(RefereeKeypoints& refereeKeypoints, const offnao::Debugger_VisionDebugger_RefereeKeypoints& pb) {
      ::deserialise(refereeKeypoints.points, pb.points());
   }

   static void deserialise(VisionDebuggerBlackboard& vdbb, const offnao::Debugger_VisionDebugger &pb) {
      ::deserialise(vdbb.scanGrid, pb.scangrid());
      ::deserialise(vdbb.ballSpots, pb.ballspots());
      ::deserialise(vdbb.colorScanLineRegionsHorizontal, pb.colorscanlineregionshorizontal());
      ::deserialise(vdbb.colorScanLineRegionsVerticalClipped, pb.colorscanlineregionsvertical());
      ::deserialise(vdbb.candidates, pb.candidates());
      ::deserialise(vdbb.candidatesBefore, pb.candidatesbefore());
      ::deserialise(vdbb.candidatesAfter, pb.candidatesafter());
      ::deserialise(vdbb.circleCandidates, pb.circlecandidates());
      ::deserialise(vdbb.intersectionCandidates, pb.intersectioncandidates());
      ::deserialise(vdbb.lineSpots, pb.linespots());
      ::deserialise(vdbb.centreCircle, pb.centrecircle());
      ::deserialise(vdbb.planeSpots, pb.planespots());
      ::deserialise(vdbb.isWhiteSpots, pb.iswhitespots());
      ::deserialise(vdbb.robotsImage, pb.robotsimage());
      ::deserialise(vdbb.refereeKeypoints, pb.refereekeypoints());
   }

   static void deserialise(DebuggerBlackboard& dbb, const offnao::Debugger &pb) {
      ::deserialise(*dbb.vision, pb.vision());
   }

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Blackboard deserlisation
static const uint8_t *deserialise(const offnao::Blackboard &pb,
                                  int height,
                                  int width,
                                  bool (::offnao::Vision::*hasframejpeg)() const,
                                  const string &(::offnao::Vision::*getframejpeg)() const) {
   if ((pb.vision().*hasframejpeg)()) {
      static uint8_t                *yuvFrame = new uint8_t[height * width * 3];
      // os 2.1 has turbo jpeg 1.1.1
      // os 2.8 has turbo jpeg 1.4.2, which has some more convenient functions, supposedly
      // https://github.com/libjpeg-turbo/libjpeg-turbo/blob/1.1.x/example.c
      struct jpeg_decompress_struct cinfo;
      struct jpeg_error_mgr         jerr; // TODO: not exit on error
      JSAMPARRAY                    buffer; /* Output row buffer */
      int                           row_stride; /* physical row width in output buffer */
      cinfo.err = jpeg_std_error(&jerr);
      jpeg_create_decompress(&cinfo);
      jpeg_mem_src(&cinfo,
                   (unsigned char *) (pb.vision().*getframejpeg)().data(),
                   (pb.vision().*getframejpeg)().size());
      (void) jpeg_read_header(&cinfo, TRUE);
      cinfo.output_components = 3;
      cinfo.out_color_space   = JCS_YCbCr;
      (void) jpeg_start_decompress(&cinfo);
      row_stride = cinfo.output_width * cinfo.output_components;
      buffer     = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
      while (cinfo.output_scanline < cinfo.output_height) {
         (void) jpeg_read_scanlines(&cinfo, buffer, 1);
         memcpy(&yuvFrame[(cinfo.output_scanline - 1) * row_stride], buffer[0], row_stride);
      }
      (void) jpeg_finish_decompress(&cinfo);

      // only works with yuvyuv not yuyv
      const int     yuyvFrameSize   = height * width * 2;
      uint8_t       *yuyvFrame      = new uint8_t[yuyvFrameSize];
      const uint8_t *yuvFrameRead   = yuvFrame;
      uint8_t       *yuyvFrameWrite = yuyvFrame;
      const uint8_t *const yuyvFrameWriteEnd = yuyvFrame + yuyvFrameSize;
      do {
         // convert two pixels at a time
         yuyvFrameWrite[0] = yuvFrameRead[0];
         yuyvFrameWrite[1] = yuvFrameRead[1]; // ignore u of the second pixel
         yuyvFrameWrite[2] = yuvFrameRead[3];
         yuyvFrameWrite[3] = yuvFrameRead[2]; // ignore v of the second pixel
         yuvFrameRead += 6;
         yuyvFrameWrite += 4;
      } while (yuyvFrameWrite < yuyvFrameWriteEnd);
      jpeg_destroy_decompress(&cinfo);

      return yuyvFrame;
   }

   return nullptr;
}

// the if/mask statements belong in serialise not deserialise, but we keep them here anyway so we don't need `if (!pb.vision().landmarks().empty())` or `if(pb.vision().has_timestamp())`
void deserialise(Blackboard &cpp, const offnao::Blackboard &pb) {
   cpp.mask = pb.mask();
   if (cpp.mask & BLACKBOARD_MASK) {
      cpp.gameController->player_number = pb.gamecontroller().player_number();

      // Deserialise Motion
      ::deserialise(cpp.motion->sensors, pb.motion().sensors());
      ::deserialise(cpp.motion->pose, pb.motion().pose());
      ::deserialise(cpp.motion->com, pb.motion().com());
      ::deserialise(cpp.motion->odometry, pb.motion().odometry());
      ::deserialise(cpp.motion->active, pb.motion().active());
      ::deserialise(cpp.motion->jointRequest, pb.motion().jointrequest());
      ::deserialise(cpp.motion->motionDebugInfo, pb.motion().motiondebuginfo());

      cpp.perception->behaviour       = pb.perception().behaviour();
      cpp.perception->kinematics      = pb.perception().kinematics();
      cpp.perception->stateEstimation = pb.perception().stateestimation();
      cpp.perception->total           = pb.perception().total();
      cpp.perception->vision          = pb.perception().vision();

      // Deserialise Behaviour Request
      deserialiseArray(cpp.behaviour->request, pb.behaviour().request());

      // Deserialise Kinematics
      ::deserialise(cpp.kinematics->parameters, pb.kinematics().parameters());

      if (cpp.mask & ROBOT_FILTER_MASK) {
         ::deserialise(cpp.stateEstimation->robotObstacles, pb.stateestimation().robotobstacles());
      }

      /* Only serialise the things below if WHITEBOARD_MASK is not set.
       * We also ONLY want this to happen when we are serialising in Offnao,
       * which occurs when we save the dump. WHITEBOARD_MASK can only be set
       * in the save function in offnao.
       */
      if (!(cpp.mask & WHITEBOARD_MASK)) {
         cpp.vision->timestamp = pb.vision().timestamp();
         ::deserialise(cpp.vision->balls, pb.vision().balls());
         ::deserialise(cpp.vision->robots, pb.vision().robots());
         ::deserialise(cpp.vision->fieldBoundary, pb.vision().fieldboundary());
         ::deserialise(cpp.vision->fieldFeatures, pb.vision().fieldfeatures());
         cpp.vision->refereeGesture.gesture = static_cast<RefereeGesture::Gesture>(pb.vision().refereegesture().gesture());

         ::deserialise(cpp.vision->regions, pb.vision().regions());
      }

      ::deserialise(cpp.vision->topResolution, pb.vision().topresolution());
      ::deserialise(cpp.vision->botResolution, pb.vision().botresolution());
      ::deserialise(cpp.vision->topCameraSettings, pb.vision().topcamerasettings());
      ::deserialise(cpp.vision->botCameraSettings, pb.vision().botcamerasettings());
      ::deserialise(cpp.vision->topAutoExposureWeightTable, pb.vision().topautoexposureweighttable());
      ::deserialise(cpp.vision->botAutoExposureWeightTable, pb.vision().botautoexposureweighttable());

      // Deserialise Receiver BB
      deserialiseArray(cpp.receiver->message, pb.receiver().message());
      deserialiseArray(cpp.receiver->data, pb.receiver().data());
      ::deserialiseWithImplicitCast(cpp.receiver->lastReceived, pb.receiver().lastreceived());
      // TODO: add the sizes to the message
      ::deserialise(cpp.receiver->incapacitated, pb.receiver().incapacitated(), pb.receiver().incapacitated_size());


      // Deserialise State Estimation
      ::deserialise(cpp.stateEstimation->robotPos, pb.stateestimation().robotpos());
      ::deserialise(cpp.stateEstimation->allRobotPos, pb.stateestimation().allrobotpos());
      ::deserialise(cpp.stateEstimation->ballPosRR, pb.stateestimation().ballposrr());
      ::deserialise(cpp.stateEstimation->ballPosRRC, pb.stateestimation().ballposrrc());
      ::deserialise(cpp.stateEstimation->ballVelRRC, pb.stateestimation().ballvelrrc());
      ::deserialise(cpp.stateEstimation->ballVel, pb.stateestimation().ballvel());
      ::deserialise(cpp.stateEstimation->ballPos, pb.stateestimation().ballpos());
      ::deserialise(cpp.stateEstimation->teamBallPos, pb.stateestimation().teamballpos());
      ::deserialise(cpp.stateEstimation->teamBallVel, pb.stateestimation().teamballvel());
      ::deserialise(cpp.stateEstimation->sharedStateEstimationBundle,
                    pb.stateestimation().sharedstateestimationbundle());
      cpp.stateEstimation->havePendingOutgoingSharedBundle = pb.stateestimation().havependingoutgoingsharedbundle();
      ::deserialise(cpp.stateEstimation->havePendingIncomingSharedBundle,
                    pb.stateestimation().havependingincomingsharedbundle(),
            // TODO: add the sizes to the message
                    pb.stateestimation().havependingincomingsharedbundle_size());
      ::deserialise(cpp.stateEstimation->walkToPoint, pb.stateestimation().walktopoint());
   }

   if (cpp.mask & CAMERA_IMAGE_MASK) {
      SerialiseImage::deserialise(cpp.vision->topImage, pb.vision().topimage());
      SerialiseImage::deserialise(cpp.vision->botImage, pb.vision().botimage());

      // TODO TW: For now this depends on having camera image transmitted
      //          to get the camera image dimensions.
      //          The TODO is to find another method to get the dimensions
      if (cpp.mask & JPEG_IMAGE_MASK) {
         cpp.vision->topFrame = ::deserialise(pb,
                                              cpp.vision->topImage.height,
                                              cpp.vision->topImage.width,
                                              &::offnao::Vision::has_topframejpeg,
                                              &::offnao::Vision::topframejpeg);
         cpp.vision->botFrame = ::deserialise(pb,
                                              cpp.vision->botImage.height,
                                              cpp.vision->botImage.width,
                                              &::offnao::Vision::has_botframejpeg,
                                              &::offnao::Vision::botframejpeg);
      }
   }

   // Deserialise Debugger Blackboard
   if (cpp.mask & BB_DEBUG_MASK) {
      if (pb.has_debugger()) {
         ::deserialise(*cpp.debugger, pb.debugger());
      } else {
         cpp.debugger = nullptr;
      }
   }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Base ProtobufSerialisable functions creating the actual streamable data

// Note: pbType can't be const here, as it's generated from the array
//       but it's const from the deserialise call onwards
template <class PS, typename T>
static void deserialise(PS& pbs, T& pbType, std::istream &is) {
   static_assert(std::is_base_of<ProtobufSerialisable, PS>::value, 
                 "PS must be a ProtobufSerialisable type");

   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size;
   is.read(reinterpret_cast<char *>(&size), sizeof(size));
   char *array = new char[size];
   is.read(array, size);
   pbType.ParseFromArray(array, size);

   deserialise(pbs, pbType);
}

void Blackboard::deserialise(std::istream &is) {
   offnao::Blackboard bb;
   ::deserialise<Blackboard, offnao::Blackboard>(*this, bb, is);
}

void CameraSettingsSerialse::deserialise(std::istream &is) {
   offnao::UpdateCameraSettings bb;
   ::deserialise<CameraSettingsSerialse, offnao::UpdateCameraSettings>(*this, bb, is);
}
