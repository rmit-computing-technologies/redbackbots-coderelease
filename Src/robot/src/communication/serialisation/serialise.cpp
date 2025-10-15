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

#include "communication/serialisation/serialise.hpp"

// Additional serialisable objects
#include "types/serialise/CameraSettingsSerialse.hpp"

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
// Seralise functions

// Forward Declarations
static void serialise(const RobotObstacle &, offnao::StateEstimation_RobotObstacle &);

static void serialise(const AbsCoord &, offnao::AbsCoord &);

static void serialise(const BehaviourRequest &, offnao::Behaviour_BehaviourRequest &);

static void serialise(const BallInfo &, offnao::Vision_BallInfo &);

static void serialise(const RobotVisionInfo &, offnao::Vision_RobotVisionInfo &);

static void serialise(const FieldBoundary &, offnao::Vision_FieldBoundary &);

static void serialise(const FieldFeatureInfo &, offnao::Vision_FieldFeatureInfo &);

static void serialise(const SPLStandardMessage &, offnao::Receiver_SPLStandardMessage &);

static void serialise(const BroadcastData &, offnao::Receiver_BroadcastData &);

static void serialise(const BehaviourDebugInfo &, offnao::BehaviourDebugInfo &);

static void serialise(const BehaviourSharedData &, offnao::BehaviourSharedData &);

#ifdef TMP_NDEBUG
#else 
static void serialise(const ScanGrid::ScanGridLine &, offnao::Debugger_VisionDebugger_ScanGrid_ScanGridLine &);
static void serialise(const ScanGrid &, offnao::Debugger_VisionDebugger_ScanGrid &);
#endif

template<typename T>
static void serialise(const T &cpp, offnao::RepeatedInt &pb);

template<typename T>
static void serialise(const T &cpp, offnao::RepeatedFloat &pb);

template<typename T, int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, google::protobuf::RepeatedField<T> &pb);

template<int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<uint8_t, size1_Rows, size2_Cols, Eigen::RowMajor> &cpp, 
                      offnao::UIntMatrix &pb);

static void serialise(const boost::numeric::ublas::matrix<float> &cpp, offnao::FloatMatrix &pb);

template<int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<float, size1_Rows, size2_Cols> &cpp, offnao::FloatMatrix &pb);

// Friend Serialisation Classes
template <typename T>
void SerialiseImage::serialise(const Image<T> &cpp, offnao::Vision_CameraImage &pb) {
   // llog(INFO) << "Image:: serialise " << cpp.width << " x " << cpp.height << " with size " << sizeof(T[cpp.height * cpp.width]) << std::endl;
   pb.set_height(cpp.height);
   pb.set_width(cpp.width);
   pb.set_data(cpp.image, sizeof(T[cpp.height * cpp.width]));
};

// Serialise standard data types
static void serialise(const string &cpp, string &pb) {
   pb = cpp;
}

template<unsigned int size>
static void serialise(const char(&array)[size], string &pb) {
   serialise(string(array, size), pb);
}

template<typename T, typename Container>
static void serialise(const Container &container, ::google::protobuf::RepeatedField<T> &repeatedField, int size) {
   repeatedField.Reserve(size);
   for (int i = 0; i < size; ++i) {
      repeatedField.Add(container[i]);
   }
}

template<typename T, int size>
static void serialise(const T(&array)[size], ::google::protobuf::RepeatedField<T> &repeatedField) {
   serialise(array, repeatedField, size);
}

// different name so it's not used accidentally
template<typename T, typename U, int size>
static void serialiseWithImplicitCast(const T(&array)[size], ::google::protobuf::RepeatedField<U> &repeatedField) {
   serialise(array, repeatedField, size);
}

template<typename T>
static void serialise(const vector<T> &vector, ::google::protobuf::RepeatedField<T> &repeatedField) {
   serialise(vector, repeatedField, vector.size());
}

template<typename T, std::size_t N>
static void serialise(const std::array<T, N> &array, 
                      ::google::protobuf::RepeatedField<T> &repeatedField) {
   serialise(array, repeatedField, array.size());
}

template<typename T, typename Container>
static void serialise(const Container &container, ::google::protobuf::RepeatedPtrField<T> &repeatedPtrField, int size) {
   repeatedPtrField.Reserve(size);
   for (int i = 0; i < size; ++i) {
      T *t = repeatedPtrField.Add();
      serialise(container[i], *t);
   }
}

// TODO (TW): After testing remove
// template<typename T, typename U>
// static void serialise(const T *array, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField, int size) {
//    repeatedPtrField.Reserve(size);
//    for (int i = 0; i < size; ++i) {
//       U *u = repeatedPtrField.Add();
//       serialise(array[i], *u);
//    }
// }

template<typename T, typename U, int size>
static void serialise(const T(&array)[size], ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(array, repeatedPtrField, size);
}

template<typename T, typename U>
static void serialise(const vector <T> &vector, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(vector.data(), repeatedPtrField, vector.size());
}

template<typename T, std::size_t N, typename U>
static void serialise(const std::array<T, N> &array, 
                      ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(array, repeatedPtrField, array.size());
}

static void serialise(const pair<int, int> &cpp, offnao::PairIntInt &pb) {
   pb.set_first(cpp.first);
   pb.set_second(cpp.second);
}

template<typename T>
static void serialise(const T &cpp, offnao::RepeatedInt &pb) {
   serialise(cpp, *pb.mutable_values());
}

template<typename T>
static void serialise(const T &cpp, offnao::RepeatedFloat &pb) {
   serialise(cpp, *pb.mutable_values());
}

template<typename T, int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, google::protobuf::RepeatedField<T> &pb) {
   serialise(cpp.data(), pb, size1_Rows * size2_Cols);
}

template<int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<uint8_t, size1_Rows, size2_Cols, Eigen::RowMajor> &cpp, 
                      offnao::UIntMatrix &pb) {
   pb.set_size1(size1_Rows);
   pb.set_size2(size2_Cols);
   serialise(cpp.data(), *pb.mutable_data(), size1_Rows * size2_Cols);
}

static void
serialise(const boost::numeric::ublas::matrix<float> &cpp, offnao::FloatMatrix &pb) {
   unsigned int size1 = cpp.size1();
   unsigned int size2 = cpp.size2();
   pb.set_size1(size1);
   pb.set_size2(size2);
   serialise(cpp.data(), *pb.mutable_data(), size1 * size2);
}

template<int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<float, size1_Rows, size2_Cols> &cpp, offnao::FloatMatrix &pb) {
   pb.set_size1(size1_Rows);
   pb.set_size2(size2_Cols);
   serialise(cpp.data(), *pb.mutable_data(), size1_Rows * size2_Cols);
}

static void serialise(const CameraSettings &cpp, offnao::CameraSettings &pb) {
   serialise(cpp.settings, *pb.mutable_settings());

   pb.set_hflip(cpp.hflip);
   pb.set_vflip(cpp.vflip);
}

static void serialise(const JointValues &cpp, offnao::JointValues &pb) {
   // offnao is compiled for v5 and so it expects the v5 order
   serialise(cpp.angles, *pb.mutable_angles());
   serialise(cpp.stiffnesses, *pb.mutable_stiffnesses());
   serialise(cpp.temperatures, *pb.mutable_temperatures());
   serialise(cpp.currents, *pb.mutable_currents());
}

static void serialise(const SensorValues &cpp, offnao::SensorValues &pb) {
   serialise(cpp.joints, *pb.mutable_joints());
   serialise(cpp.sensors, *pb.mutable_sensors());
}

// Making this static creates a warning in the friend declaration.
// There are some GCC pragmas to squelch the warning, but they don't work in g++...
void serialise(const RobotPose &cpp, offnao::Motion_Pose &pb) {
   serialise(cpp.topCameraToWorldTransform, *pb.mutable_topcameratoworldtransform());
   serialise(cpp.botCameraToWorldTransform, *pb.mutable_botcameratoworldtransform());
   serialise(cpp.neckToWorldTransform, *pb.mutable_necktoworldtransform());
   serialise(cpp.origin, *pb.mutable_origin());
   serialise(cpp.zunit, *pb.mutable_zunit());
   serialise(cpp.topCOrigin, *pb.mutable_topcorigin());
   serialise(cpp.botCOrigin, *pb.mutable_botcorigin());
   serialise(cpp.horizon, *pb.mutable_horizon());
   serialiseWithImplicitCast(cpp.topExclusionArray, *pb.mutable_topexclusionarray());
   serialiseWithImplicitCast(cpp.botExclusionArray, *pb.mutable_botexclusionarray());
}

void serialise(const Odometry &cpp, offnao::Motion_Odometry &pb) {
   pb.set_forward(cpp.forward);
   pb.set_left(cpp.left);
   pb.set_turn(cpp.turn);
}

static void serialise(const XYZ_Coord &cpp, offnao::XYZ_Coord &pb) {
   pb.set_x(cpp.x);
   pb.set_y(cpp.y);
   pb.set_z(cpp.z);
}

static void serialise(const ActionCommand::Head &cpp, offnao::ActionCommandAll_Head &pb) {
   pb.set_yaw(cpp.yaw);
   pb.set_pitch(cpp.pitch);
   pb.set_isrelative(cpp.isRelative);
   pb.set_yawspeed(cpp.yawSpeed);
   pb.set_pitchspeed(cpp.pitchSpeed);
}

static void serialise(const ActionCommand::Body &cpp, offnao::ActionCommandAll_Body &pb) {
   pb.set_actiontype(static_cast<offnao::ActionType>(cpp.actionType));
   pb.set_forward(cpp.forward);
   pb.set_left(cpp.left);
   pb.set_turn(cpp.turn);
   pb.set_power(cpp.power);
}

static void serialise(const ActionCommand::rgb &cpp, offnao::ActionCommandAll_LED_rgb &pb) {
   pb.set_red(cpp.red);
   pb.set_green(cpp.green);
   pb.set_blue(cpp.blue);
}

static void serialise(const ActionCommand::rgbSegments &cpp, offnao::ActionCommandAll_LED_rgbSegments &pb) {
   for (size_t i = 0; i < cpp.segments.size(); ++i) {
      const ActionCommand::rgb &segment = cpp.segments[i];
      offnao::ActionCommandAll_LED_rgb* pbSegment = pb.add_segments();

      pbSegment->set_red(segment.red);
      pbSegment->set_green(segment.green);
      pbSegment->set_blue(segment.blue);
   }
}

static void serialise(const ActionCommand::LED &cpp, offnao::ActionCommandAll_LED &pb) {
   pb.set_leftear(cpp.leftEar);
   pb.set_rightear(cpp.rightEar);
   serialise(cpp.leftEye, *pb.mutable_lefteye());
   serialise(cpp.rightEye, *pb.mutable_righteye());
   serialise(cpp.chestButton, *pb.mutable_chestbutton());
   serialise(cpp.leftFoot, *pb.mutable_leftfoot());
   serialise(cpp.rightFoot, *pb.mutable_rightfoot());
}

static void serialise(const ActionCommand::All &cpp, offnao::ActionCommandAll &pb) {
   serialise(cpp.head, *pb.mutable_head());
   serialise(cpp.body, *pb.mutable_body());
   serialise(cpp.leds, *pb.mutable_leds());
}

static void serialise(const FootPosition &cpp, offnao::Motion_FootPosition &pb) {
   pb.set_x(cpp.x);
   pb.set_y(cpp.y);
   pb.set_theta(cpp.theta);
}

static void serialise(const FeetPosition &cpp, offnao::Motion_FeetPosition &pb) {
   serialise(cpp.left, *pb.mutable_left());
   serialise(cpp.right, *pb.mutable_right());
}

static void serialise(const MotionDebugInfo &cpp, offnao::Motion_MotionDebugInfo &pb) {
   serialise(cpp.feetPosition, *pb.mutable_feetposition());
}

static void serialise(const Parameters<float> &cpp, offnao::Kinematics_FloatParameters &pb) {
   pb.set_camerapitchtop(cpp.cameraPitchTop);
   pb.set_camerayawtop(cpp.cameraYawTop);
   pb.set_camerarolltop(cpp.cameraRollTop);
   pb.set_camerayawbottom(cpp.cameraYawBottom);
   pb.set_camerapitchbottom(cpp.cameraPitchBottom);
   pb.set_camerarollbottom(cpp.cameraRollBottom);
   pb.set_bodypitch(cpp.bodyPitch);
}

static void serialise(const AbsCoord &cpp, offnao::AbsCoord &pb) {
   serialise(cpp.vec, *pb.mutable_vec());
   serialise(cpp.var, *pb.mutable_var());
   pb.set_weight(cpp.weight);
}

static void serialise(const RRCoord &cpp, offnao::RRCoord &pb) {
   serialise(cpp.vec, *pb.mutable_vec());
   serialise(cpp.var, *pb.mutable_var());
}

static void serialise(const Rangei &cpp, offnao::Rangei &pb) {
   pb.set_min(cpp.min);
   pb.set_max(cpp.max);
}

static void serialise(const Boundaryi &cpp, offnao::Boundaryi &pb) {
   serialise(cpp.x, *pb.mutable_x());
   serialise(cpp.y, *pb.mutable_y());
}

static void serialise(const SharedStateEstimationBundle &cpp, offnao::SharedStateEstimationBundle &pb) {
   serialise(cpp.robotPos, *pb.mutable_robotpos());
   serialise(cpp.ballPosRRC, *pb.mutable_ballposrrc());
   serialise(cpp.ballVelRRC, *pb.mutable_ballvelrrc());
   pb.set_haveballupdate(cpp.haveBallUpdate);
   pb.set_haveteamballupdate(cpp.haveTeamBallUpdate);
}

static void serialise(const RobotObstacle &cpp, offnao::StateEstimation_RobotObstacle &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::RobotVisionInfoType>(cpp.type));
   serialise(cpp.rrc, *pb.mutable_rrc());
   serialise(cpp.pos, *pb.mutable_pos());
   pb.set_tangentheadingleft(cpp.tangentHeadingLeft);
   pb.set_tangentheadingright(cpp.tangentHeadingRight);
   serialise(cpp.evadeVectorLeft, *pb.mutable_evadevectorleft());
   serialise(cpp.evadeVectorRight, *pb.mutable_evadevectorright());
}

static void serialise(const BehaviourRequest &cpp, offnao::Behaviour_BehaviourRequest &pb) {
   serialise(cpp.actions, *pb.mutable_actions());
   serialise(cpp.behaviourSharedData, *pb.mutable_behaviourshareddata());
   serialise(cpp.behaviourDebugInfo, *pb.mutable_behaviourdebuginfo());
}

static void serialise(const BehaviourDebugInfo &cpp, offnao::BehaviourDebugInfo &pb) {
   pb.set_bodybehaviourhierarchy(cpp.bodyBehaviourHierarchy);
   pb.set_headbehaviourhierarchy(cpp.headBehaviourHierarchy);
   pb.set_haveballmanoeuvretarget(cpp.haveBallManoeuvreTarget);
   pb.set_ballmanoeuvretargetx(cpp.ballManoeuvreTargetX);
   pb.set_ballmanoeuvretargety(cpp.ballManoeuvreTargetY);
   pb.set_ballmanoeuvreheadingerror(cpp.ballManoeuvreHeadingError);
   pb.set_ballmanoeuvretype(cpp.ballManoeuvreType);
   pb.set_ballmanoeuvrehard(cpp.ballManoeuvreHard);
   pb.set_anticipating(cpp.anticipating);
   pb.set_anticipatex(cpp.anticipateX);
   pb.set_anticipatey(cpp.anticipateY);
   pb.set_anticipateh(cpp.anticipateH);
}

static void serialise(const Eigen::Matrix<int, 2, 1> &cpp, offnao::Point &pb) {
   pb.set_x(cpp[0]);
   pb.set_y(cpp[1]);
}

static void serialise(const Eigen::Matrix<float, 2, 1> &cpp, offnao::PointF &pb) {
   pb.set_x(cpp[0]);
   pb.set_y(cpp[1]);
}

static void serialise(const BBox &cpp, offnao::BBox &pb) {
   serialise(cpp.a, *pb.mutable_a());
   serialise(cpp.b, *pb.mutable_b());
}

static void serialise(const BallInfo &cpp, offnao::Vision_BallInfo &pb) {
   pb.set_status(static_cast<offnao::Vision_BallStatus>(cpp.status));
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_radius(cpp.radius);
   serialise(cpp.imageCoords, *pb.mutable_imagecoords());
   pb.set_topcamera(cpp.topCamera);
}

static void serialise(const RobotVisionInfo &cpp, offnao::Vision_RobotVisionInfo &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::RobotVisionInfoType>(cpp.type));
   pb.set_cameras(static_cast<offnao::Vision_Cameras>(cpp.cameras));
   serialise(cpp.imageCoords, *pb.mutable_imagecoords());
   serialise(cpp.topImageCoords, *pb.mutable_topimagecoords());
   serialise(cpp.botImageCoords, *pb.mutable_botimagecoords());
}

static void serialise(const FieldBoundary &cpp, offnao::Vision_FieldBoundary &pb) {
   serialise(cpp.boundaryOnField, *pb.mutable_boundaryonfield());
   serialise(cpp.boundaryInImage, *pb.mutable_boundaryinimage());
   serialise(cpp.boundaryInImageLowerBound, *pb.mutable_boundaryinimagelowerbound());
   serialise(cpp.boundaryInImageUpperBound, *pb.mutable_boundaryinimageupperbound());
   pb.set_isvalid(cpp.isValid);
   pb.set_extrapolated(cpp.extrapolated);
   pb.set_odd(cpp.odd);
}

static void serialise(const FieldFeatureInfo &cpp, offnao::Vision_FieldFeatureInfo &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::Vision_FieldFeatureInfoType>(cpp.type));
   serialise(cpp.p1, *pb.mutable_p1());
   serialise(cpp.p2, *pb.mutable_p2());
   serialise(cpp.field1, *pb.mutable_field1());
   serialise(cpp.field2, *pb.mutable_field2());
   pb.set_topcamera(cpp.topCamera);
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void serialise(const RegionI &cpp, offnao::Vision_RegionI &pb) {
   pb.set_is_top_camera_(cpp.is_top_camera_);
   serialise(cpp.bounding_box_rel_, *pb.mutable_bounding_box_rel_());
   serialise(cpp.bounding_box_fovea_, *pb.mutable_bounding_box_fovea_());
   serialise(cpp.bounding_box_raw_, *pb.mutable_bounding_box_raw_());
   pb.set_n_raw_cols_in_region_(cpp.n_raw_cols_in_region_);
   pb.set_n_raw_rows_in_region_(cpp.n_raw_rows_in_region_);
   pb.set_density_to_raw_(cpp.density_to_raw_);
   pb.set_y_offset_raw_(cpp.y_offset_raw_);
   pb.set_x_offset_raw_(cpp.x_offset_raw_);
   pb.set_raw_total_width_(cpp.raw_total_width_);
   pb.set_raw_to_fovea_density_(cpp.raw_to_fovea_density_);
   pb.set_fovea_width_(cpp.fovea_width_);
}

static void serialise(const SPLStandardMessage &cpp, offnao::Receiver_SPLStandardMessage &pb) {
   serialise(cpp.header, *pb.mutable_header());
   pb.set_version(cpp.version);
   pb.set_playernum(cpp.playerNum);
   pb.set_teamnum(cpp.teamNum);
   pb.set_fallen(cpp.fallen);
   serialise(cpp.pose, *pb.mutable_pose());
   pb.set_ballage(cpp.ballAge);
   serialise(cpp.ball, *pb.mutable_ball());
   pb.set_numofdatabytes(cpp.numOfDataBytes);
   pb.set_data(cpp.data, cpp.numOfDataBytes);
}

static void serialise(const BehaviourSharedData &cpp, offnao::BehaviourSharedData &pb) {
   pb.set_role(cpp.role);
   pb.set_playingball(cpp.playingBall);
   pb.set_playingballscore(cpp.playingBallScore);
   pb.set_needassistance(cpp.needAssistance);
   pb.set_isassisting(cpp.isAssisting);
   pb.set_secondssincelastkick(cpp.secondsSinceLastKick);
   pb.set_iskickedoff(cpp.isKickedOff);
   pb.set_walkingtox(cpp.walkingToX);
   pb.set_walkingtoy(cpp.walkingToY);
   pb.set_walkingtoh(cpp.walkingToH);
}

static void serialise(const BroadcastData &cpp, offnao::Receiver_BroadcastData &pb) {
   pb.set_playernum(cpp.playerNum);
   serialise(cpp.robotPos, *pb.mutable_robotpos());
   serialise(cpp.ballPosAbs, *pb.mutable_ballposabs());
   serialise(cpp.ballPosRR, *pb.mutable_ballposrr());
   serialise(cpp.sharedStateEstimationBundle, *pb.mutable_sharedstateestimationbundle());
   serialise(cpp.behaviourSharedData, *pb.mutable_behaviourshareddata());
   pb.set_acb(static_cast<offnao::ActionType>(cpp.acB));
   pb.set_uptime(cpp.uptime);
   pb.set_gamestate(cpp.gameState);
}

static void serialise(const CameraSettingsSerialse& csSerialise, offnao::UpdateCameraSettings &pb) {
   pb.set_whichcamera(csSerialise.whichCamera);
   serialise(csSerialise.settings, *pb.mutable_settings());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugger serialise, compile out everything is NDEBUG is set
#ifdef TMP_NDEBUG
   static void serialise(const DebuggerBlackboard& dbbSerialise, offnao::Debugger &pb) {
      // do nothing
   }
#else 

   static void serialise(const ScanGrid::ScanGridLine& sgl, offnao::Debugger_VisionDebugger_ScanGrid_ScanGridLine &pb) {
      pb.set_x(sgl.x);
      pb.set_ymin(sgl.yMin);
      pb.set_ymax(sgl.yMax);
      pb.set_lowresymaxindex(sgl.lowResYMaxIndex);
      pb.set_ymaxindex(sgl.yMaxIndex);
   }

   static void serialise(const ScanGrid::ScanGridHorizontalLine& sgl, offnao::Debugger_VisionDebugger_ScanGrid_ScanGridHorizontalLine &pb) {
      pb.set_y(sgl.y);
      pb.set_left(sgl.left);
      pb.set_right(sgl.right);
   }

   static void serialise(const ScanGrid& sg, offnao::Debugger_VisionDebugger_ScanGrid &pb) {
      ::serialise(sg.fullResY, *pb.mutable_fullresy());
      ::serialise(sg.lowResHorizontalLines, *pb.mutable_lowreshorizontallines());
      ::serialise(sg.verticalLines, *pb.mutable_verticallines());
      pb.set_fieldlimit(sg.fieldLimit);
      pb.set_lowresstart(sg.lowResStart);
      pb.set_lowresstep(sg.lowResStep);
   }

   static void serialise(const ScanLineRange& slrange, offnao::Debugger_VisionDebugger_ScanLineRange &pb) {
      pb.set_fromsl(slrange.from);
      pb.set_tosl(slrange.to);
      pb.set_uppersl(slrange.upper);
      pb.set_lowersl(slrange.lower);
      pb.set_leftsl(slrange.left);
      pb.set_rightsl(slrange.right);
   }

   static void serialise(const ScanLineRegion& slregion, offnao::Debugger_VisionDebugger_ScanLineRegion &pb) {
      ::serialise(slregion.range, *pb.mutable_range());
      pb.set_color(static_cast<offnao::Debugger_VisionDebugger_ScanLineRegion_ScanLineColor>(slregion.color));
   }

   static void serialise(const ColorScanLineRegionsHorizontal::ScanLine& sl, offnao::Debugger_VisionDebugger_ColorScanLineRegionsHorizontal_HorizontalScanLine &pb) {
      pb.set_y(sl.y);
      ::serialise(sl.regions, *pb.mutable_regions());
   }

   static void serialise(const ColorScanLineRegionsHorizontal& sl, offnao::Debugger_VisionDebugger_ColorScanLineRegionsHorizontal &pb) {
      ::serialise(sl.scanLines, *pb.mutable_scanlines());
   }

   static void serialise(const ColorScanLineRegionsVertical::ScanLine& sl, offnao::Debugger_VisionDebugger_ColorScanLineRegionsVertical_VerticalScanLine &pb) {
      pb.set_x(sl.x);
      ::serialise(sl.regions, *pb.mutable_regions());
   }

   static void serialise(const ColorScanLineRegionsVertical& sl, offnao::Debugger_VisionDebugger_ColorScanLineRegionsVertical &pb) {
      ::serialise(sl.scanLines, *pb.mutable_scanlines());
      pb.set_lowresstart(sl.lowResStart);
      pb.set_lowresstep(sl.lowResStep);
   }

      static void serialise(const PenaltyMarkRegions& pmr, offnao::Debugger_VisionDebugger_PenaltyMarkRegions &pb) {
      ::serialise(pmr.regions, *pb.mutable_regions());
   }

   static void serialise(const BallSpots& sg, offnao::Debugger_VisionDebugger_BallSpots &pb) {
      ::serialise(sg.ballSpots, *pb.mutable_ballspots());
      pb.set_firstspotispredicted(sg.firstSpotIsPredicted);
   }

   static void serialise(const IntersectionCandidates& ic, offnao::Debugger_VisionDebugger_IntersectionCandidates &pb) {
      ::serialise(ic.intersections, *pb.mutable_intersections());
   }

   static void serialise(const IntersectionCandidates::IntersectionCandidate& ics, offnao::Debugger_VisionDebugger_IntersectionCandidate &pb) {
      // ::serialise(ics.pos, *pb.mutable_pos());
      ::serialise(ics.img, *pb.mutable_img());
      // ::serialise(ics.dir1, *pb.mutable_dir1());
      // ::serialise(ics.dir1, *pb.mutable_dir2());
   }

   static void serialise(const Spot& spot, offnao::Debugger_VisionDebugger_Spot &pb) {
      pb.set_imagex(spot.image.x());
      pb.set_imagey(spot.image.y());
      pb.set_fieldx(spot.field.x());
      pb.set_fieldy(spot.field.y());
      pb.set_candidate(spot.candidate);
   }

   static void serialise(const Candidate& candidate, offnao::Debugger_VisionDebugger_Candidate &pb) {
      pb.set_n0x(candidate.n0.x());
      pb.set_n0y(candidate.n0.y());
      pb.set_d(candidate.d);

      for (const auto* spot : candidate.spots) {
         if (spot) {
            auto* pbSpot = pb.add_spots();  // Add a new Spot to the Protobuf message
            ::serialise(*spot, *pbSpot);   // Serialize the spot into the Protobuf spot
         }
      }
   }

   static void serialise(const CircleCandidate& circleCandidate, offnao::Debugger_VisionDebugger_CircleCandidate &pb) {
      pb.set_centerx(circleCandidate.center.x());
      pb.set_centery(circleCandidate.center.y());
      pb.set_radius(circleCandidate.radius);

      for (const auto& spot : circleCandidate.fieldSpots) {
         auto* pbSpot = pb.add_fieldspots();  // Add a new Spot to the Protobuf message
         pbSpot->set_x(spot.x());
         pbSpot->set_y(spot.y());
      }
      for (const auto& spot : circleCandidate.imageSpots) {
         auto* pbSpot = pb.add_imagespots();  // Add a new Spot to the Protobuf message
         pbSpot->set_x(spot.x());
         pbSpot->set_y(spot.y());
      }
   }

   static void serialise(const Candidates& candidates, offnao::Debugger_VisionDebugger_Candidates &pb) {
      // Serialise horizontalCandidates
      for (const auto& candidate : candidates.horizontalCandidates) {
         auto* pbCandidate = pb.add_horizontalcandidates();
         ::serialise(candidate, *pbCandidate);
      }

      // Serialise verticalCandidates
      for (const auto& candidate : candidates.verticalCandidates) {
         auto* pbCandidate = pb.add_verticalcandidates();
         ::serialise(candidate, *pbCandidate);
      }
   }

   static void serialise(const CircleCandidates& circleCandidates, offnao::Debugger_VisionDebugger_CircleCandidates &pb) {
      for (const auto& circleCandidate : circleCandidates.candidates) {
         auto* pbCircleCandidate = pb.add_candidates();
         ::serialise(circleCandidate, *pbCircleCandidate);
      }
   }

   static void serialise(const LineSpots::Line& line, offnao::Debugger_VisionDebugger_Line &pb) {
      // Serialize Geometry::Line
      pb.mutable_line()->mutable_base()->set_x(line.line.base.x());
      pb.mutable_line()->mutable_base()->set_y(line.line.base.y());
      pb.mutable_line()->mutable_direction()->set_x(line.line.direction.x());
      pb.mutable_line()->mutable_direction()->set_y(line.line.direction.y());

      // Serialize spotsInField
      for (const auto& spot : line.spotsInField) {
         auto* pbSpot = pb.add_spotsinfield();
         pbSpot->set_x(spot.x());
         pbSpot->set_y(spot.y());
      }

      // Serialize spotsInImg
      for (const auto& spot : line.spotsInImg) {
         auto* pbSpot = pb.add_spotsinimg();
         pbSpot->set_x(spot.x());
         pbSpot->set_y(spot.y());
      }

      // Serialize first and last image spots
      pb.mutable_firstimg()->set_x(line.firstImg.x());
      pb.mutable_firstimg()->set_y(line.firstImg.y());
      pb.mutable_lastimg()->set_x(line.lastImg.x());
      pb.mutable_lastimg()->set_y(line.lastImg.y());

      // Serialize first and last field positions
      pb.mutable_firstfield()->set_x(line.firstField.x());
      pb.mutable_firstfield()->set_y(line.firstField.y());
      pb.mutable_lastfield()->set_x(line.lastField.x());
      pb.mutable_lastfield()->set_y(line.lastField.y());

      // Serialize other attributes
      pb.set_belongstocircle(line.belongsToCircle);
      pb.set_length(line.length);
   }

   static void serialise(const LineSpots& lineSpots, offnao::Debugger_VisionDebugger_LineSpots &pb) {
      for (const auto& line : lineSpots.lines) {
         auto* pbLine = pb.add_lines();
         serialise(line, *pbLine);
      }
   }

   static void serialise(const CentreCircle& cc, offnao::Debugger_VisionDebugger_CentreCircle &pb) {
      ::serialise(cc.image, *pb.mutable_image());
      pb.set_wasseen(cc.wasSeen);
   }


   static void serialise(const PlaneSpots& planeSpots, offnao::Debugger_VisionDebugger_PlaneSpots &pb) {
      for (const auto& spot : planeSpots.spots) {
         auto* pbSpot = pb.add_spots();
         ::serialise(*spot, *pbSpot);
      }
   }

   static void serialise(const Comparison& comparison, offnao::Debugger_VisionDebugger_Comparison &pb){
      ::serialise(comparison.pointInImage, *pb.mutable_pointinimage());
      ::serialise(comparison.referencePoint, *pb.mutable_referencepoint());
      pb.set_type(comparison.type);
   }

   static void serialise(const IsWhiteSpots& isWhiteSpots, offnao::Debugger_VisionDebugger_IsWhiteSpots &pb){
      for (const Comparison& comparison : isWhiteSpots.comparisons) {
         auto* pbComparison = pb.add_comparisons();
         ::serialise(comparison, *pbComparison);
      }
   }

   static void serialise(const RobotObstaclesImage::Obstacle& obstacle, offnao::Debugger_VisionDebugger_ObstacleImage& pb) {
      pb.set_top(obstacle.top);
      pb.set_bottom(obstacle.bottom);
      pb.set_left(obstacle.left);
      pb.set_right(obstacle.right);
      pb.set_bottomfound(obstacle.bottomFound);
      pb.set_fallen(obstacle.fallen);
      pb.set_confidence(obstacle.confidence);
      pb.set_distance(obstacle.distance);
   }

   static void serialise(const RobotObstaclesImage& robotsImage, offnao::Debugger_VisionDebugger_RobotObstaclesImage& pb){
      ::serialise(robotsImage.obstacles, *pb.mutable_obstacles());
   }

   static void serialise(const RefereeKeypoints::Point& refereeKeypoint, offnao::Debugger_VisionDebugger_RefereeKeypoint& pb) {
      pb.mutable_position()->set_x(refereeKeypoint.position.x());
      pb.mutable_position()->set_y(refereeKeypoint.position.y());
      pb.set_valid(refereeKeypoint.valid);
   }

   static void serialise(const RefereeKeypoints& refereeKeypoints, offnao::Debugger_VisionDebugger_RefereeKeypoints &pb){
      ::serialise(refereeKeypoints.points, *pb.mutable_points());
   }

   static void serialise(const VisionDebuggerBlackboard& vdbb, offnao::Debugger_VisionDebugger &pb) {
      ::serialise(vdbb.scanGrid, *pb.mutable_scangrid());
      ::serialise(vdbb.ballSpots, *pb.mutable_ballspots());
      ::serialise(vdbb.colorScanLineRegionsHorizontal, *pb.mutable_colorscanlineregionshorizontal());
      ::serialise(vdbb.colorScanLineRegionsVerticalClipped, *pb.mutable_colorscanlineregionsvertical());
      ::serialise(vdbb.candidates, *pb.mutable_candidates());
      ::serialise(vdbb.candidatesBefore, *pb.mutable_candidatesbefore());
      ::serialise(vdbb.candidatesAfter, *pb.mutable_candidatesafter());
      ::serialise(vdbb.circleCandidates, *pb.mutable_circlecandidates());
      ::serialise(vdbb.intersectionCandidates, *pb.mutable_intersectioncandidates());
      ::serialise(vdbb.lineSpots, *pb.mutable_linespots());
      ::serialise(vdbb.centreCircle, *pb.mutable_centrecircle());
      ::serialise(vdbb.planeSpots, *pb.mutable_planespots());
      ::serialise(vdbb.isWhiteSpots, *pb.mutable_iswhitespots());
      ::serialise(vdbb.robotsImage, *pb.mutable_robotsimage());
      ::serialise(vdbb.refereeKeypoints, *pb.mutable_refereekeypoints());
   }

   static void serialise(const DebuggerBlackboard& dbb, offnao::Debugger &pb) {
      ::serialise(*dbb.vision, *pb.mutable_vision());
   }

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Blackboard Serliasation
static void serialise(offnao::Blackboard &pb,
                      int bytesPerPixel,
                      const uint8_t *frame,
                      int8_t quality,
                      int height,
                      int width,
                      void (::offnao::Vision::*setframejpeg)(const void *, size_t)) {
   if (frame) {
      if (quality >= 0) {
         // TODO: TW, Update to work with new camera image
         // previous comment: only works with yuvyuv not yuyv
         if (bytesPerPixel == 2) {
            const int      newBytesPerPixel = 3;
            const int      yuvFrameSize     = height * width * newBytesPerPixel;
            static uint8_t *yuvFrame        = new uint8_t[yuvFrameSize];

            const uint8_t *yuyvFrameRead = frame;
            uint8_t       *yuvFrameWrite = yuvFrame;
            const uint8_t *const yuvFrameWriteEnd = yuvFrame + yuvFrameSize;
            do {
               // convert two pixels at a time
               yuvFrameWrite[0] = yuyvFrameRead[0];
               yuvFrameWrite[1] = yuyvFrameRead[1];
               yuvFrameWrite[2] = yuyvFrameRead[3];
               yuvFrameWrite[3] = yuyvFrameRead[2];
               yuvFrameWrite[4] = yuyvFrameRead[1];
               yuvFrameWrite[5] = yuyvFrameRead[3];
               yuyvFrameRead += 4;
               yuvFrameWrite += 6;
            } while (yuvFrameWrite < yuvFrameWriteEnd);

            frame         = yuvFrame;
            bytesPerPixel = newBytesPerPixel;
         }

         // os 2.1 has turbo jpeg 1.1.1
         // os 2.8 has turbo jpeg 1.4.2, which has some more convenient functions, supposedly
         // https://github.com/libjpeg-turbo/libjpeg-turbo/blob/1.1.x/example.c
         struct jpeg_compress_struct cinfo;
         struct jpeg_error_mgr       jerr; // TODO: not exit on error
         unsigned long               outsize    = height * width * bytesPerPixel;
         static unsigned char        *outbuffer = new unsigned char[outsize];
         JSAMPROW                    row_pointer[1]; /* pointer to JSAMPLE row[s] */
         int                         row_stride;     /* physical row width in image buffer */
         cinfo.err = jpeg_std_error(&jerr);
         jpeg_create_compress(&cinfo);
         jpeg_mem_dest(&cinfo, &outbuffer, &outsize);
         cinfo.image_width      = width;             /* image width and height, in pixels */
         cinfo.image_height     = height;
         cinfo.input_components = bytesPerPixel;     /* # of color components per pixel */
         cinfo.in_color_space   = JCS_YCbCr;         /* colorspace of input image */
         jpeg_set_defaults(&cinfo);
         jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
         jpeg_start_compress(&cinfo, TRUE);
         row_stride = width * bytesPerPixel;         /* JSAMPLEs per row in image_buffer */
         while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = const_cast<JSAMPROW>(&frame[cinfo.next_scanline * row_stride]);
            (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
         }
         jpeg_finish_compress(&cinfo);
         (pb.mutable_vision()->*setframejpeg)(outbuffer, outsize);
         jpeg_destroy_compress(&cinfo);
      }
   }
}

// the if/mask statements belong in serialise not serialise, but we keep them here anyway so we don't need `if (!pb.vision().landmarks().empty())` or `if(pb.vision().has_timestamp())`
void serialise(const Blackboard &cpp, offnao::Blackboard &pb) {
   pb.set_mask(cpp.mask);
   if (cpp.mask & BLACKBOARD_MASK) {
      pb.mutable_gamecontroller()->set_player_number(cpp.gameController->player_number);

      // Motion serialise
      ::serialise(cpp.motion->sensors, *pb.mutable_motion()->mutable_sensors());
      ::serialise(cpp.motion->pose, *pb.mutable_motion()->mutable_pose());
      ::serialise(cpp.motion->com, *pb.mutable_motion()->mutable_com());
      ::serialise(cpp.motion->odometry, *pb.mutable_motion()->mutable_odometry());
      ::serialise(cpp.motion->active, *pb.mutable_motion()->mutable_active());
      ::serialise(cpp.motion->jointRequest, *pb.mutable_motion()->mutable_jointrequest());
      ::serialise(cpp.motion->motionDebugInfo, *pb.mutable_motion()->mutable_motiondebuginfo());

      pb.mutable_perception()->set_behaviour(cpp.perception->behaviour);
      pb.mutable_perception()->set_kinematics(cpp.perception->kinematics);
      pb.mutable_perception()->set_stateestimation(cpp.perception->stateEstimation);
      pb.mutable_perception()->set_total(cpp.perception->total);
      pb.mutable_perception()->set_vision(cpp.perception->vision);

      // Serialise Behaviour Request
      ::serialise(cpp.behaviour->request, *pb.mutable_behaviour()->mutable_request());

      // Serialise Kinematics
      ::serialise(cpp.kinematics->parameters, *pb.mutable_kinematics()->mutable_parameters());

      if (cpp.mask & ROBOT_FILTER_MASK) {
         ::serialise(cpp.stateEstimation->robotObstacles, *pb.mutable_stateestimation()->mutable_robotobstacles());
      }

      /* Only serialise the things below if WHITEBOARD_MASK is not set.
       * We also ONLY want this to happen when we are serialising in Offnao,
       * which occurs when we save the dump. WHITEBOARD_MASK can only be set
       * in the save function in offnao.
       */
      if (!(cpp.mask & WHITEBOARD_MASK)) {
         pb.mutable_vision()->set_timestamp(cpp.vision->timestamp);
         ::serialise(cpp.vision->balls, *pb.mutable_vision()->mutable_balls());
         ::serialise(cpp.vision->robots, *pb.mutable_vision()->mutable_robots());
         ::serialise(cpp.vision->fieldBoundary, *pb.mutable_vision()->mutable_fieldboundary());
         ::serialise(cpp.vision->fieldFeatures, *pb.mutable_vision()->mutable_fieldfeatures());
         pb.mutable_vision()->mutable_refereegesture()->set_gesture(static_cast<offnao::Vision_RefereeGesture_Gesture>(cpp.vision->refereeGesture.gesture));

         ::serialise(cpp.vision->regions, *pb.mutable_vision()->mutable_regions());
      }

      pb.mutable_vision()->set_topresolution(static_cast<offnao::Vision_CameraResolution>(cpp.vision->topResolution));
      pb.mutable_vision()->set_botresolution(static_cast<offnao::Vision_CameraResolution>(cpp.vision->botResolution));
      ::serialise(cpp.vision->topCameraSettings, *pb.mutable_vision()->mutable_topcamerasettings());
      ::serialise(cpp.vision->botCameraSettings, *pb.mutable_vision()->mutable_botcamerasettings());
      ::serialise(cpp.vision->topAutoExposureWeightTable, *pb.mutable_vision()->mutable_topautoexposureweighttable());
      ::serialise(cpp.vision->botAutoExposureWeightTable, *pb.mutable_vision()->mutable_botautoexposureweighttable());

      // Reveriver BB serialise
      ::serialise(cpp.receiver->message, *pb.mutable_receiver()->mutable_message());
      ::serialise(cpp.receiver->data, *pb.mutable_receiver()->mutable_data());
      serialiseWithImplicitCast(cpp.receiver->lastReceived, *pb.mutable_receiver()->mutable_lastreceived());
      ::serialise(cpp.receiver->incapacitated, *pb.mutable_receiver()->mutable_incapacitated());

      // State estimation serialise
      ::serialise(cpp.stateEstimation->robotPos, *pb.mutable_stateestimation()->mutable_robotpos());
      ::serialise(cpp.stateEstimation->allRobotPos, *pb.mutable_stateestimation()->mutable_allrobotpos());
      ::serialise(cpp.stateEstimation->ballPosRR, *pb.mutable_stateestimation()->mutable_ballposrr());
      ::serialise(cpp.stateEstimation->ballPosRRC, *pb.mutable_stateestimation()->mutable_ballposrrc());
      ::serialise(cpp.stateEstimation->ballVelRRC, *pb.mutable_stateestimation()->mutable_ballvelrrc());
      ::serialise(cpp.stateEstimation->ballVel, *pb.mutable_stateestimation()->mutable_ballvel());
      ::serialise(cpp.stateEstimation->ballPos, *pb.mutable_stateestimation()->mutable_ballpos());
      ::serialise(cpp.stateEstimation->teamBallPos, *pb.mutable_stateestimation()->mutable_teamballpos());
      ::serialise(cpp.stateEstimation->teamBallVel, *pb.mutable_stateestimation()->mutable_teamballvel());
      ::serialise(cpp.stateEstimation->sharedStateEstimationBundle,
                  *pb.mutable_stateestimation()->mutable_sharedstateestimationbundle());
      pb.mutable_stateestimation()
            ->set_havependingoutgoingsharedbundle(cpp.stateEstimation->havePendingOutgoingSharedBundle);
      ::serialise(cpp.stateEstimation->havePendingIncomingSharedBundle,
                  *pb.mutable_stateestimation()->mutable_havependingincomingsharedbundle());
      ::serialise(cpp.stateEstimation->walkToPoint, *pb.mutable_stateestimation()->mutable_walktopoint());
   }

   if (cpp.mask & CAMERA_IMAGE_MASK) {
      SerialiseImage::serialise(cpp.vision->topImage, *pb.mutable_vision()->mutable_topimage());
      SerialiseImage::serialise(cpp.vision->botImage, *pb.mutable_vision()->mutable_botimage());

      // TODO TW: For deceralisation depends on having camera image transmitted
      //          to get the camera image dimensions.
      //          The TODO is to find another method to get the dimensions
      if (cpp.mask & JPEG_IMAGE_MASK) {
         ::serialise(pb,
                     2,
                     cpp.vision->topFrame,
                     cpp.vision->topFrameJPEGQuality,
                     cpp.vision->topImage.height,
                     cpp.vision->topImage.width,
                     &::offnao::Vision::set_topframejpeg);
         ::serialise(pb,
                     2,
                     cpp.vision->botFrame,
                     cpp.vision->botFrameJPEGQuality,
                     cpp.vision->botImage.height,
                     cpp.vision->botImage.width,
                     &::offnao::Vision::set_botframejpeg);
      }
   }

   // Serialise Debugger Blackboard
   if (cpp.mask & BB_DEBUG_MASK) {
      if (cpp.debugger != nullptr) {
         ::serialise(*cpp.debugger, *pb.mutable_debugger());
      }
   }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Base ProtobufSerialisable functions creating the actual streamable data

template <class PS, typename T>
static void serialise(const PS& pbs, T& pbType, std::ostream &os) {
   static_assert(std::is_base_of<ProtobufSerialisable, PS>::value, 
                 "PS must be a ProtobufSerialisable type");
   
   // Do serialisation
   serialise(pbs, pbType);

   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size = pbType.ByteSize();
   os.write(reinterpret_cast<char *>(&size), sizeof(size));
   pbType.SerializeToOstream(&os);
}

void Blackboard::serialise(std::ostream &os) const {
   offnao::Blackboard bb;
   ::serialise<Blackboard, offnao::Blackboard>(*this, bb, os);
}

void CameraSettingsSerialse::serialise(std::ostream &os) const {
   offnao::UpdateCameraSettings ucs;
   ::serialise<CameraSettingsSerialse, offnao::UpdateCameraSettings>(*this, ucs, os);
}
