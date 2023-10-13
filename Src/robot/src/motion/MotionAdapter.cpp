#include "motion/MotionAdapter.hpp"

#include <bitset>
#include <boost/asio/error.hpp>
#include <boost/bind.hpp>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/BehaviourBlackboard.hpp"
#include "blackboard/modules/KinematicsBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "blackboard/modules/ThreadBlackboard.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "motion/touch/LoLATouch.hpp"
#include "motion/effector/LoLAEffector.hpp"
#include "motion/touch/NullTouch.hpp"
#include "motion/effector/NullEffector.hpp"
#include "motion/generator/ClippedGenerator.hpp"
#include "motion/generator/DistributedGenerator.hpp"
#include "thread/Thread.hpp"
#include "types/AbsCoord.hpp"
#include "types/ActionCommand.hpp"
#include "types/JointValues.hpp"
#include "types/SensorValues.hpp"
#include "utils/Logger.hpp"
#include "utils/body.hpp"
#include "utils/incapacitated.hpp"
#include "utils/speech.hpp"
#include "utils/Timer.hpp"

using namespace std;

void construct(Touch** touch, std::string name, int team, int player_number) {
   if (name == "Agent" || name == "LoLA") {
      *touch = (Touch*) new LoLATouch(team, player_number);
   } else if (name == "Null") {
      *touch = (Touch*) new NullTouch();
   }
   if (*touch == NULL) {
      llog(FATAL) << "MotionAdapter: NULL " + name + "Touch" << endl;
   }
}

void construct(Effector** effector, std::string name, int team, int player_number) {
   if (name == "Agent" || name == "LoLA") {
      *effector = (Effector*) new LoLAEffector(team, player_number);
   } else if (name == "Null") {
      *effector = (Effector*) new NullEffector();
   }
   if (*effector == NULL) {
      llog(FATAL) << "MotionAdapter: NULL " + name + "Effector" << endl;
   }
}

/*-----------------------------------------------------------------------------
 * Motion thread constructor
 *---------------------------------------------------------------------------*/
MotionAdapter::MotionAdapter(Blackboard *bb)
   : Adapter(bb), uptime(0) {
   llog(INFO) << "Constructing MotionAdapter" << endl;

   // We only construct the NullTouch/Generators, the rest are done on demand
   touches["Null"] = (Touch*)(new NullTouch());
   if (touches["Null"] == NULL) {
      llog(FATAL) << "MotionAdapter: NULL NullTouch" << endl;
   }

   // don't need to support both names, but we do it anyway for legacy support
   touches["Agent"] = touches["LoLA"] = (Touch*)(NULL);
   touch = touches["Null"];

   DistributedGenerator * g = new DistributedGenerator();
   generator = (Generator*) new ClippedGenerator( (Generator *) g);
   if (generator == NULL) {
      llog(FATAL) << "MotionAdapter: NULL Generator" << endl;
   }

   effectors["Null"] = (Effector*)(new NullEffector());
   if (effectors["Null"] == NULL) {
      llog(FATAL) << "MotionAdapter: NULL NullEffector" << endl;
   }
   // don't need to support both names, but we do it anyway for legacy support
   effectors["Agent"] = effectors["LoLA"] = (Effector*)(NULL);
   effector = effectors["Null"];

   readOptions(bb->config);
   touch->readOptions(bb->config); // XXX: Do NOT put this in readOptions or robot will fall

   llog(INFO) << "MotionAdapater constructed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * Motion thread destructor
 *---------------------------------------------------------------------------*/
MotionAdapter::~MotionAdapter() {
   llog(INFO) << "Destroying MotionAdapter" << endl;

   writeTo(thread, configCallbacks[Thread::name], boost::function<void(const boost::program_options::variables_map &)>());

   for (std::map<std::string, Touch*>::iterator it = touches.begin();
        it != touches.end(); it++) {
      delete it->second;
   }

   delete generator;

   for (std::map<std::string, Effector*>::iterator it = effectors.begin();
        it != effectors.end(); it++) {
      delete it->second;
   }
}

/*-----------------------------------------------------------------------------
 * read motion options
 *---------------------------------------------------------------------------*/
void MotionAdapter::readOptions(const boost::program_options::variables_map& config) {
   std::string e = config["motion.effector"].as<string>();
   std::string t = config["motion.touch"].as<string>();
   llog(INFO) << "MotionAdapter using effector " << e << " and touch " << t << std::endl;

   // Look through the list of touches for the one requested,
   // initialize it if it exists.
   if (touches.count(t)) {
      if (touches[t] == NULL) {
         construct(&touches[t],
                   t,
                   config["player.team"].as<int>(),
                   config["player.number"].as<int>());
      }
      touch = touches[t];
   }

   // Look through the list of effectors for the one requested,
   // initialize it if it exists.
   if (effectors.count(e)) {
      if (effectors[e] == NULL) {
         construct(&effectors[e],
                   e,
                   config["player.team"].as<int>(),
                   config["player.number"].as<int>());
      }
      effector = effectors[e];
   }

   // Read the generator options
   generator->readOptions(config);

   isSonarLeftWorking  = config["touch.isSonarLeftWorking"].as<bool>();
   isSonarRightWorking = config["touch.isSonarRightWorking"].as<bool>();
}

/**
 * @return bitset
 */
static uint16_t getEar(float charge) {
   bitset<16> leds;
   for (int i = 0; i < 10; ++i) {
      leds[i] = charge >= (i + 1) / 10.f;
   }
   return static_cast<uint16_t>(leds.to_ulong());
}

static void setInfinityIfNotWorking(SensorValues &sensors,
                                    bool isSonarLeftWorking,
                                    bool isSonarRightWorking) {
  static float infinity = std::numeric_limits<float>::infinity();
  if (!isSonarLeftWorking)
      sensors.sensors[Sensors::SonarLeft] = infinity;
  if (!isSonarRightWorking)
      sensors.sensors[Sensors::SonarRight] = infinity;
}

/*-----------------------------------------------------------------------------
 * Motion thread tick function
 *---------------------------------------------------------------------------*/
void MotionAdapter::tick() {
   llog(TRACE) << "Motion tick\n";

   Timer t;
   // Get the motion request from behaviours
   int behaviourReadBuf = readFrom(behaviour, readBuf);
   ActionCommand::All request = readFrom(behaviour, request[behaviourReadBuf]).actions;

   try {
      if (touch == nullptr) {
         llog(FATAL) << "Touch is NULL\n";
      }
      touch->loadCache();
   } catch (const boost::system::system_error &se) {
      SAY(string("Could not load cache: ") + se.what());
      // "Invalid argument" when not connected to lola.  just ignore it as we'll try again on the next cycle
      if (se.code().category() == boost::asio::error::system_category && se.code().value() == EINVAL) {
         return;
      }
      // "End of file" when disconnected from lola.  just ignore it as we'll try again on the next cycle
      if (se.code().category() == boost::asio::error::misc_category && se.code().value() == boost::asio::error::eof) {
         return;
      }
   }

   // Get sensor information from kinematics
   SensorValues sensors = touch->getSensors(kinematics);

   // We must invert the gyroscopeZ for the V6
   sensors.sensors[Sensors::InertialSensor_GyroscopeZ] = -sensors.sensors[Sensors::InertialSensor_GyroscopeZ];

   kinematics.setSensorValues(sensors);
   kinematics.parameters = readFrom(kinematics, parameters);

   // Calculate the Denavit-Hartenberg chain
   kinematics.updateDHChain();
   writeTo(motion, pose, kinematics.getPose());

   // LoLATouch::getButtons sets standing, so run this first
   ButtonPresses buttons = touch->getButtons();
   bool standing = touch->getStanding();
   bool sitUnstiff = touch->getSitUnstiff();
   llog(TRACE) << "touch->getSensors took "
               << t.elapsed_ms() << "ms" << std::endl;
   t.restart();

   // Keep a running time for standing
   if (standing) {
      uptime = 0.0f;
   } else {
      uptime += 0.01f;
   }
   writeTo(motion, uptime, uptime);

   setInfinityIfNotWorking(sensors, isSonarLeftWorking, isSonarRightWorking);
   writeTo(motion, sensors, sensors);

   // set the leftEar based on the charge
   request.leds.leftEar = getEar(sensors.sensors[Sensors::Battery_Charge]);

   if (isIncapacitated(request.body.actionType)) {
      uptime = 0.0f;
   }
   buttons |= readFrom(motion, buttons);
   writeTo(motion, buttons, buttons);

   llog(TRACE) << "writeTo / readFrom took "
                 << t.elapsed_ms() << "ms" << std::endl;
   t.restart();

   if (sitUnstiff){
      request.body = ActionCommand::Body::SIT_UNSTIFF;
   }
   else if (standing) {
      generator->reset();
      request.body = ActionCommand::Body::INITIAL;
      odometry.clear();
   }

   // Get the position of the ball in robot relative cartesian coordinates

   AbsCoord robotPose = readFrom(stateEstimation, robotPos);
   AbsCoord ballAbs = readFrom(stateEstimation, ballPos);
   AbsCoord ball = ballAbs.convertToRobotRelativeCartesian(robotPose);

   // Update the body model
   bodyModel.kinematics = &kinematics;
   bodyModel.update(&odometry, sensors);

   // Create new MotionDebugInfo to be filled out
   MotionDebugInfo motionDebugInfo;

   // Get the joints requested by whichever generator we're using
   JointValues joints = generator->makeJoints(&request, &odometry, sensors, bodyModel, ball.x(), ball.y(), motionDebugInfo);

   // Save the body model Center of Mass
   writeTo(motion, com, bodyModel.getCoM());

   writeTo(motion, active, request);

   writeTo(motion, motionDebugInfo, motionDebugInfo);

   // Odometry is lagged by walk's estimations, and it also correctly synchronises with vision
   writeTo(motion, odometry, Odometry(odometry));

   llog(TRACE) << "generator->makeJoints took "
                 << t.elapsed_ms() << "ms" << std::endl;

   t.restart();

   writeTo(motion, jointRequest, joints);

   // Actuate joints as requested.
   effector->actuate(joints, request.leds, request.stiffen);
   llog(TRACE) << "effector->actuate took "
                 << t.elapsed_ms() << "ms" << std::endl;
}

