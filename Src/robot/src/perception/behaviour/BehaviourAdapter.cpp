#include <Python.h>

#include "perception/behaviour/BehaviourAdapter.hpp"

#include <fstream>
#include <limits>
#include <utility>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/BehaviourBlackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/KinematicsBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "perception/behaviour/BehaviourHelpers.hpp"
#include "perception/behaviour/python/PythonSkill.hpp"
#include "types/BehaviourRequest.hpp"
#include "utils/basic_maths.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/OptionConstants.hpp"
#include "types/SensorValues.hpp"
#include "utils/speech.hpp"

#include <boost/python.hpp>

using namespace std;
using namespace boost::python;

BehaviourAdapter::BehaviourAdapter(Blackboard *bb) : Adapter(bb), calibrationSkill(bb),
         safetySkill(bb->behaviour->useGetups)
{
   llog(INFO) << "Constructing BehaviourAdapter" << endl;
   //Read options from black board
   readOptions(bb->config);

   std::string hostname;
   ifstream hostfile ("/etc/hostname");
   getline (hostfile, hostname);
   pythonSkill = new PythonSkill(bb);

   // Alert redbackbots team - whistle detection requires 4 channels, not 2
   std::string noHearWhistles = "";
   int ret = system("python $HOME/whistle/alert_pulseaudio.py");
   if (ret != 0) {
      noHearWhistles += "I can not hear whistles. ";
      std::cout << noHearWhistles << std::endl;
   }

   // Max string length is 70 characters as defined by MAX_SAY_LENGTH in robot/libagent/AgentData.hpp
   std::stringstream startupSpeech;
   startupSpeech << noHearWhistles << std::string("Player ") << BehaviourHelpers::playerNumber(blackboard);
   if (ret != 0) {
     startupSpeech << " ... " << hostname;
   } else {
      if (BehaviourHelpers::teamNumber(blackboard) == 52) {
          startupSpeech << " team red back bots";
      } else {
          startupSpeech << " team " << BehaviourHelpers::teamNumber(blackboard);
      }

      startupSpeech << " ... I am ... " << hostname;
   }
   std::cout << startupSpeech.str() << std::endl;
   SAY(startupSpeech.str());
}

BehaviourAdapter::~BehaviourAdapter() {
}

void BehaviourAdapter::readOptions(const boost::program_options::variables_map& config) {
   runningIMUCalibrationSkill = config["calibration.imu"].as<bool>();
   if (runningIMUCalibrationSkill) {
      llog(INFO) << "BehaviourAdapter using imu calibration skill";
   }
   runningKickCalibrationSkill = config[CALIBRATION_KICK].as<bool>();
   if (runningKickCalibrationSkill) {
      llog(INFO) << "BehaviourAdapter using kick calibration skill";
      std::string footString = config["kick.foot"].as<std::string>();
      if(footString == "RIGHT")
         kickCalibrationFoot = ActionCommand::Body::RIGHT;
      else {
         if (footString != "LEFT")
            llog(INFO) << "Could not parse foot argument. Setting to left" << std::endl;
         kickCalibrationFoot = ActionCommand::Body::LEFT;
      }

   }
   // for Log.py
   setenv("LOG_PATH", config["debug.log.dir"].as<string>().c_str(), 1);
   // so we don't have to worry about old .pyc files
   setenv("PYTHONDONTWRITEBYTECODE", "1", 1);
   safetySkill.readOptions(config);
}

void BehaviourAdapter::tick() {
   BehaviourRequest behaviourRequest;
   if (readFrom(kinematics, isCalibrating)) {
      // kinematics calibrator
      behaviourRequest = calibrationSkill.execute();
   } else if (runningIMUCalibrationSkill) {
      behaviourRequest = imuCalibrationSkill.execute(readFrom(motion, sensors));
   } else if (runningKickCalibrationSkill) {
      behaviourRequest = kickCalibrationSkill.execute(readFrom(stateEstimation, ballPosRRC), readFrom(vision, balls),
                                                      kickCalibrationFoot);
   } else {
     // Run the python skill
      behaviourRequest = pythonSkill->execute();
   }

   writeTo(stateEstimation, ballAge, behaviourRequest.actions.ballAge);

   int playerNumber = readFrom(gameController, player_number);
   TeamInfo teamInfo = readFrom(gameController, our_team);
   bool isPenalised = teamInfo.players[playerNumber - 1].penalty != PENALTY_NONE;
   RoboCupGameControlData gameControllerData = readFrom(gameController, data);
   bool isMotionAllowed = gameControllerData.state == STATE_PLAYING || gameControllerData.state == STATE_READY;
   bool ukemiEnabled = isMotionAllowed && !isPenalised;

   // Write ActionCommands to blackboard
   int writeBuf = (readFrom(behaviour, readBuf) + 1) % 2;
   writeTo(behaviour, request[writeBuf], safetySkill.wrapRequest(behaviourRequest, readFrom(motion, sensors), ukemiEnabled));
   writeTo(behaviour, readBuf, writeBuf);

   // Write behaviourSharedData to blackboard, to broadcast to the team
   writeTo(behaviour, behaviourSharedData, behaviourRequest.behaviourSharedData);
}
