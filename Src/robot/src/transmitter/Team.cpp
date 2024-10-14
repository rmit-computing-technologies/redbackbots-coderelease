#include "transmitter/Team.hpp"

#include <iostream>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/BehaviourBlackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "utils/Logger.hpp"
#include "types/SPLStandardMessage.hpp"
#include "types/NewTeamMessage.hpp"
#include "utils/incapacitated.hpp"

using namespace boost::asio;
using namespace std;

TeamTransmitter::TeamTransmitter(Blackboard *bb) :
   Adapter(bb),
   NaoTransmitter((bb->config)["network.transmitter_base_port"].as<int>()
                  + (bb->config)["player.team"].as<int>(),
                  (bb->config)["network.transmitter_address"].as<string>()),
   service(),
   socket(service, ip::udp::v4())
{}

void TeamTransmitter::tick() {
   haveTeamBallUpdate = readFrom(stateEstimation, haveTeamBallUpdate);
   haveBallScoreUpdate = readFrom(stateEstimation, haveBallScoreUpdate);

   // Only send to team if we have at least 50 messages left
   if (readFrom(gameController,our_team).messageBudget>50) {   
      if (haveTeamBallUpdate || haveBallScoreUpdate) {
         sendToTeam();
         if (haveBallScoreUpdate) {
            llog(INFO) << "============================ Sending ball score update ============================" << std::endl;
         }
         else {
            llog(INFO) << "============================ Sending Team ball update ============================" << std::endl;
         }
         writeTo(stateEstimation, haveTeamBallUpdate, false);
         writeTo(stateEstimation, haveBallScoreUpdate, false);
      }
   } else {
      // If we can't send a message, need to set it back to false otherwise it will stay set True for the rest of the match
      writeTo(stateEstimation, haveTeamBallUpdate, false);
      writeTo(stateEstimation, haveBallScoreUpdate, false);
   }
              
   sendToGameController();    // Send to GC every tick
 }

TeamTransmitter::~TeamTransmitter() {
   socket.close();
}

// void TeamTransmitter::sendToTeam(){
//    int playerNum = (blackboard->config)["player.number"].as<int>();
//    const AbsCoord &robotPos = readFrom(stateEstimation, robotPos);


//    BroadcastData bd = BroadcastData(
//                                  playerNum,
//                                  robotPos,
//                                  readFrom(stateEstimation, ballPos),
//                                  readFrom(stateEstimation, ballPosRR),
//                                  readFrom(stateEstimation, sharedStateEstimationBundle),
//                                  readFrom(behaviour, behaviourSharedData),
//                                  readFrom(motion, active).body.actionType,
//                                  readFrom(motion, uptime),
//                                  readFrom(gameController, gameState));

//    // calculate incapacitated
//    bool incapacitated = false;

//    if (readFrom(gameController, our_team).players[playerNum - 1].penalty
//        != PENALTY_NONE) {
//       incapacitated = true;
//    }

//    const ActionCommand::Body::ActionType &acB =
//             readFrom(motion, active).body.actionType;
//    incapacitated |= isIncapacitated(acB);


//    SPLStandardMessage m = SPLStandardMessage(
//                                           playerNum,
//                                           readFrom(gameController, our_team).teamNumber, 
//                                           incapacitated,
//                                           robotPos, 
//                                           0, // ballAge
//                                           readFrom(stateEstimation, ballPos), 
//                                           bd
//    );


//    writeTo(stateEstimation, havePendingOutgoingSharedBundle, false);

//    NaoTransmitter::tick(boost::asio::buffer(&m, sizeof(SPLStandardMessage)));
// }




void TeamTransmitter::sendToTeam(){
   int playerNum = (blackboard->config)["player.number"].as<int>();
   const AbsCoord &robotPos = readFrom(stateEstimation, robotPos);

   BroadcastData bd = BroadcastData(
                                 playerNum,
                                 robotPos,
                                 readFrom(stateEstimation, ballPos),
                                 readFrom(stateEstimation, ballPosRR),
                                 readFrom(stateEstimation, sharedStateEstimationBundle),
                                 readFrom(behaviour, behaviourSharedData),
                                 readFrom(motion, active).body.actionType,
                                 readFrom(motion, uptime),
                                 readFrom(gameController, gameState));




   // calculate incapacitated
   bool incapacitated = false;

   if (readFrom(gameController, our_team).players[playerNum - 1].penalty
       != PENALTY_NONE) {
      incapacitated = true;
   }

   const ActionCommand::Body::ActionType &acB =
            readFrom(motion, active).body.actionType;
   incapacitated |= isIncapacitated(acB);


   SPLStandardMessage m = SPLStandardMessage(
                                          playerNum,
                                          readFrom(gameController, our_team).teamNumber, 
                                          incapacitated,
                                          robotPos, 
                                          readFrom(stateEstimation, ballAge), // ballAge
                                          readFrom(stateEstimation, ballPos), 
                                          bd
   );

   NewTeamMessage ntm = NewTeamMessage(
                     playerNum,
                     readFrom(gameController, our_team).teamNumber, 
                     incapacitated,
                     robotPos,
                     readFrom(stateEstimation, ballAge), // ballAge
                     readFrom(stateEstimation, ballPos),
                     bd
   );


   writeTo(stateEstimation, havePendingOutgoingSharedBundle, false);

   NaoTransmitter::tick(boost::asio::buffer(&ntm, sizeof(NewTeamMessage)));
}
void TeamTransmitter::sendToGameController() {
   char* sendToIP = readFrom(gameController, lastGameControllerIPAddress);
   if (sendToIP == NULL) {
      // sendToIP is only initialised once GameController sends us a packet
      return;
   }

   gameControllerEndpoint.address(ip::address::from_string(sendToIP));
   gameControllerEndpoint.port(GAMECONTROLLER_RETURN_PORT);
   boost::system::error_code ec = boost::system::error_code();
   socket.connect(gameControllerEndpoint, ec);

   RoboCupGameControlReturnData d = RoboCupGameControlReturnData();
   d.teamNum = (blackboard->config)["player.team"].as<int>();
   d.playerNum = (blackboard->config)["player.number"].as<int>();
   d.fallen =  0;
   AbsCoord robot_pos = readFrom(stateEstimation, robotPos);
   d.pose[0] = robot_pos.x();
   d.pose[1] = robot_pos.y();
   d.pose[2] = robot_pos.theta();
   RRCoord ball_rel_pos = readFrom(stateEstimation, ballPosRR);
   d.ball[0] = ball_rel_pos.distance();
   d.ball[1] = ball_rel_pos.heading() * 1000;
   d.ballAge = readFrom(stateEstimation, ballAge);



   // TODO (Peter): If GameController PC goes away, we should get some kind of
   // catchable error here so we can stop sending packets until another
   // GC comes online
   socket.send(boost::asio::buffer(&d, sizeof(RoboCupGameControlReturnData)), 0, ec);
}
