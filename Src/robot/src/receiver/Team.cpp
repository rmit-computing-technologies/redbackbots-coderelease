#include "receiver/Team.hpp"

#include <ctime>
#include <iostream>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/ReceiverBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "thread/Thread.hpp"
#include "types/SPLStandardMessage.hpp"
#include "types/NewTeamMessage.hpp"
#include "utils/incapacitated.hpp"


using namespace std;

TeamReceiver::TeamReceiver(Blackboard *bb, void(TeamReceiver::*handler)
                           (const boost::system::error_code & error, std::size_t))
   : Adapter(bb), NaoReceiver(this,
                              handler,
                              (bb->config)["network.transmitter_base_port"].as<int>()
                              + (bb->config)["player.team"].as<int>()) {}

void TeamReceiver::naoHandler(const boost::system::error_code &error,
                              std::size_t size) {
   if (Thread::name == NULL) {
      Thread::name = "TeamReceiverBoostThread";
   }

   // SPLStandardMessage* m = reinterpret_cast<SPLStandardMessage*>(recvBuffer);
   NewTeamMessage* ntm = reinterpret_cast<NewTeamMessage*>(recvBuffer);
   // BroadcastData* bd = (BroadcastData*)m->data;
   if (size == sizeof(NewTeamMessage)) {
      // std::cout << "NewTeamMessage Recieved!" << std::endl;
      
      const int &playerNum = ntm->playerNum;
      const AbsCoord &robotPos = AbsCoord(ntm->pose[0],ntm->pose[1],ntm->pose[2]);
      const AbsCoord &ballPosAbs = AbsCoord(ntm->absBallPos[0], ntm->absBallPos[1], ntm->absBallPos[2]);
      const RRCoord &ballPosRR = RRCoord(ntm->ballPosRR[0], ntm->ballPosRR[1], ntm->ballPosRR[2]);

      const AbsCoord &ballPosRRC = AbsCoord(ntm->ballPosRRC[0],ntm->ballPosRRC[1],ntm->ballPosRRC[2]);
      const AbsCoord &ballVelRRC = AbsCoord(ntm->ballVelRRC[0], ntm->ballVelRRC[1], ntm->ballVelRRC[2]);
      const bool &haveBallUpdate = ntm->haveBallUpdate;
      SharedStateEstimationBundle sharedStateEstimationBundle = SharedStateEstimationBundle(
                     robotPos,
                     ballPosRRC,
                     ballVelRRC,
                     haveBallUpdate
      );

      const int secondsSinceLastKick = ntm->secondsSinceLastKick;
      const int role = ntm->role;
      const bool playingBall = ntm->playingBall;
      const bool needAssistance = ntm->needAssistance;
      const bool isAssisting = ntm->isAssisting;
      const bool isKickedOff = ntm->isKickedOff;
      const float walkingToX = ntm->walkingTo[0];
      const float walkingToY = ntm->walkingTo[1];
      const float walkingToH = ntm->walkingTo[2];
      const bool kickNotification = ntm->kickNotification;
      BehaviourSharedData behaviourSharedData = BehaviourSharedData(
                                                      secondsSinceLastKick,
                                                      role,
                                                      playingBall,
                                                      needAssistance,
                                                      isAssisting,
                                                      isKickedOff,
                                                      walkingToX,
                                                      walkingToY,
                                                      walkingToH,
                                                      kickNotification
      );

      const ActionCommand::Body::ActionType acb = ntm->acb;
      const float uptime = ntm->upTime;
      const uint8_t gameState = ntm->gameState;

      
      BroadcastData bd = BroadcastData(
                                 playerNum,
                                 robotPos,
                                 // readFrom(stateEstimation, ballPos),
                                 ballPosAbs,
                                 // readFrom(stateEstimation, ballPosRR),
                                 ballPosRR,
                                 // readFrom(stateEstimation, sharedStateEstimationBundle),
                                 sharedStateEstimationBundle,
                                 // readFrom(behaviour, behaviourSharedData),
                                 behaviourSharedData,
                                 // readFrom(motion, active).body.actionType,
                                 acb,
                                 // readFrom(motion, uptime),
                                 uptime,
                                 // readFrom(gameController, gameState)
                                 gameState
                                 );

      SPLStandardMessage m = SPLStandardMessage(
                              playerNum,
                              ntm->teamNum,
                              ntm->fallen,
                              robotPos,
                              ntm->ballAge,
                              ballPosAbs,
                              bd

      );

      // if (m->playerNum >= 1 && m->playerNum <= ROBOTS_PER_TEAM &&
      //     m->teamNum == readFrom(gameController, our_team).teamNumber &&
      //     (m->numOfDataBytes == sizeof(BroadcastData) or true) &&
      //     (bd->sanityCheck() or true)) {
      if (true){

         // Not sure why Oleg thought it was ok to modify someone else's blackboard, especially in a different thread
         // This should be based on lastReceived as that is lastReceived's purpose
         std::vector<bool> pendingIncomingUpdates = readFrom(stateEstimation, havePendingIncomingSharedBundle);
         pendingIncomingUpdates[ntm->playerNum - 1] = true;
         writeTo(stateEstimation, havePendingIncomingSharedBundle, pendingIncomingUpdates);

         writeTo(receiver, message[m.playerNum - 1], m);
         writeTo(receiver, data[m.playerNum - 1], bd);
         // TODO (jayen): use ms as we could get two packets in the same second
         writeTo(receiver, lastReceived[m.playerNum - 1], time(NULL));

         // calculate incapacitated
         bool incapacitated = false;
         if (readFrom(gameController, our_team).players[m.playerNum - 1].penalty
             != PENALTY_NONE) {
            incapacitated = true;
         }

         const ActionCommand::Body::ActionType &acB =
            readFrom(receiver, data)[m.playerNum - 1].acB;
         incapacitated |= isIncapacitated(acB);

         std::vector<bool> incapacitatedVec = readFrom(receiver, incapacitated);
         incapacitatedVec[m.playerNum - 1] = incapacitated;
         writeTo(receiver, incapacitated, incapacitatedVec);
      }
   } else {
      llog(WARNING) << "Received packet of " << size << " bytes, but expected "
                                                        "packet of " << sizeof(NewTeamMessage) << " bytes."  << endl;
   }
   startReceive(this, &TeamReceiver::naoHandler);
}

void TeamReceiver::stdoutHandler(const boost::system::error_code &error,
                                 std::size_t size) {
   SPLStandardMessage* m = (SPLStandardMessage*)recvBuffer;
   BroadcastData* bd = (BroadcastData*)m->data;
   cout << "Received data from player " << bd->playerNum << endl;
   startReceive(this, &TeamReceiver::stdoutHandler);
}

void TeamReceiver::tick() {
   std::vector<bool> incapacitatedVec = readFrom(receiver, incapacitated);
   for (int robot = 0; robot < ROBOTS_PER_TEAM; ++robot) {
      if (time(NULL) - readFrom(receiver, lastReceived)[robot] > SECS_TILL_INCAPACITATED) {
         incapacitatedVec[robot] = true;
      }
   }
   writeTo(receiver, incapacitated, incapacitatedVec);
}
