#include "communication/receiver/TeamReceiver.hpp"

#include <ctime>
#include <iostream>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/ReceiverBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "thread/Thread.hpp"
// #include "types/SPLStandardMessage.hpp"
// #include "types/NewTeamMessage.hpp"
#include "utils/defs/RobotDefinitions.hpp"
#include "utils/incapacitated.hpp"
#include "utils/Logger.hpp"

#include "types/events/AbstractEventData.hpp"
#include "utils/EventDefinitions.hpp"
#include "types/events/PlayerEventData.hpp"

using namespace std;

TeamReceiver::TeamReceiver(Blackboard *bb, void(TeamReceiver::*handler)
                     (const boost::system::error_code & error, std::size_t))
   : Adapter(bb), NaoReceiver(this,
                       handler,
                       (bb->config)["network.transmitter_base_port"].as<int>()
                       + (bb->config)["player.team"].as<int>()), 
      eventReceiverBB(*(bb->eventReceiver)) {
  llog(INFO) << "TeamReceiver initialized with a single EventReceiverBlackboard instance" << std::endl;
}

void TeamReceiver::naoHandler(const boost::system::error_code &error,
                       std::size_t size) {
   if (Thread::name == NULL) {
     Thread::name = "TeamReceiverBoostThread";
   }

   llog(INFO) << "Received raw data of size " << size << " bytes." << std::endl;

   // Try to parse the incoming packet using EventSerializer
   if (size > 0) {
     llog(INFO) << "Attempting to deserialize incoming data..." << std::endl;

     // Convert recvBuffer to std::vector<uint8_t>
     std::vector<uint8_t> buffer(recvBuffer, recvBuffer + size);

     // Update the EventReceiverBlackboard with deserialized data
     eventReceiverBB.decodeIncomingPacket(buffer);
    }

  //  SPLStandardMessage* m = reinterpret_cast<SPLStandardMessage*>(recvBuffer);
  //  // NewTeamMessage* ntm = reinterpret_cast<NewTeamMessage*>(recvBuffer);
  //  BroadcastData* bd = (BroadcastData*)m->data;
  //  if (size == sizeof(SPLStandardMessage)) {
  //    // std::cout << "NewTeamMessage Recieved!" << std::endl;
     
  //    const int &playerNum = m->playerNum;
  //    AbsCoord robotPos(bd->robotPos[0], bd->robotPos[1], bd->robotPos[2]);
  //    AbsCoord ballPosAbs(bd->ballPosAbs.x(), bd->ballPosAbs.y(), bd->ballPosAbs.theta());
  //    RRCoord ballPosRR(bd->ballPosRR.distance(), bd->ballPosRR.heading(), bd->ballPosRR.orientation());

  //    AbsCoord ballPosRRC(bd->sharedStateEstimationBundle.ballPosRRC.x(),
  //                        bd->sharedStateEstimationBundle.ballPosRRC.y(),
  //                        bd->sharedStateEstimationBundle.ballPosRRC.theta());
  //    AbsCoord ballVelRRC(bd->sharedStateEstimationBundle.ballVelRRC.x(),
  //                        bd->sharedStateEstimationBundle.ballVelRRC.y(),
  //                        bd->sharedStateEstimationBundle.ballVelRRC.theta());
  //    const bool &haveBallUpdate = bd->sharedStateEstimationBundle.haveBallUpdate;
  //    const bool &haveTeamBallUpdate = bd->sharedStateEstimationBundle.haveTeamBallUpdate;
  //    SharedStateEstimationBundle sharedStateEstimationBundle = SharedStateEstimationBundle(
  //               robotPos,
  //               ballPosRRC,
  //               ballVelRRC,
  //               haveBallUpdate,
  //               haveTeamBallUpdate
  //    );

  //    const int secondsSinceLastKick = bd->behaviourSharedData.secondsSinceLastKick;
  //    const int role = bd->behaviourSharedData.role;
  //    const bool playingBall = bd->behaviourSharedData.playingBall;
  //    const float playingBallScore = bd->behaviourSharedData.playingBallScore;
  //    llog(INFO) << "Playing ball score received: " << playingBallScore << std::endl;
  //    llog(INFO) << "From player: " << playerNum << std::endl;
  //    const bool needAssistance = bd->behaviourSharedData.needAssistance;
  //    const bool isAssisting = bd->behaviourSharedData.isAssisting;
  //    const bool isKickedOff = bd->behaviourSharedData.isKickedOff;
  //    const float walkingToX = bd->behaviourSharedData.walkingToX;
  //    const float walkingToY = bd->behaviourSharedData.walkingToY;
  //    const float walkingToH = bd->behaviourSharedData.walkingToH;
  //    const bool kickNotification = bd->behaviourSharedData.kickNotification;
  //    BehaviourSharedData behaviourSharedData = BehaviourSharedData(
  //                                        secondsSinceLastKick,
  //                                        role,
  //                                        playingBall,
  //                                        playingBallScore,
  //                                        needAssistance,
  //                                        isAssisting,
  //                                        isKickedOff,
  //                                        walkingToX,
  //                                        walkingToY,
  //                                        walkingToH,
  //                                        kickNotification
  //    );

  //    const ActionCommand::Body::ActionType acb = bd->acB;
  //    const float uptime = bd->uptime;
  //    const uint8_t gameState = bd->gameState;

     
  //    BroadcastData newBd = BroadcastData(
  //                        playerNum,
  //                        robotPos,
  //                        // readFrom(stateEstimation, ballPos),
  //                        ballPosAbs,
  //                        // readFrom(stateEstimation, ballPosRR),
  //                        ballPosRR,
  //                        // readFrom(stateEstimation, sharedStateEstimationBundle),
  //                        sharedStateEstimationBundle,
  //                        // readFrom(behaviour, behaviourSharedData),
  //                        behaviourSharedData,
  //                        // readFrom(motion, active).body.actionType,
  //                        acb,
  //                        // readFrom(motion, uptime),
  //                        uptime,
  //                        // readFrom(gameController, gameState)
  //                        gameState
  //                        );

  //    SPLStandardMessage newM = SPLStandardMessage(
  //                      m->playerNum,
  //                      m->teamNum,
  //                      m->fallen,
  //                      robotPos,
  //                      m->ballAge,
  //                      ballPosAbs,
  //                      newBd

  //    );

     // if (m->playerNum >= 1 && m->playerNum <= ROBOTS_PER_TEAM &&
     //     m->teamNum == readFrom(gameController, our_team).teamNumber &&
     //     (m->numOfDataBytes == sizeof(BroadcastData) or true) &&
     //     (bd->sanityCheck() or true)) {
  //    if (true){

  //      // Not sure why Oleg thought it was ok to modify someone else's blackboard, especially in a different thread
  //      // This should be based on lastReceived as that is lastReceived's purpose
  //      std::vector<bool> pendingIncomingUpdates = readFrom(stateEstimation, havePendingIncomingSharedBundle);
  //      pendingIncomingUpdates[m->playerNum - 1] = true;
  //      writeTo(stateEstimation, havePendingIncomingSharedBundle, pendingIncomingUpdates);

  //      writeTo(receiver, message[m->playerNum - 1], newM);
  //      writeTo(receiver, data[m->playerNum - 1], newBd);
  //      // TODO (jayen): use ms as we could get two packets in the same second
  //      writeTo(receiver, lastReceived[m->playerNum - 1], time(NULL));

  //      // calculate incapacitated
  //      bool incapacitated = false;
  //      if (readFrom(gameController, our_team).players[m->playerNum - 1].penalty
  //         != PENALTY_NONE) {
  //        incapacitated = true;
  //      }

  //      const ActionCommand::Body::ActionType &acB =
  //        readFrom(receiver, data)[m->playerNum - 1].acB;
  //      incapacitated |= isIncapacitated(acB);

   //       std::vector<bool> incapacitatedVec = readFrom(receiver, incapacitated);
   //       incapacitatedVec[m.playerNum - 1] = incapacitated;
   //       writeTo(receiver, incapacitated, incapacitatedVec);
   //    }
   // } else {
   //    llog(WARNING) << "Received packet of " << size << " bytes, but expected "
   //                                                      "packet of " << sizeof(NewTeamMessage) << " bytes."  << endl;
   // }
   startReceive(this, &TeamReceiver::naoHandler);
}

void TeamReceiver::stdoutHandler(const boost::system::error_code &error,
                                 std::size_t size) {
   // SPLStandardMessage* m = (SPLStandardMessage*)recvBuffer;
   // BroadcastData* bd = (BroadcastData*)m->data;
   llog(INFO) << "Received data from player " << eventReceiverBB.getLatestPlayer() << std::endl;
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
