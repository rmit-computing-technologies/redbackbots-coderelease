#pragma once

#include "blackboard/Adapter.hpp"
#include "communication/transmitter/NaoTransmitter.hpp"
#include <string>
#include "utils/EventSerializer.hpp"
#include "utils/EventDefinitions.hpp"
#include "blackboard/modules/EventTransmitterBlackboard.hpp"

class TeamTransmitter : Adapter, NaoTransmitter {
   public:
      /**
       * Constructor.  Opens a socket for listening.
       */
      TeamTransmitter(Blackboard *bb);
      ~TeamTransmitter();

      /**
       * One cycle of this thread
       */
      void tick();

   private:
      void sendToTeam();
      void sendToGameController();
      EVENT_HASH_TYPE computeEventHash(std::array<EventDataPtr, EVENTS_ARRAY_SIZE> events);

      bool haveTeamBallUpdate;
      bool haveBallScoreUpdate;
      boost::asio::io_service service;
      boost::asio::ip::udp::socket socket;
      boost::asio::ip::udp::endpoint gameControllerEndpoint;

      // Access the EventTransmitterBlackboard from the Blackboard
      EventTransmitterBlackboard& eventTransmitterBB;
};
