#pragma once

#include "Nao.hpp"
#include <string>
#include "blackboard/Adapter.hpp"

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

      int tickCounter;
      boost::asio::io_service service;
      boost::asio::ip::udp::socket socket;
      boost::asio::ip::udp::endpoint gameControllerEndpoint;
};
