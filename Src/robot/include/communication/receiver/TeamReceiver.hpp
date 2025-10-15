#pragma once

#include "blackboard/Adapter.hpp"
#include "communication/receiver/NaoReceiver.hpp"
#include "types/BroadcastData.hpp"
#include "blackboard/Adapter.hpp"
#include "utils/EventSerializer.hpp"
#include "utils/EventDefinitions.hpp"
#include "blackboard/modules/EventReceiverBlackboard.hpp"
#include <string>

// Forward Declaration
class Blackboard;

class TeamReceiver : Adapter, NaoReceiver {
   public:
      /**
       * Constructor.  Opens a socket for listening.
       */
      TeamReceiver(Blackboard *bb, void(TeamReceiver::*handler)
                   (const boost::system::error_code & error, std::size_t) =
                      &TeamReceiver::naoHandler);

      /**
       * One cycle of this thread
       */
      void tick();

   private:
      void naoHandler(const boost::system::error_code &error, std::size_t size);
      void stdoutHandler(const boost::system::error_code &error, std::size_t size);

      // Access the EventReceiverBlackboard from the Blackboard
      EventReceiverBlackboard& eventReceiverBB;
};
