#pragma once

#include "types/ActionCommand.hpp"
#include "types/BehaviourDebugInfo.hpp"
#include "types/BehaviourSharedData.hpp"

class BehaviourRequest {
   public:

      // Action to send to motion
      ActionCommand::All actions;

      // Information to share to teammates
      BehaviourSharedData behaviourSharedData;

      // Debug information to show in offnao
      BehaviourDebugInfo behaviourDebugInfo;

      BehaviourRequest() {
      };
};
