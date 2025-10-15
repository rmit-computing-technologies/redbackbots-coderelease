#pragma once

class Blackboard;

class Adapter {
   protected:
      Adapter(Blackboard *blackboard) {
         this->blackboard = blackboard;
      }
      Blackboard *blackboard;
};

