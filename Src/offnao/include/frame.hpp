#pragma once

#include <boost/shared_ptr.hpp>
#include <stdint.h> // cstdint is c++0x
#include <cstdio>
#include <ctime>

#include <map>
#include <string>

class Blackboard;

/*
 * Here we store all the info we wish to receive from the nao
 */
class Frame {
   public:
      Blackboard *blackboard;
      time_t timestamp;
      Frame() : blackboard(0) {
         timestamp = time(0);
      }
      ~Frame() {}
};
