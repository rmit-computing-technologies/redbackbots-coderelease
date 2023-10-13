/*
 * ReadySkillPositionAllocation.hpp
 *
 *  Created on: 11/07/2014
 *      Author: osushkov
 */

#pragma once

#include <vector>

class ReadySkillPositionAllocation {
public:
   int fromPlayerNum;
   int readyPositionAllocation0;
   int readyPositionAllocation1;
   int readyPositionAllocation2;
   int readyPositionAllocation3;
   int readyPositionAllocation4;

   ReadySkillPositionAllocation()
      : fromPlayerNum(0),
        readyPositionAllocation0(0),
        readyPositionAllocation1(0),
        readyPositionAllocation2(0),
        readyPositionAllocation3(0),
        readyPositionAllocation4(0)
   {}
};
