#pragma once

#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>
#include "types/BehaviourRequest.hpp"
#include "types/BehaviourSharedData.hpp"

/**
 * Data Behaviour module will be sharing with others.
 */
struct BehaviourBlackboard {
    explicit BehaviourBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    
    BehaviourRequest request[2]; // double buffer to avoid concurrent access with motion
    int readBuf;
    std::string skill;
    std::string headskill;
    std::string positioning;
    BehaviourSharedData behaviourSharedData;
    bool remoteStiffen;
    bool useGetups;
};
