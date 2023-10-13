#pragma once

#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>
#include "perception/kinematics/Parameters.hpp"

/**
 * Shared data the Kinematics module will be sharing with others.
 */
struct KinematicsBlackboard {
    explicit KinematicsBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    
    bool isCalibrating;
    Parameters<float> parameters;
};
