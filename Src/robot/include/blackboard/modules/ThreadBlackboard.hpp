#pragma once

#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>

/**
 * Shared data for managing the program threads.
 */
struct ThreadBlackboard {
    explicit ThreadBlackboard();
    
    std::map<std::string, boost::function<void(const boost::program_options::variables_map &)> > configCallbacks;
};
