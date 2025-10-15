#pragma once

#include <boost/program_options/variables_map.hpp>

enum WhistleDetectionState {
    dontKnow,
    notDetected,
    isDetected
};

struct WhistleBlackboard {
    explicit WhistleBlackboard();
    void readOptions(const boost::program_options::variables_map& config);

    WhistleDetectionState whistleDetectionState;

    bool whistleThreadCrashed;
};