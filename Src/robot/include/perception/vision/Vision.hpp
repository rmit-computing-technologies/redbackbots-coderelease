#pragma once

#include "blackboard/Blackboard.hpp"
#include "perception/vision/Detector.hpp"
#include "perception/vision/VisionInfoIn.hpp"
#include "perception/vision/VisionInfoOut.hpp"
#include "perception/vision/fovea/Region.hpp"
#include "utils/Timer.hpp"
#include "utils/ml/asmjit_forwdec.hpp"

#include <list>
#include <utility>
#include <vector>

// Forward Declare vision info middle layer
class VisionInfoMiddle;

class Vision {
public:

    /**
     * Construct Vision. Blackboard is required for the current loaded config
     */
    Vision(Blackboard* blackboard);
    ~Vision();

    /**
     * Process a frame, running appropriate finders and detectors.
     * Vision logic mainly happens here.
     */
    VisionInfoOut processFrame(const VisionInfoIn& info_in);

private:
    // Configure algorithms to run, and the order in which to run each algorithm
    void setupAlgorithms_(Blackboard* blackboard);

    // Execute the algorithms for the given frame
    void runAlgorithms_();

    // Reset the internal middle/out vision info structs for the next frame
    void resetVisionInfo();

    // Order of detectors to iterate, mapped to their associated timer
    std::vector<std::pair<std::shared_ptr<Detector>, int> > detectors_;
    std::vector<std::pair<std::shared_ptr<Detector>, int> > referee_detectors_;

    // Save blackbord ref
    Blackboard* blackboard;

    // Vision info components
    VisionInfoIn* info_in_;
    VisionInfoMiddle* info_middle_;
    VisionInfoOut* info_out_;

    // JitRuntime instance of this vision thread for all detectors using CompiledNN
    asmjit::_abi_1_9::JitRuntime* asmjitRuntime;

    // Keeps track of run times for average output.
    int frameCount;
    std::vector<int> timers;
};
