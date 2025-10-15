#pragma once

#include <boost/program_options/variables_map.hpp>
#include <memory>

#ifdef TMP_NDEBUG

struct DebuggerBlackboard {
    explicit DebuggerBlackboard() {};
};

#else

// Forward declarations
class VisionDebuggerBlackboard;

/**
 * Data for debugging.
 * 
 * Information here is only sent to offnao / vatnao.
 * This class is only compiled for Debug and Develop targets.
 * Otherwise for competition mode, this is 'compiled' out.
 * 
 * This blackboard object serves as a wrapper around all of the
 * debugger blackboards.
 * 
 */
struct DebuggerBlackboard {
    explicit DebuggerBlackboard();
    virtual ~DebuggerBlackboard();

    void readOptions(const boost::program_options::variables_map& config);


    // Vision Debugger blackboard pointer reference
    std::shared_ptr<VisionDebuggerBlackboard> vision;
};

#endif
