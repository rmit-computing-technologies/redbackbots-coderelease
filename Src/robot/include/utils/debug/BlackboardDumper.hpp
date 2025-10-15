/**
 * @file BlackboardDumper.hpp
 * 
 * Only for debugging purposes, dump blackboard.
*/

#pragma once

#include "utils/debug/DebugDumper.hpp"

#ifdef NDEBUG

    class BlackboardDumper : public DebugDumper {
    public:
        // Create dumper from blackboard configuration
        BlackboardDumper(Blackboard *blackboard) : DebugDumper(blackboard) {};
        virtual ~BlackboardDumper() {};

        inline void dumpBlackboard(const Blackboard *blackboard) {};
    };

#else

    #include <string>
    #include <fstream>

    #include "utils/Logger.hpp"

    class BlackboardDumper : public DebugDumper {
    public:
        BlackboardDumper(Blackboard *blackboard) :
            DebugDumper(blackboard)
        {
            if (dump) {
                std::string blackboardFile = dir + "/dump_blackboard.bb2";
                llog(INFO) << "Dumping blackboard to: " << blackboardFile << std::endl;
                blackboardDumpStream = std::ofstream(blackboardFile);
            }
        };
        virtual ~BlackboardDumper() {};

        // Dump contents of blackboard
        void dumpBlackboard(const Blackboard *blackboard) {
            if (dump) {
                if (dumpTimer.elapsed_us() > dumpRate) {
                    blackboard->serialise(blackboardDumpStream);
                    dumpTimer.restart();
                }
            }
        };

    private:
        // Dump stream for blackboard
        std::ofstream blackboardDumpStream;
    };

#endif