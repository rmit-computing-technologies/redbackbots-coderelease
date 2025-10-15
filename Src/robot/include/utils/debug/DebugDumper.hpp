/**
 * @file DebugDumper.hpp
 * 
 * Only for debugging purposes, base class for dumping various contents to logging locations.
 * Dumper is only enabled in Develop/Debug builds. 
 * Release build 'compiles out' the dumper
*/

#pragma once

#ifdef NDEBUG

    // Forward Declaration
    class Blackboard;

    class DebugDumper {
    public:
        // Create dumper from blackboard configuration
        DebugDumper(Blackboard *blackboard) {};
        virtual ~DebugDumper() {};
    };

#else

    #include <string>
    #include <fstream>

    #include "blackboard/Blackboard.hpp"
    #include "blackboard/modulesList.hpp"
    #include "utils/Logger.hpp"
    #include "utils/Timer.hpp"

    class DebugDumper {
    public:
        // Create dumper from blackboard configuration
        DebugDumper(Blackboard *blackboard) {
            dump = blackboard->config["debug.dump"].as<bool>();

            // If dumping is enabled, then configure output locations
            if (dump) {
                dir = Logger::getLogDir();
                dumpRate = blackboard->config["debug.dumprate"].as<int>() * 1000;
                dumpTimer.restart();
            }
        };
        virtual ~DebugDumper() {};

    protected:
        // True if should dump
        bool dump;

        // Rate at which to Dump
        int dumpRate;

        // Timer for dumping
        Timer dumpTimer;

        // Root path to dump to
        std::string dir;
    };

#endif
