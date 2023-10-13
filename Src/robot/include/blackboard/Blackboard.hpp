#pragma once

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/function.hpp>
#include <deque>
#include <map>
#include <memory>
#include <signal.h>
#include <string>
#include <vector>

#include "soccer.hpp"
#include "transmitter/TransmitterDefs.hpp"
#include "utils/Logger.hpp"
#include "utils/ProtobufSerialisable.hpp"
#include "modules/VisionBlackboard.hpp"


namespace offnao
{
   class Blackboard;
}

/**
 * Macro to wrap reads to module's blackboard.
 * @param module which module's blackboard to read
 * @param component the component to read
 */
#define readFrom(module, component) \
    blackboard->read(&(blackboard->module->component))

/**
 * Macro to wrap array reads from module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to read from
 * @param component while component to be written
 * @param dest where to write to
 */
#define readArray(module, component, dest) \
    memcpy(dest, blackboard->module->component, \
           sizeof(blackboard->module->component));

/**
 * Macro to wrap writes to module's blackboard.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeTo(module, component, value) \
    blackboard->write(&(blackboard->module->component), value);

/**
 * Macro to wrap array writes to module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeArray(module, component, value) \
    memcpy(blackboard->module->component, value, \
           sizeof(blackboard->module->component));

/**
 * Blackboard shared memory class, used for inter-module communication.
 * 
 * Blackboard consists of a set of inner-class blackboards for each module.
 * Each module is defined in separate header files in blackboard/modules.
 * Classes that require the full definition of a given module should include the
 *      relevant module header file.
 * Constructors for each module are in Blackboard.cpp.
 * 
 * The safeRun templated function has access to the blackboard
 * to look up thread timings and things like that
 */

class NetworkReader;
class OverviewTab;

struct BehaviourBlackboard;
struct GameControllerBlackboard;
struct KinematicsBlackboard;
struct MotionBlackboard;
struct PerceptionBlackboard;
struct ReceiverBlackboard;
struct StateEstimationBlackboard;
struct SynchronisationBlackboard;
struct ThreadBlackboard;



class Blackboard : ProtobufSerialisable {
    // Functions
    template <class T> friend void *safelyRun(void * foo);

    public:
        // used internally, and by the blackboard wrapper
        explicit Blackboard();
        explicit Blackboard(const boost::program_options::variables_map &vm);
        virtual ~Blackboard();

        /* Function to read a component from the Blackboard */
        template<class T> const T& read(const T *component);

        /* Write a component to the Blackboard */
        template<class T> void write(T *component, const T& value);

        /**
         * serialises the blackboard with protobuf for storing to a file
         * or network
         */
        void serialise(std::ostream&) const;
        static void serialise(const Blackboard &cpp, offnao::Blackboard &pb);

        /**
         * deserialises the blackboard with protobuf for loading from a file
         * or network
         */
        void deserialise(std::istream&);
        static void deserialise(Blackboard &cpp, const offnao::Blackboard &pb);

        /* We now have a private inner-class blackboard for each of the
         * modules. Appropriate constructors should be placed in Blackboard.cpp
         * since there's no guarantee as to which order threads start in. */

        // TODO: (find a better solution) private:

        /* Stores command-line/config-file options
         * should be read by modules on start
         * functionality may be added later to allow change at runtime */
        boost::program_options::variables_map config;

        /**
         * the mask of what is stored/loaded from a file or network
         */
        OffNaoMask_t mask;

        /**
         * NOTE: (TW)
         * All of these are boost shared pointers and NOT C++14 shared pointers.
         * This is because Boost v1.59 does not support used std::shared_ptr with Boost::Python.
         * Thus boost pointers must be used.
         * If a version of Boost v1.63 or later is used, then std::shared_ptr can be used
         * 
         * See boost::python::register_ptr_to_python documentation.
         * See release note for Boost v1.63 (https://www.boost.org/users/history/version_1_63_0.html)
         */

        /* Options callback for changes at runtime */
        void readOptions(const boost::program_options::variables_map& config);

        /* Data Kinematics module will be sharing with others */
        boost::shared_ptr<KinematicsBlackboard> kinematics;

        /* Data Behaviour module will be sharing with others */
        boost::shared_ptr<BehaviourBlackboard> behaviour;

        /* Data Localisation module will be sharing */
        boost::shared_ptr<StateEstimationBlackboard> stateEstimation;

        /* Data Vision module will be sharing with others */
        boost::shared_ptr<VisionBlackboard> vision;

        /* Data Perception system will be sharing with others */
        boost::shared_ptr<PerceptionBlackboard> perception;

        /* Data GameController will be sharing */
        boost::shared_ptr<GameControllerBlackboard> gameController;

        /* Data Motion module will be sharing with others */
        boost::shared_ptr<MotionBlackboard> motion;

        /* Data received from friendly robots */
        boost::shared_ptr<ReceiverBlackboard> receiver;

        /* Data ThreadWatcher will be sharing with others */
        boost::shared_ptr<ThreadBlackboard> thread;

        /* Locks used for inter-thread synchronisation */
        boost::shared_ptr<SynchronisationBlackboard> locks;

    private:
        /* Creates the Blackboard objects */
        void createInnerBlackboards();
};

// TODO: These are moved from Blackboard.tcc
template<class T>
const T& Blackboard::read(const T *component) {
    return *component;
}

template<class T>
void Blackboard::write(T *component, const T& value) {
    *component = value;
}

