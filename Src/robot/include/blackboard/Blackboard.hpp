#pragma once

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options/variables_map.hpp>
#include <memory>

#include "communication/serialisation/ProtobufSerialisable.hpp"
#include "communication/transmitter/TransmitterDefs.hpp"
#include "utils/Logger.hpp"

// Blackboard Modules
#include "blackboard/modulesList.hpp"

/**
 * Macro to wrap reads to module's blackboard.
 * @param module which module's blackboard to read
 * @param component the component to read
 */
#define readFrom(module, component) \
    blackboard->read(&(blackboard->module->component))

/**
 * Macro to wrap reads to a debugger module blackboard.
 * Compiled out if NDEBUG is set.
 * @param module which debugger module blackboard to read
 * @param component the component to read
 */
#ifdef TMP_NDEBUG
#define readFrom_debugger(module, component) \
    static_cast<void>(0)
#else
#define readFrom_debugger(module, component) \
    blackboard->read(&(blackboard->debugger->module->component))
#endif

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
 * Macro to wrap writes to a debugger module blackboard.
 * Compiled out if NDEBUG is set.
 * @param module which debugger module blackboard to write
 * @param component the component to write
 * @param value the value to be written
 */
#ifdef TMP_NDEBUG
#define writeTo_debugger(module, component, value) \
    static_cast<void>(0)
#else
#define writeTo_debugger(module, component, value) \
    blackboard->write(&(blackboard->debugger->module->component), value)
#endif

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
struct DebuggerBlackboard;
struct GameControllerBlackboard;
struct KinematicsBlackboard;
struct MotionBlackboard;
struct PerceptionBlackboard;
struct WhistleBlackboard;
struct ReceiverBlackboard;
struct EventTransmitterBlackboard;
struct EventReceiverBlackboard;
struct StateEstimationBlackboard;
struct SynchronisationBlackboard;
struct ThreadBlackboard;
struct VisionBlackboard;


class Blackboard : ProtobufSerialisable {
private:
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

    /**
     * deserialises the blackboard with protobuf for loading from a file
     * or network
     */
    void deserialise(std::istream&);

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

    /* Data Whistle system will be sharing with others */
    boost::shared_ptr<WhistleBlackboard> whistle;

    /* Data GameController will be sharing */
    boost::shared_ptr<GameControllerBlackboard> gameController;

    /* Data Motion module will be sharing with others */
    boost::shared_ptr<MotionBlackboard> motion;

    /* Data received from friendly robots */
    boost::shared_ptr<ReceiverBlackboard> receiver;

    /* Event data we will be sharing with others */
    boost::shared_ptr<EventTransmitterBlackboard> eventTransmitter;
    
    /* Event data received from friendly robots */
    boost::shared_ptr<EventReceiverBlackboard> eventReceiver;

    /* Data ThreadWatcher will be sharing with others */
    boost::shared_ptr<ThreadBlackboard> thread;

    /* Locks used for inter-thread synchronisation */
    boost::shared_ptr<SynchronisationBlackboard> locks;

    /* Data Debuggers will be sharing with offnao / vatnao tools */
    boost::shared_ptr<DebuggerBlackboard> debugger;

private:
    /* Creates the Blackboard objects */
    void createInnerBlackboards();

    /* Creates debugger Blackboard objects */
    void createInnerDebuggerBlackboards();
};

// Template implementation - read
template<class T>
const T& Blackboard::read(const T *component) {
    if (component == nullptr) {
        llog(ERROR) << "Attempted to read a null component from Blackboard." << std::endl;
        throw std::runtime_error("Null component read from Blackboard.");
    }
    return *component;
}

// Template implementation - write
template<class T>
void Blackboard::write(T *component, const T& value) {
    if (component == nullptr) {
        llog(ERROR) << "Attempted to write to a null component in Blackboard." << std::endl;
        throw std::runtime_error("Null component write to Blackboard.");
    }
    *component = value;
}

