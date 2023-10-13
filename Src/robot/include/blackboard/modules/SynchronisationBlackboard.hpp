#pragma once

#include <boost/thread/mutex.hpp>

/**
 * Macro to wrap acquiring a mutex on a the blackboard.
 * @param name which module's lock to acquire
 */
#define acquireLock(name) \
    (blackboard->locks->name)->lock();

/**
 * Macro to wrap releasing a mutex on the blackboard.
 * @param name which module's lock to release
 */
#define releaseLock(name) \
    (blackboard->locks->name)->unlock();

/**
 * Shared data for managing synchronisation and update of the Blacboard.
 */
struct SynchronisationBlackboard {
    explicit SynchronisationBlackboard();
    
    boost::mutex *buttons;
    boost::mutex *serialization;
};
