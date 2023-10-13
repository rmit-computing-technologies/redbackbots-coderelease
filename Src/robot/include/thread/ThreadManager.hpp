#pragma once

#include <chrono>
#include <pthread.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/time.h>
#include <string>
#include <thread>

#include <soccer.hpp>
#include <thread/ConcurrentMap.hpp>
#include <thread/Thread.hpp>
#include <utils/Logger.hpp>
#include <utils/speech.hpp>

// TODO: This is required for threadmanager to work - on below TODO
// It also affects files that inlucde this which don't obviously need blackboard
#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/ThreadBlackboard.hpp"

#define ALL_SIGNALS -1  // for indicating that we should register
                        // all signal handlers

// Use a forward delcaration of Blackboard only
class Blackboard;

// Forward Declare Thread Manager for run args
class ThreadManager;

// Map of points to jump to on restoring a crashed thread
extern ConcurrentMap<pthread_t, jmp_buf*> jumpPoints;

// Args required for running the thread
struct SafelyRunArgs {
   ThreadManager *threadManager;
   Blackboard *blackboard;
};

template<class T, void(T::*mem_fn) (Blackboard *)> void* thunk(void* args);

void overtimeAlert(int);
void handleSignals(int sigNumber, siginfo_t* info, void*);
void registerSignalHandlers(int signal = ALL_SIGNALS);

class ThreadManager {
public:
   std::string name;
   pthread_t pthread;
   bool running;

   /**
    * Setup the thread with a set name.
    * Optionally, restrict the thread to executed once every padMicroseconds
    * If not set, (or set to 0), the thread will execute as frequently as possible
    */
   ThreadManager(const char *name, unsigned int padMicroseconds = 0);
   ~ThreadManager();

   template <class T> void run(Blackboard *bb);

   void join();

private:
   template <class T>
   void safelyRun(Blackboard *bb);
   SafelyRunArgs args;

   // Duration to iterate the thread on
   std::chrono::microseconds padMicroseconds;
   bool noPadding;
};



template<class T, void(T::*mem_fn) (Blackboard *)> void* thunk(void* args) {
   void *p = ((SafelyRunArgs*)args)->threadManager;
   Blackboard *b = ((SafelyRunArgs*)args)->blackboard;
   (static_cast<T*>(p)->*mem_fn)(b);
   return 0;
}

template <class T> void ThreadManager::run(Blackboard *bb) {
   // initialise logger(name)
   // TODO: TW: What is the purpose of this??
   bb->thread->configCallbacks[name];
   
   args.blackboard = bb;
   args.threadManager = this;
   llog(INFO) << "Running ThreadManager for " << name << std::endl;
   if (name == "Motion") {
      // set up real-time priorities
      struct sched_param param;
      int policy;
      pthread_attr_t attr;
      pthread_attr_init(&attr);
      pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
      pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
      param.sched_priority = 65;
      pthread_attr_setschedparam(&attr, &param);
      pthread_create(&pthread, &attr, &thunk<ThreadManager, &ThreadManager::safelyRun<T> >, &args);
      pthread_attr_destroy(&attr);
      pthread_getschedparam(pthread, &policy, &param);
      llog(INFO) << "Motion granted priority: " << param.sched_priority << std::endl;
   } else {
      pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T> >, &args);
   }
   running = true;

}

template <class T>
void ThreadManager::safelyRun(Blackboard *bb) {
   Thread::name = name.c_str();
   pthread_t threadID = pthread_self();
   llog(INFO) << "Registering thread name '" << name
              << "' with ID " << threadID << std::endl;

   if (name == "Perception") {
      signal(SIGALRM, overtimeAlert);
   }
   while (!attemptingShutdown) {
      try {
         // we don't care if this leaks
         // but it shouldn't reach this line again in this thread
         jumpPoints[threadID] = reinterpret_cast<jmp_buf*>(malloc(sizeof(jmp_buf)));
         if (!jumpPoints[threadID]) {
            llog(FATAL) << "malloc failed for" << name << "\n";
            throw std::runtime_error("Malloc failed in ThreadManager");
         }
         llog(INFO) << "Thread " << name << " started\n";

         // register jump point for where to resume if we crash
         if (!setjmp(*jumpPoints[threadID])) {
            T t(bb);
            std::chrono::system_clock::time_point timer;
            std::chrono::microseconds elapsedMicroseconds(0);

            while (!attemptingShutdown) {
               timer = std::chrono::system_clock::now();

               // set watchdog timer to alert us about stuck threads
               if (name == "Perception") {
                  struct itimerval itval5;
                  itval5.it_value.tv_sec = 5;
                  itval5.it_value.tv_usec = 0;
                  itval5.it_interval.tv_sec = 0;
                  itval5.it_interval.tv_usec = 0;
                  setitimer(ITIMER_REAL, &itval5, NULL);
               }

               // Execute one cycle of the module
               t.tick();

               // unset watchdog timer to alert us about stuck threads
               if (name == "Perception") {
                  struct itimerval itval0;
                  itval0.it_value.tv_sec = 0;
                  itval0.it_value.tv_usec = 0;
                  itval0.it_interval.tv_sec = 0;
                  itval0.it_interval.tv_usec = 0;
                  setitimer(ITIMER_REAL, &itval0, NULL);
               }

               elapsedMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now() - timer);
               llog(TRACE) << "Thread '" << name << "' took "
                          << elapsedMicroseconds.count() << " us." << std::endl;
               if (elapsedMicroseconds < padMicroseconds) {
                  if (elapsedMicroseconds >= std::chrono::microseconds(0)) {
                     // usleep(padMicroseconds - elapsedMicroseconds);
                     std::this_thread::sleep_for(padMicroseconds - elapsedMicroseconds);
                  } else {
                     llog(ERROR) << "ERROR: Thread " << name << " duration invalid "
                                 << elapsedMicroseconds.count() << "us!" << std::endl;
                  }
               } else if (name == "Perception" && elapsedMicroseconds > std::chrono::microseconds(1000000)) {
                  llog(ERROR) << "WARNING: Thread " << name << " ran overtime: "
                              << elapsedMicroseconds.count() / 1000  << "ms!" << std::endl;
                  SAY("perception overtime");
               }
            }
         } else {
            //llog(DEBUG) << "Thread " << name << " jump points missing/NULL." << std::endl;
         }
         llog(INFO) << "Thread " << name << " terminated." << std::endl;
      } catch(const std::exception & e) {
         SAY("exception caught");
         free(jumpPoints[threadID]);
         // usleep(500000);
         std::this_thread::sleep_for(std::chrono::microseconds(500000));
         llog(ERROR) << "exception derivative was caught with error msg: "
                     << e.what() << std::endl;
         llog(ERROR) << "in " << name
                     << " with id " << threadID << std::endl;
      } catch(...) {
         SAY("exception caught");
         free(jumpPoints[threadID]);
         // usleep(500000);
         std::this_thread::sleep_for(std::chrono::microseconds(500000));
         llog(ERROR) << "Something was thrown from "
                     << name
                     << " with id " << threadID << std::endl;
      }
   }
}
