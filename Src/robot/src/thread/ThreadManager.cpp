#include "thread/ThreadManager.hpp"

ConcurrentMap<pthread_t, jmp_buf*> jumpPoints;

ThreadManager::ThreadManager(const char *name, unsigned int padMicroseconds) {
   this->name = name;
   running = false;
   this->padMicroseconds = std::chrono::microseconds(padMicroseconds);
   this->noPadding = padMicroseconds == 0;
}

ThreadManager::~ThreadManager() {
   join();
}

void ThreadManager::join() {
   if (running) {
      pthread_join(pthread, NULL);
   }
   running = false;
}

void overtimeAlert(int) {
   // Thread::name is thread local, and the signal handler always
   // happens in main thread, even if perception thread has frozen
   // TODO (jayen): use sigmask to have this happen in the thread itself, i think
   SAY(std::string(Thread::name) + " thread has frozen");
   //system("aplay /opt/aldebaran/share/naoqi/wav/fall_jpj.wav");
   // TODO (jayen): use sigaction() to pass data to this function
   signal(SIGALRM, overtimeAlert);
   // TODO (jayen): see if using alarm() works and is simpler
   struct itimerval itval5;
   itval5.it_value.tv_sec = 5;
   itval5.it_value.tv_usec = 0;
   itval5.it_interval.tv_sec = 0;
   itval5.it_interval.tv_usec = 0;
   setitimer(ITIMER_REAL, &itval5, NULL);
}

/**
 * The signal handler. Handles the signal and flag that the thread has died
 * and allow the watcher thread to restart it.
 * @param sigNumber The POSIX signal identifier
 * @param info Signal info struct for the signal
 * @see registerSignalHandler
 */
void handleSignals(int sigNumber, siginfo_t* info, void*) {
   // End the RedBackBots module [CTRL-C]. Call all destructors
   if (sigNumber == SIGINT) {
      std::cerr << std::endl;
      std::cerr << "###########################" << std::endl;
      std::cerr << "##    SIGINT RECEIVED    ##" << std::endl;
      std::cerr << "##  ATTEMPTING SHUTDOWN  ##" << std::endl;
      std::cerr << "###########################" << std::endl;
      attemptingShutdown = true;
   } else if (sigNumber == SIGTERM) {
      std::cerr << std::endl;
      std::cerr << "###########################" << std::endl;
      std::cerr << "##   SIGTERM RECEIVED    ##" << std::endl;
      std::cerr << "##  ATTEMPTING SHUTDOWN  ##" << std::endl;
      std::cerr << "###########################" << std::endl;
      attemptingShutdown = true;
   } else {
      // re-register the signal handler
	   SAY("crash detected");
      llog(DEBUG) << std::string(Thread::name) << " crash detected " << std::endl;
      registerSignalHandlers(sigNumber);
      pthread_t thread = pthread_self();

      std::cerr << std::string(Thread::name) << " with id " << thread <<
      " received signal " << sigNumber << " and is restarting" << std::endl;
      llog(ERROR) << std::string(Thread::name) << " with id "
                  << thread << " received signal "
                  << sigNumber << " and is restarting" << std::endl;

      longjmp(*jumpPoints[thread], 1);
   }
   return;
}


/**
 * @param signal default param is ALL_SIGNALS which is -1
 */
void registerSignalHandlers(int signal) {
   // setup the sigaction
   struct sigaction act;
   act.sa_sigaction = handleSignals;
   sigemptyset(&act.sa_mask);
   act.sa_flags = SA_SIGINFO | SA_RESETHAND;

   // register the signal handlers
   if (signal == SIGINT || signal == ALL_SIGNALS)
      sigaction(SIGINT, &act, NULL);   // CTRL-C termination
   if (signal == SIGTERM || signal == ALL_SIGNALS)
      sigaction(SIGTERM, &act, NULL);   // kill -15 termination
   if (signal == SIGSEGV || signal == ALL_SIGNALS)
      sigaction(SIGSEGV, &act, NULL);  // seg fault
   if (signal == SIGFPE || signal == ALL_SIGNALS)
      sigaction(SIGFPE, &act, NULL);   // floating point exception
   if (signal == SIGSTKFLT || signal == ALL_SIGNALS)
      sigaction(SIGSTKFLT, &act, NULL);   // stack faults
   if (signal == SIGHUP || signal == ALL_SIGNALS)
      sigaction(SIGHUP, &act, NULL);   // lost controlling terminal
}

