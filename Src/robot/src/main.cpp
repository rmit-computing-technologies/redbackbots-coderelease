#include <boost/program_options.hpp>
#include <chrono>
#include <cerrno>
#include <iostream>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

#include "soccer.hpp"

#include "thread/ThreadManager.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"
#include "motion/MotionAdapter.hpp"
#include "transmitter/OffNao.hpp"
#include "transmitter/Team.hpp"
#include "receiver/Team.hpp"
#include "gamecontroller/GameController.hpp"
#include "perception/PerceptionThread.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/vision/camera/NaoCamera.hpp"
#include "perception/vision/camera/NaoCameraV4.hpp"
#include "perception/vision/camera/CombinedCamera.hpp"

#define TICK_AS_FAST_AS_POSSIBLE 0

#define handle_error_en(en, msg) \
   do { \
      errno = en; \
      perror(msg); \
      exit(1); \
   } while (0)

namespace po = boost::program_options;

// TODO: Remove git_version string
// extern string git_version;

/**
 * Strcut for shutdown Timer callback.
 * This is prefered as a direct conversion from void* to int generates
 * precision conversion error messages.
 */
typedef struct {
  int time;
} ShutdownTimer_struct;

/**
 * A timer function.  After arg seconds, redbackbots will shut down.
 *
 * @param arg coerced to int.  seconds to shutdown
 * @returns NULL
 */
static void* shutdownTimer(void* arg) {
   ShutdownTimer_struct *t_arg = (ShutdownTimer_struct *) arg;
   int time = t_arg->time;
   if (time > 5) {
      std::this_thread::sleep_for(std::chrono::seconds(time - 5));
      // SAY("shutting down");
      std::this_thread::sleep_for(std::chrono::seconds(5));
   } else {
      std::this_thread::sleep_for(std::chrono::seconds(time));
   }
   attemptingShutdown = true;
   return nullptr;
}

/** Entry point for redbackbots application */
int main(int argc, char **argv) {
   po::variables_map vm;
   try {
      po::options_description generic("Generic options");
      generic.add_options()
         ("help,h", "produce help message")
         ("version,v", "print version string");

      po::options_description cmdline_options =
         store_and_notify(argc, argv, vm, &generic);

      if (vm.count("help")) {
         std::cout << cmdline_options << std::endl;
         return 1;
      }

      if (vm.count("version")) {
         // cout << "RedBackBots Nao soccer player " << git_version << endl;
         return 1;
      }

      // cout << "RedBackBots V." << git_version << endl;

      options_print(vm);
   } catch (po::error& e) {
      std::cerr << "Prog. Options error when parsing command line arguments: "
                << e.what() << std::endl;
      return 1;
   } catch (std::exception& e) {
      std::cerr << "Unknown error when parsing command line arguments: "
                << e.what() << std::endl;
      return 1;
   }

   offNao = false;
   attemptingShutdown = false;
   Thread::name = "main";

   std::cout << "Launching Logger" << std::endl;
   Logger::readOptions(vm);
   Logger::initialise();

   llog(INFO) << RBB_NAME_CAPS << " soccer library spinning up!" << std::endl;

   // Instantiate Blackboard
   Blackboard *blackboard = new Blackboard(vm);
   llog(INFO) << "Blackboard initialised" << std::endl;

   Camera *topCamera = NULL;
   Camera *botCamera = NULL;
   if (vm["thread.vision"].as<bool>()) {
      llog(INFO) << "Initialising v4 " << VIDEO_TOP << std::endl;
      topCamera = new NaoCameraV4(blackboard, VIDEO_TOP, "camera.top");

      llog(INFO) << "Initialising v4 " << VIDEO_BOTTOM << std::endl;
      botCamera = new NaoCameraV4(blackboard, VIDEO_BOTTOM, "camera.bot",
                                    IO_METHOD_MMAP,
                                    AL::kVGA);

      CombinedCamera::setCameraTop(topCamera);
      CombinedCamera::setCameraBot(botCamera);
   }

   registerSignalHandlers();

   // create thread managers
   ThreadManager perception("Perception", TICK_AS_FAST_AS_POSSIBLE); // as fast as the camera can provide as image
   ThreadManager motion("Motion"); // as fast as possible, waits on agent semaphore
   ThreadManager gameController("GameController", TICK_AS_FAST_AS_POSSIBLE); // as fast as possible, waits on udp read
   ThreadManager offnaoTransmitter("OffnaoTransmitter", 2000000); // .5fps limit
   ThreadManager teamTransmitter("TeamTransmitter", 500000); // 2fps limit
   ThreadManager teamReceiver("TeamReceiver", 100000); // 10fps limit (Congested WiFi: Higher than TeamTransmitter)

   // start threads
   if (vm["thread.perception"].as<bool>()) {
      perception.run<PerceptionThread>(blackboard);
      llog(INFO) << "Perception is running" << std::endl;
   }
   if (vm["thread.motion"].as<bool>()) {
      motion.run<MotionAdapter>(blackboard);
      llog(INFO) << "Motion is running" << std::endl;
   }
   if (vm["thread.gamecontroller"].as<bool>()) {
      gameController.run<GameController>(blackboard);
      llog(INFO) << "GameController is running" << std::endl;
   }
   if (vm["thread.offnaotransmitter"].as<bool>()) {
      offnaoTransmitter.run<OffNaoTransmitter>(blackboard);
      llog(INFO) << "Off-Nao Transmitter is running" << std::endl;
   }
   if (vm["thread.naotransmitter"].as<bool>()) {
      teamTransmitter.run<TeamTransmitter>(blackboard);
      llog(INFO) << "Nao Transmitter is running" << std::endl;
   }
   if (vm["thread.naoreceiver"].as<bool>()) {
      teamReceiver.run<TeamReceiver>(blackboard);
      llog(INFO) << "Team Receiver is running" << std::endl;
   }

   if (vm["thread.shutdowntime"].as<int>()) {
      pthread_t timer;
      ShutdownTimer_struct t_arg = {vm["thread.shutdowntime"].as<int>()};
      pthread_create(&timer, NULL, &shutdownTimer, &t_arg);
      llog(INFO) << "Timer is running" << std::endl;
   }

   if (vm["calibration.camera"].as<string>() == "terminal") {
      llog(INFO) << "Camera calibration by terminal is running" << std::endl;
      Camera::terminalCalibration();
   }

   llog(INFO) << "Shutting Down" << std::endl;
   
   system("sudo pkill -9 -f /home/nao/whistle/whistle_detector.py");
   // sys.exit(python3)

   teamReceiver.join();
   llog(DEBUG) << "Receiver thread joined\n";
   teamTransmitter.join();
   llog(DEBUG) << "Transmitter thread joined\n";
   offnaoTransmitter.join();
   llog(DEBUG) << "Offnao thread joined\n";
   gameController.join();
   llog(DEBUG) << "GC thread joined\n";
   motion.join();
   llog(DEBUG) << "Motion thread joined\n";
   perception.join();
   llog(DEBUG) << "Perception thread joined\n";

   
   
   delete blackboard;
   delete topCamera;
   delete botCamera;

   llog(INFO) << RBB_NAME_CAPS << " Completed" << std::endl;

   return 0;
}

