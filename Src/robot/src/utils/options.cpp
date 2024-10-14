#include <boost/program_options.hpp>
#include <ext/slist>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

#include "redbackbots.hpp"
#include "utils/options.hpp"
#include "transmitter/TransmitterDefs.hpp"

namespace po = boost::program_options;
// using namespace std;
using namespace __gnu_cxx;

// note that uint8_t is treated like a char, so if you want uint8_t, use int and cast when you read

void populate_options(po::options_description &config_file_options) {
   po::options_description game_config("Game options");
   game_config.add_options()
      ("game.type,g", po::value<std::string>()->default_value(std::string("MATCH")),
      "Type of game/challenge (MATCH, DRIBBLE, OPEN, PASSING)");

   po::options_description player_config("Player options");
   player_config.add_options()
      ("player.number,n", po::value<int>()->default_value(2),
      "player number")
      ("player.team,T", po::value<int>()->default_value(18),
      "team number")
      ("player.playerIP", po::value<std::string>()->default_value(std::string("")),
      "player IP")
      ("player.bodyName", po::value<std::string>()->default_value(std::string("")),
      "player body name")
      ("player.headName", po::value<std::string>()->default_value(std::string("")),
      "player head name");

   std::string defaultLogDir = "/var/volatile/rbb";
   if (offNao) {
      defaultLogDir = "/tmp/rbb";
   }

   po::options_description whistles_config("Whistle options"); // This took 5 years to implement
   whistles_config.add_options()
      ("whistle.act_on_whistle_kickoff", po::value<bool>()->default_value(true),
      "Acts when whistle heard - Go to PLAYING state when in SET state and whistle heard - for kickoff")
      ("whistle.act_on_whistle_goal", po::value<bool>()->default_value(true),
      "Acts when whistle heard - Go to READY state when in PLAYING state and whistle heard - for Goal")
      ;

   po::options_description debug_config("Debugging options");
   debug_config.add_options()
      ("debug.log,l", po::value<std::string>()->default_value(std::string("SILENT")),
       "log level used by llog")
      ("debug.connect_pydevd", po::value<bool>()->default_value(false),
        "Whether to attempt connecting to the Python debugger")
      ("debug.file_watcher", po::value<bool>()->default_value(false),
        "Whether to start the inotify file watcher and reload Python on changes")
      ("debug.log.motion,m", po::value<bool>()->default_value(false),
       "allow llog from motion")
      ("debug.log.dir", po::value<std::string>()->default_value(defaultLogDir),
       "logging directory")
      ("debug.log.stdout", po::value<bool>()->default_value(false),
       "allow llog to log to the terminal/screen (standard output)")
      ("debug.dump,D", po::value<std::string>()->default_value(""),
      "Dump blackboard in .bbd2 format. Empty std::string disables.")
      ("debug.mask", po::value<int>()->default_value(INITIAL_MASK),
      "Blackboard mask determining what is serialised")
      ("debug.top.jpeg", po::value<int>()->default_value(-1),
      "JPEG quality for raw image serialisation.  -1 to use raw image")
      ("debug.bot.jpeg", po::value<int>()->default_value(-1),
      "JPEG quality for raw image serialisation.  -1 to use raw image")
      ;

   po::options_description thread_config("Thread/Module Execution options");
   thread_config.add_options()
      ("thread.gamecontroller,G", po::value<bool>()->default_value(true),
       "enable GameController thread")
      ("thread.motion,M", po::value<bool>()->default_value(true),
       "enable Motion thread")
      ("thread.offnaotransmitter,O", po::value<bool>()->default_value(true),
       "enable OffNaoTransmitter thread")
      ("thread.perception,P", po::value<bool>()->default_value(true),
       "enable Perception thread")
      ("thread.vision,V", po::value<bool>()->default_value(true),
       "enable Vision module")
      ("thread.behaviour,B", po::value<bool>()->default_value(true),
       "enable Behaviour module")
      ("thread.naotransmitter,C", po::value<bool>()->default_value(true),
       "enable Nao Transmitter module")
      ("thread.naoreceiver,R", po::value<bool>()->default_value(true),
       "enable Nao Receiver module")
      ("thread.shutdowntime", po::value<int>()->default_value(0),
       "shutdown after arg seconds")
      ;

   po::options_description stateestimation_config("State Estimation options");
   stateestimation_config.add_options()
      ("stateestimation.handle_referee_mistakes", po::value<bool>()->default_value(true),
      "set whether to consider the possiblity of referees accidentally penalising / unpenalising robots")
      ("stateestimation.initial_pose_type", po::value<std::string>()->default_value(std::string("GAME")),
      "Type of initial pose (GAME / UNPENALISED / SPECIFIED)")
      ("stateestimation.specified_initial_x", po::value<int>()->default_value(0),
      "initial x value (if stateestimation.initial_pose_type == SPECIFIED)")
      ("stateestimation.specified_initial_y", po::value<int>()->default_value(0),
      "initial y value (if stateestimation.initial_pose_type == SPECIFIED)")
      ("stateestimation.specified_initial_theta", po::value<int>()->default_value(0),
      "initial theta value (if stateestimation.initial_pose_type == SPECIFIED)")
      ;

   po::options_description behaviour_config("Behaviour options");
   behaviour_config.add_options()
      ("behaviour.path", po::value<std::string>()->default_value(std::string("/home/nao/data/behaviours/")),
       "path containing python behaviours.")
      ("behaviour.skill,s", po::value<std::string>()->default_value(std::string("Game")),
       "The desired top level Python skill class.")
      ("behaviour.use_getups", po::value<bool>()->default_value(true),
       "Getups on/off (TRUE, FALSE)")
      // TODO: Are these necessary?
      ("behaviour.headskill,k", po::value<std::string>()->default_value("HeadCentre"),
       "The desired top level Python headskill class.")
      ("behaviour.positioning", po::value<std::string>()->default_value("PositioningAgainstKickingTeam"),
       "The desired positioning file to use, can be changed against different teams")
      // ("behaviour.remote_stiffen", po::value<bool>()->default_value(false),
      //  "Remotely stiffen (and standup) when a READY packet is received from gamecontroller.")
      ;

   po::options_description motion_config("Motion options");
   motion_config.add_options()
      // TODO: On import, check how much of this is required now
      ("motion.effector,e", po::value<std::string>()->default_value(std::string("Agent")),
       "effector to be used by motion")
      ("motion.touch,t", po::value<std::string>()->default_value(std::string("Agent")),
       "touch to be used by motion")
      ("motion.path", po::value<std::string>()->default_value(std::string("/home/nao/data/pos/")),
       "the path of .pos files for ActionGenerator")
      ("motion.individualConfigPath", po::value<std::string>()->default_value(std::string("/home/nao/data/pos/individualPoses")),
       "the path of .pos files for ActionGenerator, for individual robots")
      ("walk.f", po::value<float>()->default_value(0.5),
       "frequency of coronal plane rocking (Hz)")
      ("walk.st", po::value<float>()->default_value(1.0),
       "stiffness of leg joints (0.0 - 1.0)")
      ("walk.cs", po::value<bool>()->default_value(true),
       "coronal stabilisation on/off")
      ("walk.r", po::value<float>()->default_value(20.0),
       "amplitude of coronal plane rocking (degrees)")
      ("walk.s", po::value<float>()->default_value(5.0),
       "spread of legs when standing (degrees)")
      ("walk.l", po::value<float>()->default_value(20.0),
       "amplitude of leg lift (degrees)")
      ("walk.fs", po::value<float>()->default_value(5.0),
       "amplitude of forward step (degrees)")
      ("walk.ls", po::value<float>()->default_value(0.0),
       "amplitude of left step (degrees)")
      ("walk.ts", po::value<float>()->default_value(0.0),
       "amplitude of turn step (degrees)")
      ("walk.b", po::value<float>()->default_value(15.0),
       "bend of legs when standing upright (degrees)")
      ("walk.liftHeight", po::value<float>()->default_value(8.0f),
       "leg lift height.")
      ("walk.coronalAmplitude", po::value<float>()->default_value(0.0f),
       "Coronal rock amplitude.")
      ("walk.liftFrac", po::value<float>()->default_value(0.5f),
       "fraction of cycle in which leg lift is performed.")
      ("walk.moveFrac", po::value<float>()->default_value(0.4f),
       "fraction of cycle in which leg move is performed.")
      ("walk.m", po::value<float>()->default_value(0.0),
       "leg lift frequency multiplier (must be multiple of two)")
      ("motion.getup_speed", po::value<std::string>()->default_value(std::string("SLOW")),
      "Speed of getups (FAST, MODERATE, SLOW)")
      ("motion.KpGyro", po::value<float>()->default_value(0.07f),
       "Walk Gyro PD Controller proportional multiplier)")
      ("motion.KdGyro", po::value<float>()->default_value(0.0003934f),
       "Walk Gyro PD Controller derivative multiplier")
      ("motion.KpAngle", po::value<float>()->default_value(0.2f),
       "Walk Angle PID Controller proportional multiplier")
      ("motion.KiAngle", po::value<float>()->default_value(0.05f),
       "Walk Angle PID Controller integral multiplier")
      ("motion.KdAngle", po::value<float>()->default_value(0.008f),
       "Walk Angle PID Controller derivative multiplier")
      ("kick.lean", po::value<float>()->default_value(20.5f),
       "Lean of kick  (not robot specific) (degrees)")
      ("kick.leanOffsetL", po::value<float>()->default_value(0.0f),
       "Lean offset when kicking with left (degrees)")
      ("kick.leanOffsetR", po::value<float>()->default_value(0.0f),
       "Lean offset when kicking with right (degrees)")
      ("kick.stableNumFrames", po::value<int>()->default_value(3),
       "Number of motion frames to consider stable before kicking")
      ("kick.stableAngleX", po::value<float>()->default_value(5.0f),
       "AngleX to consider stable before kicking (degrees)")
      ("kick.extraStableAngleX", po::value<float>()->default_value(4.0f),
       "AngleX to consider stable before kicking (degrees)")
      ("kick.extraStableGyroscopeX", po::value<float>()->default_value(5.0f),
       "GyroscopeX to consider extra stable before kicking (degrees)")
      ("kick.kickGyroscopeXOntoSupportFootThresh", po::value<float>()->default_value(5.0f),
       "Maximum GyroscopeX value when rocking towards the support foot, to consider stable before kicking (degrees)")
      ("kick.kickGyroscopeXAwayFromSupportFootThresh", po::value<float>()->default_value(30.0f),
       "Maximum GyroscopeX value when rocking away from the support foot, to consider stable before kicking (degrees)")
      ("kick.foot", po::value<std::string>()->default_value(std::string("LEFT")),
       "Kicking foot when calibrating (LEFT, RIGHT)")
      // TODO: Are these necessary?
      // ("motion.v4", po::value<bool>()->default_value(false),
      //  "whether to use v3 version of getup instead")
      ;

   po::options_description vision_config("Vision options");
   vision_config.add_options()
      // TODO: On import, check how much of this is required now
      ("vision.camera,c", po::value<std::string>()->default_value(std::string("Nao")),
       "camera to be used by vision")
      ("vision.camera_controls", po::value<std::string>()->default_value(std::string("")),
       "comma separated list of cid:value pairs of controls (cid offset from V4L2_CID_BASE)")
      ("vision.dumpframes,d", po::value<bool>()->default_value(false),
       "dump frames to disk")
      ("vision.dumprate,r", po::value<int>()->default_value(1000),
       "dump frames every arg milliseconds")
      ("vision.dumpfile,f", po::value<std::string>()->default_value(std::string("dump.yuv")),
       "file to store frames in")
      ("vision.top.adaptivethresholdingwindow", po::value<int>()->default_value(101),
       "base top camera adaptive thresholding window size")
      ("vision.top.adaptivethresholdingpercent", po::value<int>()->default_value(-40),
       "base top camera adaptive thresholding percent")
      ("vision.bot.adaptivethresholdingwindow", po::value<int>()->default_value(101),
       "base bottom camera adaptive thresholding window size")
      ("vision.bot.adaptivethresholdingpercent", po::value<int>()->default_value(-40),
       "base bottom camera adaptive thresholding percent")
      ("vision.top.ball.circlefitadaptivethresholdingwindowportion", po::value<float>()->default_value(0.4),
       "the portion of a ball region to use as window size for circle fit adaptive thresholding in top camera regions")
      ("vision.top.ball.circlefitadaptivethresholdingpercent", po::value<int>()->default_value(-5),
       "base top camera circle fit adaptive thresholding percent")
      ("vision.bot.ball.circlefitadaptivethresholdingwindowportion", po::value<float>()->default_value(0.4),
       "the portion of a ball region to use as window size for circle fit adaptive thresholding in bottom camera regions")
      ("vision.bot.ball.circlefitadaptivethresholdingpercent", po::value<int>()->default_value(-5),
       "base bottom camera circle fit adaptive thresholding percent")
      ("vision.top.ball.internalregionadaptivethresholdingwindowportion", po::value<float>()->default_value(0.3),
       "the portion of a ball region to use as window size for internal region adaptive thresholding in top camera regions")
      ("vision.top.ball.internalregionadaptivethresholdingpercentbrightnessportion", po::value<float>()->default_value(0.1),
       "the portion of region brightness to use as the precent for internal region adaptive thresholding in top camera regions")
      ("vision.bot.ball.internalregionadaptivethresholdingwindowportion", po::value<float>()->default_value(0.5),
       "the portion of a ball region to use as window size for internal region adaptive thresholding in bottom camera regions")
      ("vision.bot.ball.internalregionadaptivethresholdingpercent", po::value<int>()->default_value(20),
       "base bottom camera internal region adaptive thresholding percent")
      ("vision.top.ball.blobroiadaptivethresholdingwindowportion", po::value<float>()->default_value(0.5),
       "the portion of a ball region to use as window size for blob roi adaptive thresholding in top camera regions")
      ("vision.top.ball.blobroiadaptivethresholdingpercentbrightnessportion", po::value<float>()->default_value(0.1),
       "the portion of region brightness to use as the precent for blob roi adaptive thresholding in top camera regions")
      ("vision.bot.ball.blobroiadaptivethresholdingwindowportion", po::value<float>()->default_value(0.6),
       "the portion of a ball region to use as window size for blob roi adaptive thresholding in bottom camera regions")
      ("vision.bot.ball.blobroiadaptivethresholdingpercent", po::value<int>()->default_value(20),
       "base bottom camera blob roi adaptive thresholding percent")
      ;

   po::options_description camera_config("Camera options");
   camera_config.add_options()
      ("camera.top.hflip", po::value<int>()->default_value(1),
       "camera top hflip")
      ("camera.top.vflip", po::value<int>()->default_value(1),
       "camera top vflip")
      ("camera.top.brightness", po::value<int>()->default_value(20),
       "camera top brightness")
      ("camera.top.contrast", po::value<int>()->default_value(60),
       "camera top contrast")
      ("camera.top.saturation", po::value<int>()->default_value(210),
       "camera top saturation")
      ("camera.top.hue", po::value<int>()->default_value(0),
       "camera top hue")
      ("camera.top.sharpness", po::value<int>()->default_value(2),
       "camera top sharpness")
      ("camera.top.backlightcompensation", po::value<int>()->default_value(0x00),
       "camera top backlight compensation")
      ("camera.top.exposure", po::value<int>()->default_value(90),
       "camera top exposure")
      ("camera.top.gain", po::value<int>()->default_value(250),
       "camera top gain")
      ("camera.top.whitebalance", po::value<int>()->default_value(2700),
       "camera top whitebalance")
      ("camera.top.exposureauto", po::value<int>()->default_value(1),
       "camera top exposureauto")
      ("camera.top.autowhitebalance", po::value<int>()->default_value(1),
       "camera top autowhitebalance")
      ("camera.top.autofocus", po::value<int>()->default_value(1),
       "camera top autofocus")
      ("camera.top.focusabsolute", po::value<int>()->default_value(0),
       "camera top focusabsolute")
      ("camera.top.exposurealgorithm", po::value<int>()->default_value(3),
       "camera top exposurealgorithm")
      ("camera.top.aetargetavgluma", po::value<int>()->default_value(55),
       "camera top ae target avg luma")
      ("camera.top.aetargetavglumadark", po::value<int>()->default_value(27),
       "camera top ae target avg luma")
      ("camera.top.aetargetgain", po::value<int>()->default_value(128),
       "camera top ae target gain")
      ("camera.top.aeminvirtgain", po::value<int>()->default_value(32),
       "camera top ae min virt gain")
      ("camera.top.aemaxvirtgain", po::value<int>()->default_value(256),
       "camera top ae max virt gain")
      ("camera.top.aeminvirtagain", po::value<int>()->default_value(32),
       "camera top ae min virt again")
      ("camera.top.aemaxvirtagain", po::value<int>()->default_value(256),
       "camera top ae max virt again")
      ("camera.top.aetargetexposure", po::value<int>()->default_value(10),
       "camera top ae target exposure for V6 camera")
      ("camera.top.aeuseweighttable", po::value<bool>()->default_value(true),
       "camera top ae use weight table for V6 camera")
      ("camera.top.aeweighttablex1", po::value<float>()->default_value(0.f),
       "camera top weight table left (0.0 - 1.0")
      ("camera.top.aeweighttablex2", po::value<float>()->default_value(1.f),
       "camera top weight table right (0.0 - 1.0")
      ("camera.top.aeweighttabley1", po::value<float>()->default_value(0.f),
       "camera top weight table top (0.0 - 1.0")
      ("camera.top.aeweighttabley2", po::value<float>()->default_value(1.f),
       "camera top weight table bottom (0.0 - 1.0")

      ("camera.bot.hflip", po::value<int>()->default_value(0),
       "camera bot hflip")
      ("camera.bot.vflip", po::value<int>()->default_value(0),
       "camera bot vflip")
      ("camera.bot.brightness", po::value<int>()->default_value(20),
       "camera bot brightness")
      ("camera.bot.contrast", po::value<int>()->default_value(60),
       "camera bot contrast")
      ("camera.bot.saturation", po::value<int>()->default_value(180),
       "camera bot saturation")
      ("camera.bot.hue", po::value<int>()->default_value(0),
       "camera bot hue")
      ("camera.bot.sharpness", po::value<int>()->default_value(2),
       "camera bot sharpness")
      ("camera.bot.backlightcompensation", po::value<int>()->default_value(0x00),
       "camera bot backlight compensation")
      ("camera.bot.exposure", po::value<int>()->default_value(13),
       "camera bot exposure")
      ("camera.bot.gain", po::value<int>()->default_value(250),
       "camera bot gain")
      ("camera.bot.whitebalance", po::value<int>()->default_value(2700),
       "camera bot whitebalance")
      ("camera.bot.exposureauto", po::value<int>()->default_value(1),
       "camera bot exposureauto")
      ("camera.bot.autowhitebalance", po::value<int>()->default_value(1),
       "camera bot autowhitebalance")
      ("camera.bot.autofocus", po::value<int>()->default_value(1),
       "camera bot autofocus")
      ("camera.bot.focusabsolute", po::value<int>()->default_value(0),
       "camera bot focusabsolute")
      ("camera.bot.exposurealgorithm", po::value<int>()->default_value(3),
       "camera bot exposurealgorithm")
      ("camera.bot.aetargetavgluma", po::value<int>()->default_value(55),
       "camera bot ae target avg luma")
      ("camera.bot.aetargetavglumadark", po::value<int>()->default_value(27),
       "camera bot ae target avg luma dark")
      ("camera.bot.aetargetgain", po::value<int>()->default_value(128),
       "camera bot ae target gain")
      ("camera.bot.aeminvirtgain", po::value<int>()->default_value(32),
       "camera bot ae min virt gain")
      ("camera.bot.aemaxvirtgain", po::value<int>()->default_value(256),
       "camera bot ae max virt gain")
      ("camera.bot.aeminvirtagain", po::value<int>()->default_value(32),
       "camera bot ae min virt again")
      ("camera.bot.aemaxvirtagain", po::value<int>()->default_value(256),
       "camera bot ae max virt again")
      ("camera.bot.aetargetexposure", po::value<int>()->default_value(10),
       "camera bot ae target exposure for V6 camera")
      ("camera.bot.aeuseweighttable", po::value<bool>()->default_value(false),
       "camera bot ae use weight table for V6 camera")
      ("camera.bot.aeweighttablex1", po::value<float>()->default_value(0.f),
       "camera bot weight table left (0.0 - 1.0")
      ("camera.bot.aeweighttablex2", po::value<float>()->default_value(1.f),
       "camera bot weight table right (0.0 - 1.0")
      ("camera.bot.aeweighttabley1", po::value<float>()->default_value(0.f),
       "camera bot weight table top (0.0 - 1.0")
      ("camera.bot.aeweighttabley2", po::value<float>()->default_value(1.f),
       "camera bot weight table bottom (0.0 - 1.0")
      ;


   po::options_description kinematics_config("Kinematics options");
   kinematics_config.add_options()
      ("kinematics.bodyPitch", po::value<float>()->default_value(0.0),
       "accounts for imperfections in all motors in the robots body.")
      ("kinematics.cameraYawTop", po::value<float>()->default_value(0.0),
       "difference between real camera Yaw compared to aldebaran specs")
      ("kinematics.cameraRollTop", po::value<float>()->default_value(0.0),
       "difference between real camera Roll compared to aldebaran specs")
      ("kinematics.cameraRollBottom", po::value<float>()->default_value(0.0),
       "difference between real camera Roll compared to aldebaran specs")
      ("kinematics.cameraPitchTop", po::value<float>()->default_value(0.0),
       "difference between real camera Pitch compared to aldebaran specs")
      ("kinematics.cameraYawBottom", po::value<float>()->default_value(0.0),
       "difference between real camera Yaw compared to aldebaran specs")
      ("kinematics.cameraPitchBottom", po::value<float>()->default_value(0.0),
       "difference between real camera Pitch compared to aldebaran specs")
      ;

   po::options_description touch_config("Touch options");
   touch_config.add_options()
      ("touch.gyroscopeXOffset", po::value<float>()->default_value(0.0),
       "offset on gyroscopeX.")
      ("touch.gyroscopeYOffset", po::value<float>()->default_value(0.0),
       "offset on gyroscopeY.")
      ("touch.angleXOffset", po::value<float>()->default_value(0.0),
       "offset on angleX")
      ("touch.angleYOffset", po::value<float>()->default_value(0.0),
       "offset on angleY")
      ("touch.isSonarLeftWorking", po::value<bool>()->default_value(true),
       "so we don't try to avoid phantom objects")
      ("touch.isSonarRightWorking", po::value<bool>()->default_value(true),
       "so we don't try to avoid phantom objects")
      ;

   po::options_description gamecontroller_config("GameController options");
   gamecontroller_config.add_options()
      ("gamecontroller.connect", po::value<bool>()->default_value(true),
       "whether the GameController should try to connect")
      ("gamecontroller.state", po::value<std::string>()->default_value(std::string("INITIAL")),
       "game state if gamecontroller not connected, can be: INITIAL, READY, SET, PLAYING, FINISHED")
      ("gamecontroller.gamephase", po::value<std::string>()->default_value(std::string("NORMAL")),
       "secondary game phase if gamecontroller not connected, can be: NORMAL, PENALTYSHOOT")
      ("gamecontroller.setplay", po::value<std::string>()->default_value(std::string("NONE")),
       "set play if gamecontroller not connected, can be: "
       "NONE, GOAL_FREE_KICK, PUSHING_FREE_KICK, CORNER_KICK, KICK_IN")
      ("gamecontroller.ourcolour", po::value<std::string>()->default_value(std::string("yellow")),
       "our team colour if gamecontroller not connected")
      ("gamecontroller.opponentteam", po::value<int>()->default_value(1),
       "opponent team number if gamecontroller not connected")
      ("gamecontroller.opponentcolour", po::value<std::string>()->default_value(std::string("blue")),
       "opponent team colour if gamecontroller not connected")
      ("gamecontroller.ourscore", po::value<int>()->default_value(0),
       "our team's score if gamecontroller not connected")
      ("gamecontroller.opponentscore", po::value<int>()->default_value(0),
       "opponent team's score if gamecontroller not connected")
      ("gamecontroller.firsthalf", po::value<bool>()->default_value(true),
       "whether we're in the first half if gamecontroller not connected")
      ("gamecontroller.kickingteam", po::value<int>()->default_value(18),
       "which team kicks off if gamecontroller not connected")
      ("gamecontroller.secsremaining", po::value<int>()->default_value(600),
       "seconds left in the half if gamecontroller not connected")
       ("gamecontroller.secondaryTime", po::value<int>()->default_value(45),
       "secondary timer used for ball free")
      ;
      ;

   po::options_description network_config("Networking options");
   network_config.add_options()
      ("network.ssid", po::value<std::string>()->default_value(std::string("redbackbots")),
       "WiFi network name (SSID) to connect to")
      ("network.transmitter_address", po::value<std::string>()->default_value(std::string("10.0.255.255")),
       "address to broadcast to")
      ("network.transmitter_base_port", po::value<int>()->default_value(10000),
       "port to which we add team number, and then broadcast on")
      ;

   po::options_description calibration_config("Calibration options");
   calibration_config.add_options()
      ("calibration.camera", po::value<std::string>()->default_value(std::string("")),
       "Enable camera V6 calibration. 'terminal' is the only possible value right now")
      ("calibration.imu", po::value<bool>()->default_value(false),
       "Enable imu calibration procedure")
      ("calibration.kinematicsauto", po::value<bool>()->default_value(false),
       "Enable kinematics calibration Procedure")
      ("calibration.kick", po::value<bool>()->default_value(false),
       "Enable kick calibration procedure")
      ;


   config_file_options
      .add(game_config)
      .add(player_config)
      .add(gamecontroller_config)
      .add(whistles_config)
      .add(debug_config)
      .add(thread_config)
      .add(stateestimation_config)
      .add(behaviour_config)
      .add(motion_config)
      .add(vision_config)
      .add(camera_config)
      .add(kinematics_config)
      .add(network_config)
      .add(touch_config)
      .add(calibration_config);
}

po::options_description store_and_notify(int argc, char **argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   std::vector<std::string> vs(argv + 1, argv + argc);
   // for(int arg = 1; arg < argc; ++arg)
   //    vs.push_back(argv[arg]);
   return store_and_notify(vs, vm, generic);
}


po::options_description store_and_notify(std::vector<std::string> argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   po::options_description config_file_options;
   populate_options(config_file_options);

   po::options_description cmdline_options;
   if (generic) {
      cmdline_options.add(*generic);
   }
   cmdline_options.add(config_file_options);

   /** Config hierarchy:
    *  - 1. command line arguments
    *  - 2. /home/nao/config/Robots/$hostname/robot.cfg
    *  - 3. /home/nao/config/Robots/$head/head.cfg (for head/camera specific configs, may use a different head to hostname)
    *  - 4. /home/nao/config/Robots/$body/body.cfg (for body/kinematics specific configs, may use a different body to hostname)
    *  - 5. /home/nao/config/redbackbots.cfg
    *  - 6. `pwd`/redbackbots.cfg
    **/
   // arguments from this call
   store(po::command_line_parser(argv).options(cmdline_options).run(), vm);

   // arguments from previous calls
   static slist<std::vector<std::string> > argvs;
   for (slist<std::vector<std::string> >::const_iterator argv_ci = argvs.begin();
        argv_ci != argvs.end(); ++argv_ci)
      store(po::command_line_parser(*argv_ci).options(cmdline_options).run(),
            vm);

   std::ifstream ifs;
   std::string hostname;
   std::string bodyname;
   std::string headname;
   std::string rbb(RBB_NAME);

   ifs.open("/etc/hostname");
   ifs >> hostname;
   ifs.close();

   std::cout << "Loading config files for: " << hostname << std::endl;
   
   std::string configFile = std::string("/home/nao/config/Robots/" + hostname + "/robot.cfg");
   std::cout << "- Robot config: " << configFile << std::endl;
   ifs.open(configFile.c_str());
   if (ifs.good()) {
      store(parse_config_file(ifs, config_file_options), vm);
   } else {
      std::cout << "  NOT FOUND" << std::endl;
   }
   ifs.close();

   bodyname = vm["player.bodyName"].as<std::string>();
   configFile = std::string("/home/nao/config/Robots/" + bodyname + "/body.cfg");
   ifs.open(configFile.c_str());
   std::cout << "- Robot Body config: " << configFile << std::endl;
   if (ifs.good()) {
      store(parse_config_file(ifs, config_file_options), vm);
   } else {
      std::cout << "  NOT FOUND" << std::endl;
   }
   ifs.close();

   headname = vm["player.headName"].as<std::string>();
   configFile = std::string("/home/nao/config/Robots/" + headname + "/head.cfg");
   ifs.open(configFile.c_str());
   std::cout << "- Robot Head config: " << configFile << std::endl;
   if (ifs.good()) {
      store(parse_config_file(ifs, config_file_options), vm);
   } else {
      std::cout << "  NOT FOUND" << std::endl;
   }
   ifs.close();

   configFile = std::string("/home/nao/config/" + rbb + ".cfg");
   std::cout << "- Base config: " << configFile << std::endl;
   ifs.open(configFile.c_str());
   if (ifs.good()) {
      store(parse_config_file(ifs, config_file_options), vm);
   } else {
      std::cout << "  NOT FOUND" << std::endl;
   }
   ifs.close();

   configFile = std::string(rbb + ".cfg");
   std::cout << "- `pwd` config: " << configFile << std::endl;
   ifs.open(std::string(configFile).c_str());
   if (ifs.good()) {
      store(parse_config_file(ifs, config_file_options), vm);
   } else {
      std::cout << "  NOT FOUND" << std::endl;
   }
   ifs.close();


   /** Doesn't do anything right now, but will 'notify' any variables
    * we try to set via program_options */
   po::notify(vm);

   // assuming all options were valid and no exception was thrown, save argv for
   // repeated use
   argvs.push_front(argv);

   return cmdline_options;
}

/** A little struct used to help output program options */
struct type_info_compare {
   bool operator()(const std::type_info* lhs,
                   const std::type_info* rhs) const {
      return lhs->before(*rhs);
   }
};

/** Wrapper for outputting a boost::program_options::varaible_value */
template <typename T>
struct output_value {
   void operator()(const po::variable_value& v) const {
      std::cout << v.as< T >() << std::endl;
   }
};

void options_print(boost::program_options::variables_map &vm) {
   /** Populate this map with actions that will correctly
    * print each type of program option */
   std::map<const std::type_info*,
       boost::function<void(const po::variable_value&)>,
       type_info_compare> action_map;
   action_map[&typeid(std::string)] = output_value<std::string>();
   action_map[&typeid(int)] = output_value<int>();
   action_map[&typeid(bool)] = output_value<bool>();
   action_map[&typeid(float)] = output_value<float>();

   po::variables_map::iterator it;
   std::cout << std::endl;
   for (it = vm.begin(); it != vm.end(); ++it) {
      // cout << setw(20) << left << it->first << ": ";
      const po::variable_value& v = it->second;
      if (!v.empty()) {
         std::cout << std::setw(20) << std::left << it->first << ": ";
         action_map[&v.value().type()](v);
      }
   }
}
