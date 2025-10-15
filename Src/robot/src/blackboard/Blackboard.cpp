#include "blackboard/Blackboard.hpp"

#include <boost/make_shared.hpp>

#include "thread/Thread.hpp"

#include <boost/bind.hpp>
#include <limits>

#include "utils/defs/RobotDefinitions.hpp"
#include "utils/defs/StateEstimationDefinitions.hpp"
#include "utils/math/angles.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"

Blackboard::Blackboard() {
   // Create the inner blackboard classes
   createInnerBlackboards();

   boost::program_options::variables_map vm;
   std::vector<std::string> argv;
   store_and_notify(argv, vm, NULL);
   config = boost::program_options::variables_map(vm);

   mask = INITIAL_MASK;
   readOptions(vm);
   thread->configCallbacks["Blackboard"] = boost::bind(&Blackboard::readOptions, this, _1);
}

// TODO: Make the constructors depend on each other rather than duplicate
Blackboard::Blackboard(const boost::program_options::variables_map &vm) :
      config(vm),
      mask(INITIAL_MASK)
{
   // Create inner classes
   createInnerBlackboards();
   createInnerDebuggerBlackboards();
   
   readOptions(vm);
   thread->configCallbacks["Blackboard"] = boost::bind(&Blackboard::readOptions, this, _1);
   thread->configCallbacks["Logger"] = &Logger::readOptions;
}

Blackboard::~Blackboard() {
   thread->configCallbacks["Blackboard"] =
      boost::function<void(const boost::program_options::variables_map &)>();

   // Clean-up shared pointers
   kinematics = nullptr;
   behaviour = nullptr;
   stateEstimation = nullptr;
   vision = nullptr;
   perception = nullptr;
   whistle = nullptr;
   gameController = nullptr;
   motion = nullptr;
   receiver = nullptr;
   eventTransmitter = nullptr;
   eventReceiver = nullptr;
   thread = nullptr;
   locks = nullptr;
   debugger = nullptr;
}

void Blackboard::createInnerBlackboards() {
   kinematics        = boost::make_shared<KinematicsBlackboard>();
   behaviour         = boost::make_shared<BehaviourBlackboard>();
   stateEstimation   = boost::make_shared<StateEstimationBlackboard>();
   vision            = boost::make_shared<VisionBlackboard>();
   perception        = boost::make_shared<PerceptionBlackboard>();
   whistle           = boost::make_shared<WhistleBlackboard>();
   gameController    = boost::make_shared<GameControllerBlackboard>();
   motion            = boost::make_shared<MotionBlackboard>();
   receiver          = boost::make_shared<ReceiverBlackboard>();
   eventTransmitter  = boost::shared_ptr<EventTransmitterBlackboard>(
      // Use a custom deleter to prevent the singleton from being deleted
      // This ensures that all modules that use the singleton will have access to the same instance
      &EventTransmitter,
      [](EventTransmitterBlackboard*){}
   );
   eventReceiver     = boost::shared_ptr<EventReceiverBlackboard>(
      // Use a custom deleter to prevent the singleton from being deleted
      // This ensures that all modules that use the singleton will have access to the same instance
      &EventReceiver,
      [](EventReceiverBlackboard*){}
   );
   thread            = boost::make_shared<ThreadBlackboard>();
   locks             = boost::make_shared<SynchronisationBlackboard>();

   // Verify that all shared pointers are initialized
   if (!kinematics || !behaviour || !stateEstimation || !vision ||
       !perception || !whistle || !gameController || !motion ||
       !receiver || !eventTransmitter || !eventReceiver || !thread || !locks) {
      llog(ERROR) << "One or more Blackboard modules failed to initialize." << std::endl;
      throw std::runtime_error("Blackboard module initialization failure.");
   }
}

void Blackboard::readOptions(const boost::program_options::variables_map& config) {
   if (!behaviour || !gameController || !receiver || !kinematics ||
       !stateEstimation || !whistle || !eventReceiver || !eventTransmitter || !vision) {
      llog(ERROR) << "Attempted to read options with uninitialized modules." << std::endl;
      return;
   }

   behaviour->readOptions(config);
   gameController->readOptions(config);
   receiver->readOptions(config);
   kinematics->readOptions(config);
   stateEstimation->readOptions(config);
   eventTransmitter->readOptions(config);
   eventReceiver->readOptions(config);
   vision->readOptions(config);
   whistle->readOptions(config);

   if (debugger != nullptr) {
      debugger->readOptions(config);
   } 
}

BehaviourBlackboard::BehaviourBlackboard() {
   readBuf = 0;
}

void BehaviourBlackboard::readOptions(const boost::program_options::variables_map& config) {
   // TODO: REMOVE UNUSED CONFIG ELEMENTS
   // TODO: remove remote stiffen
   skill = config["behaviour.skill"].as<std::string>();
   headskill = config["behaviour.headskill"].as<std::string>();
   positioning = config["behaviour.positioning"].as<std::string>();
   remoteStiffen = false;//config["behaviour.remote_stiffen"].as<bool>();
   useGetups = config["behaviour.use_getups"].as<bool>();
}

void StateEstimationBlackboard::readOptions(const boost::program_options::variables_map& config) {
   canScore = !config["game.use_passing"].as<bool>(); // If we are using passing set to false, else set to true
}

StateEstimationBlackboard::StateEstimationBlackboard() {
   robotObstacles.reserve(MAX_ROBOT_OBSTACLES);
   robotPos = AbsCoord();
   robotPosUncertainty = 0.0f;
   robotHeadingUncertainty = 0.0f;
   allRobotPos.reserve(MAX_POSE_HYPOTHESES);
   ballPosRR = RRCoord();
   ballPosRRC = AbsCoord();
   ballVelRRC = AbsCoord();
   ballVel = AbsCoord();
   ballPos = AbsCoord();
   teamBallPos = AbsCoord();
   teamBallVel = AbsCoord();
   teamBallPosUncertainty = 0.0f;
   sharedStateEstimationBundle = SharedStateEstimationBundle();

   // This flag is here so that state estimation can know when a shared state estimation bundle has
   // been sent so it can reset it.
   havePendingOutgoingSharedBundle = false;

   // one flag per teammate, include ourselves for simplicity.
   havePendingIncomingSharedBundle = std::vector<bool>(5, false);

   walkToPoint = AbsCoord();

   hasTouchedBall = false;

   haveTeamBallUpdate = false;
}

VisionBlackboard::VisionBlackboard()
{
   balls.reserve(MAX_BALLS);
   robots.reserve(MAX_ROBOTS);
   fieldFeatures.reserve(MAX_FIELD_FEATURES);
   refereeGesture.gesture = RefereeGesture::Gesture::none;
   
   regions.reserve(ESTIMATED_REGIONS);

   topFrame = NULL;
   botFrame = NULL;
}

void VisionBlackboard::readOptions(const boost::program_options::variables_map& config) {
   topFrameJPEGQuality = config["debug.top.jpeg"].as<int>();
   botFrameJPEGQuality = config["debug.bot.jpeg"].as<int>();
}

PerceptionBlackboard::PerceptionBlackboard() {
   kinematics = 0;
   stateEstimation = 0;
   vision = 0;
   behaviour = 0;
   total = 33;
}

MotionBlackboard::MotionBlackboard() {
   uptime = 0;
}

KinematicsBlackboard::KinematicsBlackboard() {
}

void KinematicsBlackboard::readOptions(const boost::program_options::variables_map& config) {
   isCalibrating = config["calibration.kinematicsauto"].as<bool>();
   if (!isCalibrating) {
      parameters.cameraYawBottom =
         config["kinematics.cameraYawBottom"].as<float>();
      parameters.cameraPitchBottom =
         config["kinematics.cameraPitchBottom"].as<float>();
      parameters.cameraRollBottom =
         config["kinematics.cameraRollBottom"].as<float>();
      parameters.cameraRollTop =
         config["kinematics.cameraRollTop"].as<float>();
      parameters.cameraYawTop = config["kinematics.cameraYawTop"].as<float>();
      parameters.cameraPitchTop =
         config["kinematics.cameraPitchTop"].as<float>();
      parameters.bodyPitch = config["kinematics.bodyPitch"].as<float>();
   }
}

GameControllerBlackboard::GameControllerBlackboard() {
   connected = false;
   active = false;
   memset(&our_team, 0, sizeof our_team);
   memset(&opponent_team, 0, sizeof opponent_team);
   memset(&data, 0, sizeof data);
   lastGameControllerIPAddress = NULL;
   gameState = 0;
   leftTeam = false;
   seenRefGesture = false;
}

WhistleBlackboard::WhistleBlackboard() {
   whistleDetectionState = WhistleDetectionState::dontKnow;
   whistleThreadCrashed = false;
}

static const int NUM_TEAM_COLOURS = 10;
static const std::string TEAM_COLOURS[NUM_TEAM_COLOURS] = {
   // same order as #define TEAM_colour in robot/gamecontroller/RoboCupGameControlData.hpp
   "blue",
   "red",
   "yellow",
   "black",
   "white",
   "green",
   "orange",
   "purple",
   "brown",
   "gray",
};

void GameControllerBlackboard::readOptions(const boost::program_options::variables_map& config) {
   connect = config["gamecontroller.connect"].as<bool>();
   player_number = config["player.number"].as<int>();
   our_team.teamNumber = static_cast<uint8_t>(config["player.team"].as<int>());
   our_team.fieldPlayerColour = static_cast<uint8_t>(find(TEAM_COLOURS,
                                 TEAM_COLOURS + NUM_TEAM_COLOURS,
                                                   config["gamecontroller.ourcolour"].as<std::string>()) -
                                              TEAM_COLOURS);
   if (our_team.fieldPlayerColour >= NUM_TEAM_COLOURS)
      llog(ERROR) << "Invalid gamecontroller.ourcolour\n";
   our_team.score = static_cast<uint8_t>(config["gamecontroller.ourscore"].as<int>());
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      our_team.players[i].penalty = PENALTY_NONE;
      our_team.players[i].secsTillUnpenalised = 0;
   }
   opponent_team.teamNumber = static_cast<uint8_t>(config["gamecontroller.opponentteam"].as<int>());
   opponent_team.fieldPlayerColour = static_cast<uint8_t>(find(TEAM_COLOURS,
                                   TEAM_COLOURS + NUM_TEAM_COLOURS,
                                                     config["gamecontroller.opponentcolour"].as<std::string>()) -
                                                TEAM_COLOURS);
   if (opponent_team.fieldPlayerColour >= NUM_TEAM_COLOURS)
      llog(ERROR) << "Invalid gamecontroller.opponentteam\n";
   opponent_team.score = static_cast<uint8_t>(config["gamecontroller.opponentscore"].as<int>());
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      opponent_team.players[i].penalty = PENALTY_NONE;
      opponent_team.players[i].secsTillUnpenalised = 0;
   }

   // Config allows testing states without running the Java GameController
   std::map<std::string, int> gcStateMap;
   gcStateMap["INITIAL"] = STATE_INITIAL;
   gcStateMap["READY"] = STATE_READY;
   gcStateMap["SET"] = STATE_SET;
   gcStateMap["PLAYING"] = STATE_PLAYING;
   gcStateMap["FINISHED"] = STATE_FINISHED;
   gcStateMap["STANDBY"] = STATE_STANDBY;
   data.state = gcStateMap[config["gamecontroller.state"].as<std::string>()];
   data.firstHalf = config["gamecontroller.firsthalf"].as<bool>();
   data.kickingTeam = static_cast<uint8_t>(config["gamecontroller.kickingteam"].as<int>());
   std::map<std::string, int> gcGamePhaseMap;
   gcGamePhaseMap["NORMAL"] = GAME_PHASE_NORMAL;
   gcGamePhaseMap["PENALTYSHOOT"] = GAME_PHASE_PENALTYSHOOT;
   gcGamePhaseMap["OVERTIME"] = GAME_PHASE_OVERTIME;
   gcGamePhaseMap["TIMEOUT"] = GAME_PHASE_TIMEOUT;
   data.gamePhase = gcGamePhaseMap[
      config["gamecontroller.gamephase"].as<std::string>()];
   data.secsRemaining = config["gamecontroller.secsremaining"].as<int>();
   data.secondaryTime = config["gamecontroller.secondaryTime"].as<int>();
   // I hate introducing hard-to-reproduce elements, but we should be testing both scenarios
   srand(time(NULL));
   if (rand() % 2) {
      data.teams[0] = our_team;
      data.teams[1] = opponent_team;
   } else {
      data.teams[0] = opponent_team;
      data.teams[1] = our_team;
   }

   std::map<std::string, int> gcSetPlayMap;
   gcSetPlayMap["NONE"] = SET_PLAY_NONE;
   gcSetPlayMap["GOAL_FREE_KICK"] = SET_PLAY_GOAL_KICK;
   gcSetPlayMap["PUSHING_FREE_KICK"] = SET_PLAY_PUSHING_FREE_KICK;
   gcSetPlayMap["CORNER_KICK"] = SET_PLAY_CORNER_KICK;
   gcSetPlayMap["KICK_IN"] = SET_PLAY_KICK_IN;
   data.setPlay = gcSetPlayMap[
      config["gamecontroller.setplay"].as<std::string>()];
}

ReceiverBlackboard::ReceiverBlackboard() {
   for (time_t& lr : lastReceived) {
      lr = 0;
   }

   for (unsigned i = 0; i < ROBOTS_PER_TEAM; ++i) {
      incapacitated.push_back(i);
   }
}

void WhistleBlackboard::readOptions(const boost::program_options::variables_map &config) {
   // No options to read yet
}

void EventTransmitterBlackboard::readOptions(const boost::program_options::variables_map &config) {
   llog(DEBUG) << "Reading configuration options" << std::endl;
   // No options to read yet
   llog(DEBUG) << "Configuration options read" << std::endl;
}

void EventReceiverBlackboard::readOptions(const boost::program_options::variables_map &config) {
   llog(DEBUG) << "Reading configuration options" << std::endl;
   // No options to read yet
   llog(DEBUG) << "Configuration options read" << std::endl;
}

void ReceiverBlackboard::readOptions(const boost::program_options::variables_map& config) {
   team = config["player.team"].as<int>();
}

ThreadBlackboard::ThreadBlackboard() {
}

SynchronisationBlackboard::SynchronisationBlackboard() {
   buttons = new boost::mutex();
   serialization = new boost::mutex();
}


//////////////////////////////////////////////////////////

// Debugger Blackboard creation section

#ifdef TMP_NDEBUG

void Blackboard::createInnerDebuggerBlackboards() {
   debugger = nullptr;
}

#else

#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"

void Blackboard::createInnerDebuggerBlackboards() {
   debugger = boost::make_shared<DebuggerBlackboard>();;
}

DebuggerBlackboard::DebuggerBlackboard() {
   vision = std::make_shared<VisionDebuggerBlackboard> ();
}

DebuggerBlackboard::~DebuggerBlackboard() {
   vision = nullptr;
}

void DebuggerBlackboard::readOptions(const boost::program_options::variables_map& config) {
}


VisionDebuggerBlackboard::VisionDebuggerBlackboard() {
}

#endif
