#include "communication/transmitter/TeamTransmitter.hpp"

#include <iostream>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/BehaviourBlackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/incapacitated.hpp"

#define IGNORE_RESERVE_PACKETS 0    // Set to 1 to ignore reserve packets (for testing only)

using namespace boost::asio;
using namespace std;

TeamTransmitter::TeamTransmitter(Blackboard *bb)
   : Adapter(bb),
      NaoTransmitter((bb->config)["network.transmitter_base_port"].as<int>() +
                        (bb->config)["player.team"].as<int>(),
                     (bb->config)["network.transmitter_address"].as<string>()),
      service(), socket(service, ip::udp::v4()),
      eventTransmitterBB(*(bb->eventTransmitter)) {

llog(INFO) << "TeamTransmitter constructor called." << std::endl;

// Check if blackboard is valid
if (bb == nullptr) {
   llog(ERROR) << "Blackboard pointer is null in TeamTransmitter constructor." << std::endl;
   throw std::runtime_error("Blackboard pointer is null.");
}
}

void TeamTransmitter::tick() {
llog(DEBUG) << "tick() started." << std::endl;

// Check if blackboard and modules are valid
if (blackboard == nullptr) {
   llog(ERROR) << "Blackboard pointer is null in tick()." << std::endl;
   return;
}
if (!blackboard->stateEstimation || !blackboard->gameController) {
   llog(ERROR) << "Required Blackboard modules are not available in tick()." << std::endl;
   return;
}

// Only send to team if we have at least "RESERVE_PACKETS" messages left
if ((readFrom(gameController, our_team).messageBudget > RESERVE_PACKETS) || IGNORE_RESERVE_PACKETS) {
   llog(DEBUG) << "Message budget is sufficient." << std::endl;

   bool shouldSend;
   {
      int messageBudget = readFrom(gameController, our_team).messageBudget;
      int packetsSent = PACKET_LIMIT - messageBudget;
      int secsRemaining = readFrom(gameController, data).secsRemaining;
      bool firstHalf = static_cast<bool>(readFrom(gameController, data).firstHalf);
      float packetRate = calculatePacketRate(packetsSent, secsRemaining, firstHalf);
      float projectedPacketUsage = calculateProjectedPacketUsage(packetRate);
      shouldSend = eventTransmitterBB.shouldSendPacket(projectedPacketUsage);
   }

   if (shouldSend) {
      llog(DEBUG) << "shouldSendPacket() returned true." << std::endl;
      sendToTeam();
   } else {
      llog(DEBUG) << "shouldSendPacket() returned false." << std::endl;
   }
} else {
   llog(INFO) << "Insufficient message budget." << std::endl;
}
sendToGameController(); // Send to GC every tick
llog(DEBUG) << "tick() completed." << std::endl;
}

TeamTransmitter::~TeamTransmitter() {
llog(DEBUG) << "TeamTransmitter destructor called." << std::endl;
socket.close();
}

void TeamTransmitter::sendToTeam() {
llog(DEBUG) << "sendToTeam() started." << std::endl;

// Check if blackboard is valid
if (blackboard == nullptr) {
   llog(ERROR) << "Blackboard pointer is null in sendToTeam()." << std::endl;
   return;
}

// Use the events selected by the blackboard to be sent in the packet
// (subset of all events, if all raised events don't fit in a packet)
auto selectedIndices = eventTransmitterBB.selectEventsForPacket();

// Assemble the filtered array of events
auto filtered = eventTransmitterBB.getFilteredEvents(selectedIndices);

EventMessageHeader header = {
      static_cast<uint8_t>((blackboard->config)["player.number"].as<int>()),
      computeEventHash(filtered)
};

// Serialize only selected events
auto serialized_buffer = EventSerializer::serializeEvents(header, filtered);
llog(DEBUG) << "Serialized buffer created." << std::endl;

llog(INFO) << "Packet size to transmit: " << serialized_buffer.size()
            << " bytes." << std::endl;

// Send the buffer
NaoTransmitter::tick(
      boost::asio::buffer(serialized_buffer.data(), serialized_buffer.size()));

llog(INFO) << "Successfully transmitted packet of size "
            << serialized_buffer.size() << " bytes." << std::endl;

// Clear only dispatched events
eventTransmitterBB.clearEvents(selectedIndices);

llog(DEBUG) << "sendToTeam() completed." << std::endl;
}

void TeamTransmitter::sendToGameController() {
llog(DEBUG) << "sendToGameController() started." << std::endl;

// Check if blackboard is valid
if (blackboard == nullptr) {
   llog(ERROR) << "Blackboard pointer is null in sendToGameController()." << std::endl;
   return;
}

char *sendToIP = readFrom(gameController, lastGameControllerIPAddress);
if (sendToIP == NULL) {
   llog(INFO) << "sendToIP is NULL. Exiting sendToGameController()." << std::endl;
   return;
}

gameControllerEndpoint.address(ip::address::from_string(sendToIP));
gameControllerEndpoint.port(GAMECONTROLLER_RETURN_PORT);
boost::system::error_code ec = boost::system::error_code();
socket.connect(gameControllerEndpoint, ec);
if (ec) {
   llog(ERROR) << "Failed to connect to GameController." << std::endl;
   return;
}
llog(DEBUG) << "Connected to GameController." << std::endl;

RoboCupGameControlReturnData d = RoboCupGameControlReturnData();
d.teamNum = (blackboard->config)["player.team"].as<int>();
d.playerNum = (blackboard->config)["player.number"].as<int>();
d.fallen = 0;
AbsCoord robot_pos = readFrom(stateEstimation, robotPos);
d.pose[0] = robot_pos.x();
d.pose[1] = robot_pos.y();
d.pose[2] = robot_pos.theta();
RRCoord ball_rel_pos = readFrom(stateEstimation, ballPosRR);
d.ball[0] = ball_rel_pos.distance();
d.ball[1] = ball_rel_pos.heading() * 1000;
d.ballAge = readFrom(stateEstimation, ballAge);

llog(DEBUG) << "Prepared RoboCupGameControlReturnData." << std::endl;

// TODO (Peter): If GameController PC goes away, we should get some kind of
// catchable error here so we can stop sending packets until another
// GC comes online
size_t bytesSent = socket.send(
      boost::asio::buffer(&d, sizeof(RoboCupGameControlReturnData)), 0, ec);
if (ec) {
   llog(ERROR) << "Failed to send data to GameController." << std::endl;
} else {
   llog(DEBUG) << "Data sent to GameController." << std::endl;
}
llog(DEBUG) << "sendToGameController() completed." << std::endl;
}

EVENT_HASH_TYPE TeamTransmitter::computeEventHash(std::array<EventDataPtr, EVENTS_ARRAY_SIZE> events) {
EVENT_HASH_TYPE eventHash = 0;
for (size_t i = 0; i < events.size(); ++i) {
   if (events[i] != nullptr) {
      llog(DEBUG) << "Event " << i << " has been raised." << std::endl;
      eventHash |= (1u << i);
   }
}
llog(DEBUG) << "Computed eventHash (in bits): " 
            << std::bitset<EVENT_HASH_BIT_SIZE>(eventHash) << std::endl;
return eventHash;
}

