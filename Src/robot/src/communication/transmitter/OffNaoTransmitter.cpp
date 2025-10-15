#include "communication/transmitter/OffNaoTransmitter.hpp"

#include <bitset>
#include <boost/algorithm/string/classification.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/ThreadBlackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "types/serialise/CameraSettingsSerialse.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"

using boost::asio::ip::tcp;
using namespace boost;
using namespace boost::algorithm;
namespace po = boost::program_options;
 

OffNaoTransmitter::OffNaoTransmitter(Blackboard *bb)
    : Adapter(bb), port_(10125) {
   start_accept();
}

void OffNaoTransmitter::start_accept() {
   try {
      tcp::endpoint endpoint(tcp::v4(), port_);
      acceptor_ = new tcp::acceptor(io_service_, endpoint);
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));

      acceptor_->async_accept(new_session->socket(),
                              boost::bind(&OffNaoTransmitter::handle_accept,
                                          this, new_session,
                                          boost::asio::placeholders::error));
      std::cout << "Listening for Offnao on port " << port_ << ".\n";
      // in case say.py hangs around, don't let it keep the port open
      fcntl(acceptor_->native_handle(), F_SETFD, FD_CLOEXEC);
   } catch (const std::exception& e) {
      std::cerr << "Offnao exception: " << e.what() << "\n";
   }
}

void OffNaoTransmitter::tick() {
   llog(TRACE) << "ticking away" << std::endl;
   io_service_.poll();
   room_.deliver(blackboard);
   io_service_.poll();
}

OffNaoTransmitter::~OffNaoTransmitter() {
   delete acceptor_;
}

void OffNaoTransmitter::handle_accept(offnao_session_ptr session,
                                      const boost::system::error_code& error) {
   if (!error) {
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));
         acceptor_->async_accept(new_session->socket(),
                              boost::bind(&OffNaoTransmitter::handle_accept,
                                          this, new_session,
                                          boost::asio::placeholders::error));
      session->start(blackboard);
      session->socket().set_option(tcp::no_delay(true));
      session->socket().non_blocking(false);
      // boost::asio::socket_base::non_blocking_io command(false);
      // boost::asio::ip::tcp::acceptor::non_blocking_io command(false);
      // session->socket().io_control(command);
      llog(DEBUG) << "Created wireless link." << std::endl;
   } else {
      llog(ERROR) << error.message() << std::endl;
      start_accept();
   }
}

OffNaoTransmitter::offnao_session::
offnao_session(boost::asio::io_service* io_service, offnao_room *room)
   : connection_(io_service), room_(*room), sendingMask(INITIAL_MASK) {}

tcp::socket& OffNaoTransmitter::offnao_session::socket() {
   return connection_.socket();
}

void OffNaoTransmitter::offnao_session::start(Blackboard *blackboard) {
   room_.join(shared_from_this());
   boost::asio::async_read(connection_.socket(),
                           // TODO(jayen): change this fixed size buffer to
                           // something more flexible
                           boost::asio::buffer(&receivedMask, sizeof(receivedMask)),
                           boost::bind(&offnao_session::handle_read,
                                       shared_from_this(),
                                       boost::asio::placeholders::error, blackboard));
}

void OffNaoTransmitter::offnao_session::deliver(Blackboard *blackboard) {
   // mask set before delivery to attempt to allow multiple clients to stream different things
   // llog(DEBUG) << "sending mask: " << std::bitset<sizeof(sendingMask) * 8>(sendingMask) << std::endl;
   blackboard->write(&(blackboard->mask), sendingMask);

   // Set send type through protobuf
   boost::system::error_code e;
   if (sendingMask & USE_BATCHED_MASK) {
        e = connection_.batch_write((ProtobufSerialisable&)*blackboard);
   } else {
        e = connection_.sync_write((ProtobufSerialisable&)*blackboard);
   }
   if (e) {
      llog(ERROR) << "Failed to write: " << e << std::endl;
   }
}

void OffNaoTransmitter::offnao_room::join(offnao_participant_ptr participant) {
   participants_.insert(participant);
}

void OffNaoTransmitter::offnao_room::
leave(offnao_participant_ptr participant) {
   participants_.erase(participant);
}

void OffNaoTransmitter::offnao_room::deliver(Blackboard *blackboard) {
   for_each(participants_.begin(), participants_.end(),
            boost::bind(&offnao_participant::deliver, _1, boost::ref(blackboard)));
}

void OffNaoTransmitter::offnao_session::
handle_write(boost::system::error_code const& error) {
   if (error) {
      room_.leave(shared_from_this());
   }
}

void OffNaoTransmitter::offnao_session::
handle_read(boost::system::error_code const& error, Blackboard *blackboard) {
   if (!error) {
      // llog(DEBUG) << std::bitset<sizeof(receivedMask) * 8>(receivedMask) << " - Received Mask " << std::endl;
      // llog(DEBUG) << std::bitset<sizeof(uint64_t) * 8>(TO_NAO_MASKS) << " - TO_NAO_MASKS" << std::endl;
      // llog(DEBUG) << std::bitset<sizeof(uint64_t) * 8>(STRING_MASK) << " - STRING_MASK" << std::endl;
      // llog(DEBUG) << std::bitset<sizeof(uint64_t) * 8>(CAMERA_SETTINGS_MASK) << " - CAMERA_SETTINGS_MASK" << std::endl;
      // llog(DEBUG) << (receivedMask & TO_NAO_MASKS) << std::endl;
      // llog(DEBUG) << (receivedMask & STRING_MASK) << std::endl;
      // llog(DEBUG) << (receivedMask & CAMERA_SETTINGS_MASK) << std::endl;
      if (receivedMask & TO_NAO_MASKS) {
         if (receivedMask & STRING_MASK) {
            llog(INFO) << "Received string mask" << std::endl;
            std::string command;
            connection_.sync_read(command);
            llog(INFO) << "\tstring: " << command << std::endl;
         } else if (receivedMask & CAMERA_SETTINGS_MASK) {
            llog(INFO) << "Received new camera settings" << std::endl;
            CameraSettingsSerialse csSerialise;
            connection_.sync_read((ProtobufSerialisable&) csSerialise);
            llog(INFO) << "\t for camera: " << CameraInfo::enumCameraToString(csSerialise.whichCamera) << std::endl;

            if (csSerialise.whichCamera == CameraInfo::Camera::top) {
               writeTo(vision, topCameraSettings, csSerialise.settings);
            } else {
               writeTo(vision, botCameraSettings, csSerialise.settings);
            }
         }
      } else {
         // Offnao mask has updated - therefore change the sending mask to match the new elements to send to Offnao
         sendingMask = receivedMask;
      }

      // TW: For reasons that are not quite clear to me yet (to investigate) 
      //     The connection needs to be reset/re-established after every call.
      //     Perhaps the sync version is a once-off connection
      boost::asio::async_read(connection_.socket(),
                              boost::asio::buffer(&receivedMask,
                                                  sizeof(receivedMask)),
                              boost::bind(&offnao_session::handle_read,
                                          shared_from_this(),
                                          boost::asio::placeholders::error, blackboard));
   } else {
      llog(DEBUG) << "Received data with error!!!" << std::endl;
      room_.leave(shared_from_this());
   }
}
