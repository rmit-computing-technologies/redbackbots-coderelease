#include "transmitter/Nao.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

using namespace boost::asio;
using namespace std;

NaoTransmitter::NaoTransmitter(int port, string address)
   : service(),
     socket(service, ip::udp::v4()),
     // broadcast_endpoint(ip::address_v4::broadcast(), port) 
     broadcast_endpoint(ip::address::from_string(address.c_str()), port)
        {
   // TW: The address was not being used so now use the transmitter_address
   // TODO: The transmitter address set in options.cpp isn't saving from redbackbots.cfg
   //        as the network block isn't being added to blackboard so defaults are used

   socket_base::broadcast option(true);
   socket.set_option(option);
   boost::system::error_code ec;
   socket.connect(broadcast_endpoint, ec);
   // llog(ERROR) << "Attempting to connect to: " << port << " at " << address << std::endl;
   // SAY(address.c_str());
   if (ec) {
      llog(ERROR) << "could not connect: " << ec.message() << std::endl;
      sleep(5);
      SAY("not on the network");
      throw ec;
   }
   llog(INFO) << "Nao Transmitter constructed" << endl;
}

NaoTransmitter::~NaoTransmitter() {
   socket.close();
}

void NaoTransmitter::tick(const boost::asio::mutable_buffers_1 &b) {
   boost::system::error_code ec = boost::system::error_code();
   socket.send(b, 0, ec);

   if (ec) {
      llog(ERROR) << "could not send: " << ec.message();
      static time_t lastsaid = 0;
      time_t now = time(NULL);
      if (now >= lastsaid + 5) {
         SAY("nao transmitter " + ec.message());
         lastsaid = now;
      }
   }
}
