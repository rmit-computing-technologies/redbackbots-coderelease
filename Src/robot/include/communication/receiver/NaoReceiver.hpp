#pragma once

#include <boost/system/error_code.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/thread.hpp>

#include "utils/Logger.hpp"

class NaoReceiver {
   protected:
      /**
       * Constructor.  Opens a socket for listening.
       */
      template <class SubClass, typename ReadHandler>
      NaoReceiver(SubClass *scthis, ReadHandler handler, int port);

      /**
       * Destructor. Closes the socket.
       */
      virtual ~NaoReceiver();

      template <class SubClass, typename ReadHandler>
      void startReceive(SubClass *scthis, ReadHandler handler);
      char recvBuffer[1500]; // 1500 is the generally accepted maximum transmission unit on ethernet

   private:
      boost::asio::io_service service;
      boost::asio::ip::udp::socket socket;
      boost::asio::ip::udp::endpoint remoteEndpoint;
      int port;
      boost::thread *t;
};

template <class SubClass, typename ReadHandler>
NaoReceiver::NaoReceiver(SubClass *scthis, ReadHandler handler, int port)
   : service(), socket(service, boost::asio::ip::udp::v4()), remoteEndpoint(boost::asio::ip::address_v4::broadcast(), port) {
   NaoReceiver::port = port;

   llog(INFO) << "Nao Receiver constructed" << std::endl;
   // Allows multiple processes to listen on the same port
   // Not required - but left here in case such behaviour is desirabled
   //boost::asio::socket_base::reuse_address option(true);
   //socket.set_option(option);  

   socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));

   startReceive(scthis, handler);
   t = new boost::thread(boost::bind(&boost::asio::io_service::run, &service));
   llog(INFO) << "Listening for data on port " << port << std::endl;
}

template <class SubClass, typename ReadHandler>
void NaoReceiver::startReceive(SubClass *scthis, ReadHandler handler) {
   NaoReceiver::remoteEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), NaoReceiver::port);
   socket.async_receive_from(
      boost::asio::buffer((boost::asio::mutable_buffer((void *)&recvBuffer, sizeof(recvBuffer)))), 
      NaoReceiver::remoteEndpoint,   
      boost::bind(handler, scthis,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

