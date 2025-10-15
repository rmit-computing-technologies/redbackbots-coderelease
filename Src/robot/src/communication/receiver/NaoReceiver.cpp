#include "communication/receiver/NaoReceiver.hpp"

NaoReceiver::~NaoReceiver() {
   service.stop();
   t->join();
   delete t;
   if (socket.is_open()) {
      socket.close();
   }
}
