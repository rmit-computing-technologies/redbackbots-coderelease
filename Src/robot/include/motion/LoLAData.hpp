
// TODO: CHECK MSGPACK VERSION v1/2/3 declaration
//       WORKS ON THE NAO -> v2 has compile error
// #include <msgpack/v2/object_fwd_decl.hpp>
#include <msgpack/object_fwd_decl.hpp>

#include <map>
#include <string>
#include <vector>

#include "types/SensorValues.hpp"

namespace LoLAData {
   // some are floats, some are strings, so we have to use a generic type like msgpack::object
   const std::map<std::string, std::vector<msgpack::object> > read();

   template<typename T>
   void write(const std::map<std::string, std::vector<T>> &);

   // shared between touch and effector
   // touch tells the effector when to be limp/stiff
   extern bool               head_limp;
   // shared between touch and effector
   // touch tells the effector when to be limp/stiff
   extern bool               limp;
   // shared between touch and effector
   // used by motion and motion expects this to be true for only one cycle and then false thereafter
   // TODO: use the falling edge of limp instead, so it's all handled in motion
   extern bool               standing;
   // Testing for unstiff state
   extern bool               sitUnstiff;
   // shared between touch and effector
   // used at the start of sit so we know what joint angles/stiffness to start at
   extern SensorValues       sensors;
   // shared between touch and effector
   // used to detect lost stiffness
   extern std::vector<float> targetAngles;
}
