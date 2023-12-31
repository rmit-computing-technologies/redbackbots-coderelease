#include <boost/python.hpp>

#include <gamecontroller/RoboCupGameControlData.hpp>

#include <types/JointValues.hpp>

#include "perception/behaviour/python/converters/array_conv.hpp"
#include "perception/behaviour/python/converters/EigenM33f_conv.hpp"
#include "perception/behaviour/python/converters/Point_conv.hpp"
#include "perception/behaviour/python/converters/VariablesMap_conv.hpp"

#include "types/SPLStandardMessage.hpp"
#include "types/BroadcastData.hpp"

#include "utils/SPLDefs.hpp"

/* Used for registering c arrays of fixed size */
#define REG_ARRAY_TYPE(type, size)                                              \
   boost::python::to_python_converter<type[size],array_to_python<type,size> >();\
   array_to_python<type, size>::ready()

void register_python_converters()
{

   /* Sensor and Joint Values */
   REG_ARRAY_TYPE(float, Sensors::NUMBER_OF_SENSORS);
   REG_ARRAY_TYPE(float, Joints::NUMBER_OF_JOINTS);

   /* Game Controller Data */
   REG_ARRAY_TYPE(RobotInfo, MAX_NUM_PLAYERS);
   REG_ARRAY_TYPE(TeamInfo  ,               2);

   /* Receiver Blackboard Data */
   REG_ARRAY_TYPE(bool, ROBOTS_PER_TEAM);
   REG_ARRAY_TYPE(SPLStandardMessage, ROBOTS_PER_TEAM);
   REG_ARRAY_TYPE(BroadcastData, ROBOTS_PER_TEAM);
   REG_ARRAY_TYPE(time_t, ROBOTS_PER_TEAM);

   REG_ARRAY_TYPE(float, 3);
   REG_ARRAY_TYPE(float, 2);

   boost::python::to_python_converter<
      Point,
      Point_to_python>();

   boost::python::to_python_converter<
      PointF,
      PointF_to_python>();

   boost::python::to_python_converter<
      Eigen::Matrix<float, 3, 3>,
      EigenM33f_to_python>();
   // for reading the config
    boost::python::to_python_converter<
            boost::program_options::variables_map,
            VariablesMap_to_python>();
}

