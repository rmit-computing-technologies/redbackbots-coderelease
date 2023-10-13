#include "motion/MotionOdometry.hpp"

#include "motion/MotionDefs.hpp"
#include "utils/body.hpp"

// Amount to multiply gyroscopez by to get actual results
#define TURN_MULTIPLIER 1.05f

using namespace std;

MotionOdometry::MotionOdometry() {
    reset();
}

Odometry MotionOdometry::updateOdometry(const SensorValues& sensors, Odometry walkChange) {
    float gyroZ = sensors.sensors[Sensors::InertialSensor_GyroscopeZ];
    // convert to radians per frame, same direction as walkChange
    gyroZ *= - MOTION_DT * TURN_MULTIPLIER;
    walkChange.turn = gyroZ;
    return walkChange;
}

void MotionOdometry::reset() {
}
