#include "motion/touch/LoLATouch.hpp"

#include <bitset>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <map>
#include <msgpack.hpp>
// not sure why i can't include these instead
//#include <msgpack/v1/object.hpp>
//#include <msgpack/v2/object_fwd.hpp>

#include "motion/LoLAData.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/defs/FieldDefinitions.hpp"

#define MAX_CLICK_INTERVAL 15
// number of seconds it takes to sit.  can have decimal point.
#define KILL_SLEEP_SECONDS "4"
// number of cycles of consistent lost-stiffness before we declare it really lost
#define LOST_STIFFNESS_CYCLES 83

using namespace LoLAData;

LoLATouch::LoLATouch(int team, int player_number) {
   llog(INFO) << "LoLATouch constructed" << std::endl;
}

LoLATouch::~LoLATouch() {
   llog(INFO) << "LoLATouch destroyed" << std::endl;
}

static void load(const std::vector<msgpack::object> &vector, float *array) {
   transform(vector.begin(), vector.end(), array, bind(&msgpack::object::as<float>, std::placeholders::_1));
}


// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
/**
 * @param charge from 0 to 1
 */
static void doBattery(float charge, int status) {
   // Start complaining if battery < 30%
   // initialisation of static variables only happens once
   static float old_charge = charge;
   // if charge decreasing & <= 30%
   if (old_charge > charge && charge <= 0.3f) {
      SAY("battery " + boost::lexical_cast<std::string>(charge * 100) + " percent");
   }
   old_charge = charge;

   std::bitset<32>        b                  = status;
   static std::bitset<32> old_battery_status = b;
   // discharging: 11000110111111111010100000000000
   // charging:    11000110111111101110100000000000
   if (old_battery_status[14] && !b[14]) {
      llog(INFO) << "Discharging" << std::endl;
   } else if (charge > 0.99f) {
      llog(INFO) << "Fully Charged" << std::endl;
   } else if (!old_battery_status[14] && b[14]) {
      llog(INFO) << "Charging" << std::endl;
   }
   old_battery_status = b;
}

// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
static void doTemps(JointValues joints) {
   static int t = 0;
   if (t % 100 == 0 &&
       joints.temperatures[t / 100] > 75 &&
       !limp && !standing &&
       joints.stiffnesses[t / 100] > 0) {
      SAY("OVERHEATING: " + Joints::fliteJointNames[t / 100]);
   }
   t = (t + 1) % (Joints::NUMBER_OF_JOINTS * 100);
}

template<typename T, T def>
struct HeadButtons {
        T rear=def, middle=def, front=def;
};
// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
static inline ButtonPresses doButtons(bool chest, HeadButtons<bool, false> head, bool left, bool right, bool falling) {
   static int    chest_up = 0, chest_presses = 0, chest_down = 0;
   static HeadButtons<int, 0> head_press_time;
   static std::function<void()> kill= []() {
       static bool currently_shutting_down = false;
       if(currently_shutting_down)
           return;
       currently_shutting_down = true;
       SAY("killed red back bots");
       // kill -9 me in case i don't die
       system("/usr/bin/pkill redbackbots & (sleep " KILL_SLEEP_SECONDS "; /usr/bin/pkill -9 redbackbots) &");
   };
   ButtonPresses buttons;

    // deal with button presses
   if (chest_up > MAX_CLICK_INTERVAL && chest_presses) {
      buttons.push(chest_presses);
      chest_presses = 0;
      if (sitUnstiff){
         if (buttons.pop(1)){
            sitUnstiff = false;
            standing = true;
            limp = false;
         }
      }
      if (buttons.pop(2)) {
         if (left || right) { // if foot bumper
            // falling sometimes triggers button presses
            if (!falling) {
               head_limp = !head_limp;
               SAY(std::string("head ") + (head_limp ? "limp" : "stiff"));
            }
         } else {
            if (limp) {
               limp     = false;
               standing = true;
            } else {
               // fallling sometimes triggers button presses
               if (!falling) {
                  limp     = true;
                  standing = false;
               }
            }
            SAY(std::string("body ") + (limp ? "limp" : "stiff"));
         }
      } else if (buttons.pop(3)) {
         if (left || right) {
            SAY("Restarting now key");  // yay transliteration
            // Runlevel a is set up to run nao restart on demand.
            // See man pages for inittab, init for details
            system("sudo /sbin/init a");
            // should not get here
         } else {
            // falling sometimes triggers button presses
            if(!falling) {
                kill();
            }
         }
      } else if (buttons.pop(4)) {
         if(!falling) {
            SAY("Restart wifi");
            // TODO: ADD BACK CHANGE FIELD IF NECESSARY
            //system("sudo /home/nao/bin/changeField.py");
            // changeField.py already does this, but libagent did this so we do it
            system("sudo /etc/init.d/rbbwireless.sh restart");
         }
      }
   }

   // special shutdown handler
   // we set chest_down to int_min so only one shutdown will happen
   if (chest_down > 300) {  // 3 seconds
      SAY("Shutting down");
      system("sudo /sbin/halt");
      limp       = true;
      chest_down = std::numeric_limits<int>::min();
   }

   // update counters
   if (chest) {
      if (chest_down >= 0) {
         chest_down++;
      }
      chest_up = 0;
   } else {
      chest_up++;
      if (chest_down > 0) {
         chest_presses++;
         chest_down = 0;
      }
   }
   std::function<void(bool, int&)> handle_head_press = [&](bool pressed, int &counter) {
      if (pressed) {
        counter = MAX_CLICK_INTERVAL * 3;
      } else if(counter>0) {
        counter--;
      }
   };
    handle_head_press(head.rear, head_press_time.rear);
    handle_head_press(head.middle, head_press_time.middle);
    handle_head_press(head.front, head_press_time.front);

    if (// if the buttons have been pressed recently
            head_press_time.front&&head_press_time.middle&&head_press_time.rear
            // and we're not falling
            && !falling) {
        int head_press_total=head_press_time.rear+head_press_time.middle+head_press_time.front;
        head_press_time.front=0;
        head_press_time.middle=0;
        head_press_time.rear=0;
        // if the time total is too large, the buttons were pressed too quickly. Therefore, ignore
        // we only do this in competition (convenient to touch entire head for testing)
        if(head_press_total>MAX_CLICK_INTERVAL*3*3-3) {
            sitUnstiff = true;
            return buttons;
        }
        std::cout << "Received kill command from head. Shutting down" << std::endl;
        kill();
    }
   return buttons;
}

void LoLATouch::loadCache() {
   const std::map<std::string, std::vector<msgpack::object> > sensorMap = read();
   if (sensorMap.size() == 0) {
      return;
   }

   // Cout the whole map
   // for(map<string, vector<msgpack::object>::iterator it = sensorMap.begin(); it != sensorMap.end(); ++it)
   // {
   //    std::cout << it->first << " " << it->second << "\n";
   // }

   load(sensorMap.at("Position"), sensors.joints.angles);
   load(sensorMap.at("Stiffness"), sensors.joints.stiffnesses);
   load(sensorMap.at("Temperature"), sensors.joints.temperatures);
   load(sensorMap.at("Current"), sensors.joints.currents);

   // = {} is redudunant as static arrays are zero-initialised anyway
   static uint8_t lostStiffnessCounters[Joints::NUMBER_OF_JOINTS] = {};
   for (int       jointIndex                                      = 0;
        jointIndex < Joints::NUMBER_OF_JOINTS;
        ++jointIndex) {

      // LWristYaw and RWristYaw can throw false positives joint issues, but ignore them
      if (jointIndex != Joints::LWristYaw &&
          jointIndex != Joints::RWristYaw &&
          sensors.joints.currents[jointIndex] == 0.f &&
          sensors.joints.stiffnesses[jointIndex] > 0.f &&
         // TODO: TW Check what this targetAngles is actually used for....
         // In rare circumstances targetAngles is not yet instantiated - so check length
         ( (int) LoLAData::targetAngles.size() > jointIndex &&
           abs(sensors.joints.angles[jointIndex] - LoLAData::targetAngles[jointIndex]) > DEG2RAD(2))
         ) {
         ++lostStiffnessCounters[jointIndex];
         if (lostStiffnessCounters[jointIndex] > LOST_STIFFNESS_CYCLES) {
            SAY("cannot move " + Joints::fliteJointNames[jointIndex]);
         }
      } else {
         lostStiffnessCounters[jointIndex] = 0;
      }
   }

   load(sensorMap.at("Battery"), sensors.sensors + Sensors::SensorCodesEnum::Battery_Charge);
   load(sensorMap.at("Accelerometer"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_AccelerometerX);
   load(sensorMap.at("Gyroscope"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_GyroscopeX);
   load(sensorMap.at("Angles"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_AngleX);
   load(sensorMap.at("Sonar"), sensors.sensors + Sensors::SensorCodesEnum::SonarLeft);
   load(sensorMap.at("FSR"), sensors.sensors + Sensors::LFoot_FSR_FrontLeft);
   load(sensorMap.at("Touch"), sensors.sensors + Sensors::ChestBoard_Button);

   // Cout all sensors
  /*  for (unsigned i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
    {
       std::cout << Sensors::sensorNames[i] << ": " << sensors.sensors[i] << std::endl;
    }*/

   float *status = &sensors.sensors[Sensors::Battery_Status];
   doBattery(sensors.sensors[Sensors::Battery_Charge],
         // treat the float like an int, and pray it works
             *(int *) status);
   doTemps(sensors.joints);
}

SensorValues LoLATouch::getSensors(Kinematics &kinematics) {
   return sensors;
}

bool LoLATouch::getSitUnstiff() {
   return sitUnstiff;
}

bool LoLATouch::getStanding() {
   return standing;
}

ButtonPresses LoLATouch::getButtons() {

   bool falling = (sensors.sensors[Sensors::InertialSensor_AngleY] > 0.60) ||
                  (sensors.sensors[Sensors::InertialSensor_AngleY] < -0.60);
    HeadButtons<bool, false> headButtons={.rear=static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Rear]),
                             .middle=static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Middle]),
                             .front=static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Front])
    };
   return doButtons(static_cast<bool>(sensors.sensors[Sensors::ChestBoard_Button]),headButtons,
                    static_cast<bool>(sensors.sensors[Sensors::LFoot_Bumper_Left]) ||
                    static_cast<bool>(sensors.sensors[Sensors::LFoot_Bumper_Right]),
                    static_cast<bool>(sensors.sensors[Sensors::RFoot_Bumper_Left]) ||
                    static_cast<bool>(sensors.sensors[Sensors::RFoot_Bumper_Right]),
                    falling);
}
