#!/usr/bin/python3

# This is based on the daemon process from rUNSWift

# This script listens to the chest button and starts
# redbackbots on triple-press and restarts wifi on quad-press

# Why not listen for TouchChanged events?
# because ALTouch isn't loaded

# Why not listen for CaressFtoR or CaressRtoF events?
# because it didn't work

# Why not use ALMemory?
# Because none of the devices were there

# Why not run redbackbots in a loop and let it deal with all the button presses?
# Because it's annoying if we want to run redbackbots from the command line

from __future__ import print_function
import math
import os
import signal
import socket
import struct
import sys
import time

import msgpack

PERIOD = .4
MAX_CLICK_INTERVAL = 15


def floatBitsUInt32(f):
    return struct.unpack('>L', struct.pack('>f', f))[0]


# for say
PYTHONPATH = '/opt/aldebaran/lib/python2.7/site-packages/'
os.environ["PYTHONPATH"] = PYTHONPATH


def say(s):
    # os.system('echo "%s" | python /home/nao/bin/say.py' % s)
    os.system(f'/bin/espeak "{s}"')
    print(s)


class Robot:
    def __init__(self):
        self.open()

        self.battery = [
            "Charge",
            "Status",
            "Current",
            "Temperature",
        ]
        self.joints = [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LHand",
            "RHand"
        ]
        self.sonars = [
            "Left",
            "Right",
        ]
        self.touch = [
            "ChestBoard/Button",
            "Head/Touch/Front",
            "Head/Touch/Middle",
            "Head/Touch/Rear",
            "LFoot/Bumper/Left",
            "LFoot/Bumper/Right",
            "LHand/Touch/Back",
            "LHand/Touch/Left",
            "LHand/Touch/Right",
            "RFoot/Bumper/Left",
            "RFoot/Bumper/Right",
            "RHand/Touch/Back",
            "RHand/Touch/Left",
            "RHand/Touch/Right",
        ]
        self.gyro = [
            "InertialSensor/GyroscopeX",
            "InertialSensor/GyroscopeY",
            "InertialSensor/GyroscopeZ"
        ]
        self.LEar = [
            "0",
            "36",
            "72",
            "108",
            "144",
            "180",
            "216",
            "252",
            "288",
            "324"
        ]
        self.REar = [
            "324",
            "288",
            "252",
            "216",
            "180",
            "144",
            "108",
            "72",
            "36",
            "0"
        ]
        self.actuators = {
            'Position': self.joints,
            'Stiffness': self.joints,
            'Chest': ['Red', 'Green', 'Blue'],
            'Sonar': self.sonars,
            'LEar': self.LEar,
            'REar': self.REar
        }
        self.commands = {
            # 'Position': [0.0] * 25,
            # 'Stiffness': [0.0] * 25,
            'Chest': [0.0] * 3,
            # 'Sonar': [True, True],
            'LEar': [0.0] * 10,
            'REar': [0.0] * 10,
            # 8 eye segments, each with 3 values (RGB)
            # This will make the eyes a solid white
            'LEye': [1.0]*24,
            'REye': [1.0]*24,
        }

    # totally ok if this crashes because systemd will restart us
    # we tried try/except with a retry but it hung
    def read(self):
        stream = self.socket.recv(896)
        upacker = msgpack.unpackb(stream)
        return upacker

    def command(self, category, device, value):
        self.commands[category][self.actuators[category].index(device)] = value

    def send(self):
        stream = msgpack.packb(self.commands)
        self.socket.send(stream)

    def chest(self, red, green, blue):
        self.command("Chest", "Red", red)
        self.command("Chest", "Green", green)
        self.command("Chest", "Blue", blue)

    def flash(self, period, red, green, blue):
        """
        :type period: float in seconds
        :type red: float 0-1
        :type green: float 0-1
        :type blue: float 0-1
        """
        brightness = (1 - math.cos(2 * math.pi * time.time() / period)) / 2
        self.chest(brightness * red, brightness * green, brightness * blue)

    def leftEar(self, charge, status):
        charging = status & 0x00004000
        # from 0 to 1
        linear_progress = math.fmod(time.time(), PERIOD) / PERIOD
        smooth_progress = (1 - math.cos(math.pi * linear_progress)) / 2
        #if charging:
        #    charge += smooth_progress * (10 - charge)
        #else:
        #    charge *= 1 - smooth_progress
        num_leds_on = int(round(charge * 10))
        num_leds_off = 10 - num_leds_on
        self.commands["LEar"] = [1.0] * num_leds_on + [0.0] * num_leds_off

    # totally ok if this crashes because systemd will restart us
    # we tried try/except with a retry but it hung
    def open(self):
        os.system("/usr/bin/pkill --signal 9 say.py")
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.connect("/tmp/robocup")

    def close(self):
        self.socket.close()


class Daemon:
    def __init__(self, robot):
        self.robot = robot

        # Track if daemon/robot should be operating such as shutdown
        self.operating = True

        self.chest_up = 0
        self.chest_down = 0
        self.chest_presses = 0

        self.head_press_time = {
            "front": 0,
            "middle" : 0,
            "rear" : 0
        }

        self.charge = 1  # 100%
        self.charging = True

        self.gyroWarningTime = time.time()
        self.gyroInitialiseFrames = 0
        self.initialGyroReadings = [0, 0, 0]



    def doButtons(self, chest, left, right, headFront, headMiddle, headRear):

        def handle_head_press(pressed, counter):
            if pressed:
                counter = MAX_CLICK_INTERVAL * 3
            elif counter > 0:
                counter -= 1
            return counter

        self.head_press_time["rear"] = handle_head_press(headRear, self.head_press_time["rear"])
        self.head_press_time["middle"] = handle_head_press(headMiddle, self.head_press_time["middle"])
        self.head_press_time["front"] = handle_head_press(headFront, self.head_press_time["front"])
        headSwipe = False

        # If all headbuttons have been pressed recently
        if self.head_press_time["front"] and self.head_press_time["middle"] and self.head_press_time["rear"]:
            # Check if head swipe is front to back
            headSwipe = self.head_press_time["front"] < self.head_press_time["middle"] and self.head_press_time["middle"] < self.head_press_time["rear"]

            # Reset counters
            self.head_press_time["front"]=0
            self.head_press_time["middle"]=0
            self.head_press_time["rear"]=0

        # deal with button presses
        if headSwipe or (self.chest_up > MAX_CLICK_INTERVAL and self.chest_presses):
            buttons = self.chest_presses
            print("Button presses: ", buttons)
            self.chest_presses = 0
            if buttons == 3 or headSwipe:
                if left or right:
                    # yay transliteration
                    self.robot.chest(0, 1, 1)
                    say("Restarting now key")
                    os.system("/usr/bin/pkill -9 -f redbackbots")
                    os.system("/usr/bin/nao restart")
                else:
                    self.robot.chest(0, 1, 1)
                    say("loaded red back bots")
                    os.system("/usr/bin/pkill -9 -f redbackbots")
                    self.robot.close()
                    stdout = ">> /tmp/redbackbots.stdout.txt"
                    stderr = "2>> /tmp/redbackbots.stderr.txt"
                    cmd = "/home/nao/redbackbots"
                    os.system("%s %s %s" % (cmd, stdout, stderr))
                    print ("Control returned")
                    self.robot.open()
            elif buttons == 4:
                self.robot.chest(0, 1, 1)
                say("Restart wifi")
                os.system("/usr/bin/sudo /home/nao/bin/changeField.py")
                cmd = "/usr/bin/sudo /etc/init.d/rbbwireless.sh restart"
                os.system(cmd)

        # special shutdown handler
        # we set chest_down to int_min so only one shutdown will happen
        if self.chest_down > 200:  # Approx 3 seconds
            self.robot.chest(0, 1, 1)
            print("Shutting Down")
            os.system("/usr/bin/aplay /opt/aldebaran/share/naoqi/wav/stop_jingle.wav")
            os.system("/usr/bin/sudo '/usr/bin/pkill -9 -f redbackbots'")
            os.system("/usr/bin/sudo /usr/sbin/shutdown -P now")
            self.chest_down = -sys.maxsize
            self.operating = False

        # update counters (chest)
        if chest:
            if self.chest_down >= 0:
                self.chest_down += 1
            self.chest_up = 0
        else:
            self.chest_up += 1
            if self.chest_down > 0:
                self.chest_presses += 1
                self.chest_down = 0

    def doBattery(self, charge, status):
        # from LoLATouch.doBattery
        if charge < self.charge and charge <= 0.3:
            say("battery " + str(charge * 100) + " percent")
        charging = not not (status & 0x00004000)
        self.charge = charge
        self.charging = charging

        self.robot.commands["LEye"] = [0.0 if not charging else 1.0]*24
        self.robot.commands["REye"] = [0.0 if not charging else 1.0]*24
        if charge > 0.9 and charging:
            self.robot.commands["LEye"] = [0.0] * 8 + [1.0] * 8 + [0.0] * 8
            self.robot.commands["REye"] = [0.0] * 8 + [1.0] * 8 + [0.0] * 8
        if charge < 0.3 and charging:
            self.robot.commands["LEye"] = [1.0] * 8 + [0.0] * 8 + [0.0] * 8
            self.robot.commands["REye"] = [1.0] * 8 + [0.0] * 8 + [0.0] * 8
        #if self.charging and not charging:
        #    print("Discharging")
        #elif charge > 0.99:
        #    print("Fully Charged")
        #elif not self.charging and charging:
        #    print("Charging")

    def isGyroTooLarge(self, gyroX, gyroY, gyroZ):
        SMALL = 0.017
        return abs(gyroX) > SMALL or abs(gyroY) > SMALL or abs(gyroZ) > SMALL

    def checkGyro(self, gyro):
        gather_frames = 20
        if self.gyroInitialiseFrames < gather_frames:
            self.initialGyroReadings = [
                self.initialGyroReadings[0] + gyro["InertialSensor/GyroscopeX"],
                self.initialGyroReadings[1] + gyro["InertialSensor/GyroscopeY"],
                self.initialGyroReadings[2] + gyro["InertialSensor/GyroscopeZ"]
            ]
            self.gyroInitialiseFrames += 1

        initialGyroX, initialGyroY, initialGyroZ = self.initialGyroReadings
        initialGyroX /= gather_frames
        initialGyroY /= gather_frames
        initialGyroZ /= gather_frames

        if self.isGyroTooLarge(initialGyroX, initialGyroY, initialGyroZ):
            currentTime = time.time()
            if currentTime - self.gyroWarningTime > 3:
                print("Error: initial gyro values too big (x,y,z)", initialGyroX, initialGyroY, initialGyroZ)
                self.gyroWarningTime = time.time()

        gyroX = gyro["InertialSensor/GyroscopeX"]
        gyroY = gyro["InertialSensor/GyroscopeY"]
        gyroZ = gyro["InertialSensor/GyroscopeZ"]
        #print('gyro: ', gyroX, gyroY, gyroZ)

        # reset if they come back to normal just in case of false positives from human handling
        if not self.isGyroTooLarge(gyroX, gyroY, gyroZ):
            self.initialGyroReadings = [0, 0, 0]


def main():
    def handler(signum, frame):
        # when starting from the chest button, oddly this
        # doesn't print immediately, only after redbackbots exits
        print('Signal handler called with signal', signum, file=sys.stderr)
        robot.close()
        time.sleep(1)  # give redbackbots a chance to connect

    signal.signal(signal.SIGUSR1, handler)

    robot = Robot()
    daemon = Daemon(robot)
    # TODO: wav not copied in createRootImage - to investigate wav locations
    os.system("/usr/bin/aplay /opt/aldebaran/share/naoqi/wav/start_jingle.wav")
    print("Started")
    try:
        while True:
            data = robot.read()

            # If operating
            if daemon.operating:
                # LoLA data structure gives byte-literals as the keys of the dictionary
                # Thus the byte b'xyz' syntax is required when accessing the data dictionary
                touch = dict(zip(robot.touch, data[b'Touch']))
                chest = touch['ChestBoard/Button'] >= 0.5
                left = touch['LFoot/Bumper/Left'] + touch['LFoot/Bumper/Right']
                left = left > 1
                right = touch['RFoot/Bumper/Left'] + touch['RFoot/Bumper/Right']
                right = right > 1
                headFront = touch['Head/Touch/Front'] >= 0.5
                headMiddle = touch['Head/Touch/Middle'] >= 0.5
                headRear = touch['Head/Touch/Rear'] >= 0.5

                daemon.doButtons(chest, left, right, headFront, headMiddle, headRear)

                battery = dict(zip(robot.battery, data[b'Battery']))
                daemon.doBattery(battery['Charge'],
                                 floatBitsUInt32(battery['Status']))

                gyro = dict(zip(robot.gyro, data[b'Gyroscope']))
                daemon.checkGyro(gyro)

                robot.flash(PERIOD, 1, 0, 0)
            else:
                robot.flash(PERIOD, 0, 1, 1)

            robot.leftEar(battery['Charge'],
                          floatBitsUInt32(battery['Status']))
            #print("Battery Status: ", battery['Status'])
            robot.send()
    except KeyboardInterrupt:
        robot.chest(1,1,1)
        robot.send()
        print("Exit")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
