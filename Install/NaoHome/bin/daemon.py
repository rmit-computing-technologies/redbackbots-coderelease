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
import json

import msgpack
import configparser

import socket
import random

SAY_HOST = "localhost"
SAY_PORT = 65432

PERIOD = 0.4
MAX_CLICK_INTERVAL = 15

NUM_PRESSES_TO_SELECT_SKILL = 3

APLAY_PATH = "/usr/bin/aplay"

CONFIG_FILE = "/home/nao/config/redbackbots.cfg"


def floatBitsUInt32(f):
    return struct.unpack(">L", struct.pack(">f", f))[0]


# for say
PYTHONPATH = "/opt/aldebaran/lib/python2.7/site-packages/"
os.environ["PYTHONPATH"] = PYTHONPATH


def say(message):
    """
    E-speak for voice
    """

    payload = {
        "message": message,
        "level": 'CRITICAL'
    }

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            client_socket.connect((SAY_HOST, SAY_PORT))
            client_socket.sendall(json.dumps(payload).encode("utf-8"))
            response = client_socket.recv(1024)
        except socket.error as error:
            print(f"Failed to connect to say socket: {error}")


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
            "Right"
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
            "RHand/Touch/Right"
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
        self.Skull = [
            "Front/Left/1",
            "Front/Left/0",
            "Middle/Left/0",
            "Rear/Left/0",
            "Rear/Left/1",
            "Rear/Left/2",
            "Rear/Right/2",
            "Rear/Right/1",
            "Rear/Right/0",
            "Middle/Right/0",
            "Front/Right/0",
            "Front/Right/1"
        ]
        self.actuators = {
            "Position": self.joints,
            "Stiffness": self.joints,
            "Chest": ["Red", "Green", "Blue"],
            "Sonar": self.sonars,
            "LEar": self.LEar,
            "REar": self.REar,
            "Skull": self.Skull
        }
        self.commands = {
            # 'Position': [0.0] * 25,
            # 'Stiffness': [0.0] * 25,
            "Chest": [0.0] * 3,
            # 'Sonar': [True, True],
            "LEar": [0.0] * 10,
            "REar": [0.0] * 10,
            # 8 eye segments, each with 3 values (RGB)
            # This will make the eyes a solid white
            "LEye": [1.0] * 24,
            "REye": [1.0] * 24,
            "Skull": [1.0] * 12
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

    def skull(self, brightness):
        for key, value in brightness.items():
            self.command("Skull", key, value)

    def flash(self, period, red, green, blue):
        """
        :type period: float in seconds
        :type red: float 0-1
        :type green: float 0-1
        :type blue: float 0-1
        """
        brightness = (1 - math.cos(2 * math.pi * time.time() / period)) / 2
        self.chest(brightness * red, brightness * green, brightness * blue)

    def cycle_skull(self, period, selected_skill, skill_sets, use_skill_menu):
        brightness = (1 - math.cos(2 * math.pi * time.time() / period)) / 2
        head_segements = []

        if selected_skill == skill_sets["skill_set_1"]:
            head_segements = [key for key in self.Skull if "Front" in key]
        elif selected_skill == skill_sets["skill_set_2"]:
            head_segements = [key for key in self.Skull if "Middle" in key]
        elif selected_skill == skill_sets["skill_set_3"]:
            head_segements = [key for key in self.Skull if "Rear" in key]

        if use_skill_menu:
            skull_brightness = {
                key: brightness if key in head_segements else 0.0 for key in self.Skull
            }
        else:
            skull_brightness = {
                key: brightness for key in self.Skull
            }

        self.skull(skull_brightness)

    def leftEar(self, charge, status):
        charging = status & 0x00004000
        # from 0 to 1
        linear_progress = math.fmod(time.time(), PERIOD) / PERIOD
        smooth_progress = (1 - math.cos(math.pi * linear_progress)) / 2
        num_leds_on = int(round(charge * 10))
        num_leds_off = 10 - num_leds_on
        self.commands["LEar"] = [1.0] * num_leds_on + [0.0] * num_leds_off

    # totally ok if this crashes because systemd will restart us
    # we tried try/except with a retry but it hung
    def open(self):
        # os.system("/usr/bin/pkill --signal 9 say.py")
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.connect("/tmp/robocup")

    def close(self):
        self.socket.close()


class Daemon:
    def __init__(self, robot):
        self.robot = robot

        # For the skill menu
        self.use_skill_menu = None

        # A dictionary of [<bodyskill>, <headskill>]
        self.skill_sets = {}

        self.head_button_index = {
            "front": "skill_set_1",
            "middle": "skill_set_2",
            "rear": "skill_set_3"
        }

        # Read in all configurations from redbackbots.cfg
        # Includes: skill menu & skill sets
        self.get_skills()

        # Selects the first skill set as the default
        self.selected_skill = self.skill_sets["skill_set_1"]

        # For checking if redbackbots.cfg has been updated (increase efficiency)
        self.initial_config_last_modified_time = os.path.getmtime(CONFIG_FILE)

        # Track if daemon/robot should be operating such as shutdown
        self.operating = True

        # Used by headswipe, tracks the amount of time since the button was pressed
        self.head_press_time = {
            "front": 0,
            "middle" : 0,
            "rear" : 0
        }

        # Used by skill menu to track the number of times a button is pressed
        self.head_presses = {
            "front": 0,
            "middle" : 0,
            "rear" : 0
        }

        self.chest_up = 0
        self.chest_down = 0
        self.chest_presses = 0

        self.charge = 1  # 100%
        self.charging = True

        self.gyro_warning_time = time.time()
        self.gyro_initialise_frames = 0
        self.initial_gyro_readings = [0, 0, 0]

    def check_for_config_update(self) -> bool:
        """
        Returns True if redbackbots.cfg has been updated
        If there has been an update then records the update time
        """
        time_config_last_modified = os.path.getmtime(CONFIG_FILE)
        has_been_updated = (
            time_config_last_modified != self.initial_config_last_modified_time
        )

        # Update the time that the config was read
        if has_been_updated:
            self.initial_config_last_modified_time = time_config_last_modified

        return has_been_updated

    def get_skills(self):
        """
        Gets the latest configs from redbackbots.cfg
        Removes the need to restart the robot after each config change
        Previously chosen skill will remain selected until changed in skill menu
        """
        config = configparser.ConfigParser()
        config.read(CONFIG_FILE)
        behaviour_section = config["behaviour"]

        self.use_skill_menu = behaviour_section.getboolean("use_skill_menu", fallback=False)

        self.skill_sets = {
            "skill_set_1": (
                behaviour_section.get("skill", fallback="Game"),
                behaviour_section.get("headskill", fallback="HeadAware")
            ),
            "skill_set_2": (
                behaviour_section.get("second_skill", fallback="Demo"),
                behaviour_section.get("second_headskill", fallback="HeadAware")
            ),
            "skill_set_3": (
                behaviour_section.get("third_skill", fallback="Stand"),
                behaviour_section.get("third_headskill", fallback="HeadCentre")
            )
        }

        if not self.use_skill_menu:
            self.selected_skill = self.skill_sets["skill_set_1"]

    def do_buttons(self, chest, left, right, headFront, headMiddle, headRear):

        if self.check_for_config_update():
            self.get_skills()
            say("Found config update")

        def handle_head_press(pressed, counter, button):
            if pressed[0]:
                counter = MAX_CLICK_INTERVAL * 3
                if pressed[0] != pressed[1]:
                    self.head_presses[button] += 1
            elif counter > 0:
                counter -= 1
            return counter

        def reset_head_button_timers():
            self.head_press_time["front"] = 0
            self.head_press_time["middle"] = 0
            self.head_press_time["rear"] = 0

        self.head_press_time["rear"] = handle_head_press(headRear, self.head_press_time["rear"], "rear")
        self.head_press_time["middle"] = handle_head_press(headMiddle, self.head_press_time["middle"], "middle")
        self.head_press_time["front"] = handle_head_press(headFront, self.head_press_time["front"], "front")
        head_swipe = False

        # If all headbuttons have been pressed recently
        if (
            self.head_press_time["front"]
            and self.head_press_time["middle"]
            and self.head_press_time["rear"]
        ):
            # Check if head swipe is front to back
            head_swipe = (
                self.head_press_time["front"] < self.head_press_time["middle"]
                and self.head_press_time["middle"] < self.head_press_time["rear"]
            )

            # Reset counters
            reset_head_button_timers()

        # If the skill menu is enabled
        elif self.use_skill_menu:
            for button, presses in self.head_presses.items():
                other_buttons = [b for b in self.head_presses if b != button]

                # Clear all head button presses if multiple buttons have been pressed
                if presses > 0 and any(
                    self.head_presses[b] != 0 for b in other_buttons
                ):
                    self.head_presses = {key: 0 for key in self.head_presses}

                    # Re-add the most recent button press
                    # Prevent having to press 4 times after accidental press
                    newest_button_press = max(
                        self.head_press_time, key=lambda k: self.head_press_time[k]
                    )
                    self.head_presses[newest_button_press] += 1

                # Change skill according to button press
                elif presses >= NUM_PRESSES_TO_SELECT_SKILL:
                    if all(self.head_presses[b] == 0 for b in other_buttons):
                        self.selected_skill = self.skill_sets[
                            self.head_button_index[button]
                        ]
                        self.head_presses = {key: 0 for key in self.head_presses}
                        say(f"selected {self.selected_skill[0]} skill")

        # deal with button presses
        if head_swipe or (self.chest_up > MAX_CLICK_INTERVAL and self.chest_presses):
            buttons = self.chest_presses
            self.chest_presses = 0
            if buttons == 2:
                stream = os.popen("hostname -I")
                say(f"My IP address is {stream.read()}")
            elif buttons == 3 or head_swipe:
                if left or right:
                    # yay transliteration
                    self.robot.chest(0, 1, 1)
                    say("Restarting now key")
                    os.system("/usr/bin/pkill -9 -f redbackbots")
                    os.system("/usr/bin/nao restart")
                else:
                    self.robot.chest(0, 1, 1)
                    say(f"loaded {self.selected_skill[0]}")
                    os.system("/usr/bin/pkill -9 -f redbackbots")
                    self.robot.close()
                    stdout = ">> /tmp/redbackbots.stdout.txt"
                    stderr = "2>> /tmp/redbackbots.stderr.txt"
                    cmd = f"/home/nao/redbackbots -s {self.selected_skill[0]} -k {self.selected_skill[1]}"
                    os.system(f"{cmd} {stdout} {stderr}")
                    print("Control returned")
                    self.robot.open()
            elif buttons == 4:
                self.robot.chest(0, 1, 1)
                say("Restart wifi")
                # os.system("/home/nao/bin/setprofile RBB")
                cmd = "/usr/bin/sudo /etc/init.d/rbbwireless.sh restart"
                os.system(cmd)
            elif buttons == 5:
                self.robot.chest(0, 1, 1)
                say("Set netplan to none")
                os.system("/home/nao/bin/setprofile NONE")
                cmd = "/usr/bin/sudo /etc/init.d/rbbwireless.sh restart"
                os.system(cmd)

        # special shutdown handler
        # we set chest_down to int_min so only one shutdown will happen
        if self.chest_down > 200:  # Approx 3 seconds
            self.robot.chest(0, 1, 1)
            print("Shutting Down")
            os.system(f"{APLAY_PATH} /opt/aldebaran/share/naoqi/wav/stop_jingle.wav")
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

    def do_battery(self, charge, status):
        # from LoLATouch.doBattery
        if charge < (self.charge - 0.005) and charge <= 0.01:
            say(f"battaree critical")
        elif charge < (self.charge - 0.01) and charge <= 0.05:
            say("Help! My battaree is too low!")
        elif charge < (self.charge - 0.05) and charge <= 0.3:
            say("battery " + str(round(charge * 100)) + " percent")
        charging = not not (status & 0x00004000)
        self.charge = charge
        self.charging = charging

        self.robot.commands["LEye"] = [0.0 if not charging else 1.0] * 24
        self.robot.commands["REye"] = [0.0 if not charging else 1.0] * 24
        if charge > 0.9 and charging:
            self.robot.commands["LEye"] = [0.0] * 8 + [1.0] * 8 + [0.0] * 8
            self.robot.commands["REye"] = [0.0] * 8 + [1.0] * 8 + [0.0] * 8
        if charge < 0.3 and charging:
            self.robot.commands["LEye"] = [1.0] * 8 + [0.0] * 8 + [0.0] * 8
            self.robot.commands["REye"] = [1.0] * 8 + [0.0] * 8 + [0.0] * 8

    def isGyroTooLarge(self, gyroX, gyroY, gyroZ):
        SMALL = 0.017
        return abs(gyroX) > SMALL or abs(gyroY) > SMALL or abs(gyroZ) > SMALL

    def checkGyro(self, gyro):
        gather_frames = 20
        if self.gyro_initialise_frames < gather_frames:
            self.initial_gyro_readings = [
                self.initial_gyro_readings[0] + gyro["InertialSensor/GyroscopeX"],
                self.initial_gyro_readings[1] + gyro["InertialSensor/GyroscopeY"],
                self.initial_gyro_readings[2] + gyro["InertialSensor/GyroscopeZ"]
            ]
            self.gyro_initialise_frames += 1

        initialGyroX, initialGyroY, initialGyroZ = self.initial_gyro_readings
        initialGyroX /= gather_frames
        initialGyroY /= gather_frames
        initialGyroZ /= gather_frames

        if self.isGyroTooLarge(initialGyroX, initialGyroY, initialGyroZ):
            currentTime = time.time()
            if currentTime - self.gyro_warning_time > 3:
                print(
                    "Error: initial gyro values too big (x,y,z)",
                    initialGyroX,
                    initialGyroY,
                    initialGyroZ
                )
                self.gyro_warning_time = time.time()

        gyroX = gyro["InertialSensor/GyroscopeX"]
        gyroY = gyro["InertialSensor/GyroscopeY"]
        gyroZ = gyro["InertialSensor/GyroscopeZ"]
        # print('gyro: ', gyroX, gyroY, gyroZ)

        # reset if they come back to normal just in case of false positives from human handling
        if not self.isGyroTooLarge(gyroX, gyroY, gyroZ):
            self.initial_gyro_readings = [0, 0, 0]


def main():
    def handler(signum, frame):
        # when starting from the chest button, oddly this
        # doesn't print immediately, only after redbackbots exits
        print("Signal handler called with signal", signum, file=sys.stderr)
        robot.close()
        time.sleep(1)  # give redbackbots a chance to connect

    signal.signal(signal.SIGUSR1, handler)

    robot = Robot()
    daemon = Daemon(robot)
    # TODO: wav not copied in createRootImage - to investigate wav locations
    os.system(f"{APLAY_PATH} /opt/aldebaran/share/naoqi/wav/start_jingle.wav")
    print("Started")
    try:
        # Pre-Initialise the head buttons
        headFront = [False, False]
        headMiddle = [False, False]
        headRear = [False, False]

        while True:
            data = robot.read()

            # If operating
            if daemon.operating:
                # LoLA data structure gives byte-literals as the keys of the dictionary
                # Thus the byte b'xyz' syntax is required when accessing the data dictionary
                touch = dict(zip(robot.touch, data[b"Touch"]))
                chest = touch["ChestBoard/Button"] >= 0.5
                left = touch["LFoot/Bumper/Left"] + touch["LFoot/Bumper/Right"]
                left = left > 1
                right = touch["RFoot/Bumper/Left"] + touch["RFoot/Bumper/Right"]
                right = right > 1
                headFront = [touch["Head/Touch/Front"] >= 0.5, headFront[0]]
                headMiddle = [touch["Head/Touch/Middle"] >= 0.5, headMiddle[0]]
                headRear = [touch["Head/Touch/Rear"] >= 0.5, headRear[0]]

                daemon.do_buttons(chest, left, right, headFront, headMiddle, headRear)
                battery = dict(zip(robot.battery, data[b"Battery"]))
                daemon.do_battery(battery["Charge"], floatBitsUInt32(battery["Status"]))

                gyro = dict(zip(robot.gyro, data[b"Gyroscope"]))
                daemon.checkGyro(gyro)

                robot.cycle_skull(
                    PERIOD * 10,
                    daemon.selected_skill,
                    daemon.skill_sets,
                    daemon.use_skill_menu,
                )
                robot.flash(PERIOD, 1, 0, 0)
            else:
                robot.flash(PERIOD, 0, 1, 1)

            robot.leftEar(battery["Charge"], floatBitsUInt32(battery["Status"]))
            robot.send()
    except KeyboardInterrupt:
        robot.chest(1, 1, 1)
        robot.send()
        print("Exit")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
