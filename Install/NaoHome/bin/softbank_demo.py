#!/usr/bin/env python2
# -*- coding: utf-8 -*-


# This example file is based on the demo provided by SoftBank 
# This file makes the robot stand and do nothing else

import socket
import msgpack
import time
import math

MAX_CLICK_INTERVAL = 15

def deg2rad(value) :
    return value * 0.01745329251994329575

class Robot :
    def __init__ (self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.connect( "/tmp/robocup" )
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
        self.REye = [
            "Red/0",
            "Red/45",
            "Red/90",
            "Red/135",
            "Red/180",
            "Red/225",
            "Red/270",
            "Red/315",
            "Green/0",
            "Green/45",
            "Green/90",
            "Green/135",
            "Green/180",
            "Green/225",
            "Green/270",
            "Green/315",
            "Blue/0",
            "Blue/45",
            "Blue/90",
            "Blue/135",
            "Blue/180",
            "Blue/225",
            "Blue/270",
            "Blue/315"
        ]
        self.LEye = [
            "Red/0",
            "Red/45",
            "Red/90",
            "Red/135",
            "Red/180",
            "Red/225",
            "Red/270",
            "Red/315",
            "Green/0",
            "Green/45",
            "Green/90",
            "Green/135",
            "Green/180",
            "Green/225",
            "Green/270",
            "Green/315",
            "Blue/0",
            "Blue/45",
            "Blue/90",
            "Blue/135",
            "Blue/180",
            "Blue/225",
            "Blue/270",
            "Blue/315"
        ]
        self.actuators = {
            'Position' : self.joints,
            'Stiffness' : self.joints,
            'Chest' : [ 'Red', 'Green', 'Blue' ],
            'Sonar' : self.sonars,
            'LEar' : self.LEar,
            'REar' : self.REar,
            'LEye' : self.LEye,
            'REye' : self.REye
        }
        self.commands = {
            'Position' : [ 0.0 ] * 25,
            'Stiffness' : [ 0.0 ] * 25,
            'Chest' : [ 0.0 ] * 3,
            'Sonar' : [ True, True ],
            'LEar' : [ 0.0 ] * 10,
            'REar' : [ 0.0 ] * 10,
            'LEye' : [ 0.0 ] * 24,
            'REye' : [ 0.0 ] * 24
        }
        self.chest_up = 0
        self.chest_down = 0
        self.chest_presses = 0

    def read (self):
        stream = self.socket.recv( 896 )
        upacker = msgpack.unpackb(stream)
        return upacker
    
    def command (self, category, device, value):
        self.commands[category][self.actuators[category].index(device)] = value
    
    def send (self):
        stream = msgpack.packb(self.commands)
        self.socket.send(stream)
    
    def close (self):
        self.socket.close()


    def updateChestButton(self, chestDuration):
        # update counters (chest)
        if chestDuration >= 0.5:
            if self.chest_down >= 0:
                self.chest_down += 1
            self.chest_up = 0
        else:
            self.chest_up += 1
            if self.chest_down > 0:
                self.chest_presses += 1
                self.chest_down = 0

    def setEyeColour(self, eye, r, g, b):
        for pos in [0,45,90,135,180,225,270,315]:
            numstr = str(pos)
            self.command(eye, "Red/" + numstr, r)
            self.command(eye, "Green/" + numstr, g)
            self.command(eye, "Blue/" + numstr, b)

def main ():
    robot = Robot()
    try :
        robot.command( "Position", "HeadYaw", 0.0 )
        robot.command( "Position", "HeadPitch", 0.0 )
        robot.command( "Stiffness", "HeadYaw", 1.00 )
        robot.command( "Stiffness", "HeadPitch", 1.00 )
        ear_completion = 0
        head_pitch = 0.0
        send_stand = False
        while True :
            data = robot.read()
            positions_value = data[ "Position" ]
            positions = {}
            for index,name in enumerate (robot.joints):
                positions[name] = positions_value[index]
            sonars_value = data[ "Sonar" ]
            distances = {}
            for index,name in enumerate (robot.sonars):
                distances[name] = sonars_value[index]
            #print distances
            touch_value = data[ "Touch" ]
            touch = {}
            
            # Process buttons
            for index,name in enumerate (robot.touch):
                touch[name] = touch_value[index]
            robot.updateChestButton(touch['ChestBoard/Button'])
            print ("Button Presses: ", robot.chest_presses)
            if touch[ 'Head/Touch/Front' ] == True:
                if ear_completion <= 9.0 :
                    ear_completion += 0.1
                if head_pitch <= 1.0 :
                    head_pitch += 0.01
            elif touch[ 'Head/Touch/Rear' ] == True:
                if ear_completion > 0.0 :
                    ear_completion -= 0.1
                if head_pitch >= -1.0 :
                    head_pitch -= 0.01
            elif robot.chest_up > MAX_CLICK_INTERVAL and robot.chest_presses >= 2:
                send_stand = not send_stand
                robot.chest_presses = 0

            # Send commands
            # robot.commands[ 'LEar' ] = [ 1.0 if n <= int (ear_completion) else 0.0 for n in range ( 0, 10 )]
            # robot.commands[ 'LEar' ][ int (ear_completion)] = float (ear_completion - int (ear_completion))
            # robot.commands[ 'REar' ] = [ 1.0 if n <= int (ear_completion) else 0.0 for n in range ( 0, 10 )]
            # robot.commands[ 'REar' ][ int (ear_completion)] = float (ear_completion - int (ear_completion))
            robot.commands['LEar'] = [ 1.0 ] * 10
            robot.commands['REar'] = [ 1.0 ] * 10
            robot.setEyeColour('REye', 1.0, 1.0, 1.0)
            robot.setEyeColour('LEye', 1.0, 1.0, 1.0)
            robot.command( "Position", "HeadPitch", head_pitch)
            if send_stand:
                print ("Sending Stand commands")
                robot.command("Chest", "Green", 1.0)
                robot.setEyeColour('REye', 1.0, 0.1, 0.1)
                robot.setEyeColour('LEye', 0.0, 1.0, 1.0)

                robot.command("Position", "HeadYaw", deg2rad(0))
                robot.command("Position", "HeadPitch", deg2rad(0))
                robot.command("Position", "LShoulderPitch", deg2rad(90))
                robot.command("Position", "LShoulderRoll", deg2rad(10))
                robot.command("Position", "LElbowYaw", deg2rad(0))
                robot.command("Position", "LElbowRoll", deg2rad(0))
                robot.command("Position", "LWristYaw", deg2rad(0))
                robot.command("Position", "LHipYawPitch", deg2rad(0))
                robot.command("Position", "LHipRoll", deg2rad(0))
                robot.command("Position", "LHipPitch", deg2rad(-25))
                robot.command("Position", "LKneePitch", deg2rad(50))
                robot.command("Position", "LAnklePitch", deg2rad(-25))
                robot.command("Position", "LAnkleRoll", deg2rad(0))
                robot.command("Position", "RHipRoll", deg2rad(0))
                robot.command("Position", "RHipPitch", deg2rad(-25))
                robot.command("Position", "RKneePitch", deg2rad(50))
                robot.command("Position", "RAnklePitch", deg2rad(-25))
                robot.command("Position", "RAnkleRoll", deg2rad(0))
                robot.command("Position", "RShoulderPitch", deg2rad(90))
                robot.command("Position", "RShoulderRoll", deg2rad(-10))
                robot.command("Position", "RElbowYaw", deg2rad(0))
                robot.command("Position", "RElbowRoll", deg2rad(0))
                robot.command("Position", "RWristYaw", deg2rad(0))
                robot.command("Position", "LHand", deg2rad(0))
                robot.command("Position", "RHand", deg2rad(0))

                robot.command("Stiffness", "LShoulderPitch", 1.00)
                robot.command("Stiffness", "LShoulderRoll", 1.00)
                robot.command("Stiffness", "LElbowYaw", 1.00)
                robot.command("Stiffness", "LElbowRoll", 1.00)
                robot.command("Stiffness", "LWristYaw", 1.00)
                robot.command("Stiffness", "LHipYawPitch", 1.00)
                robot.command("Stiffness", "LHipRoll", 1.00)
                robot.command("Stiffness", "LHipPitch", 1.00)
                robot.command("Stiffness", "LKneePitch", 1.00)
                robot.command("Stiffness", "LAnklePitch", 1.00)
                robot.command("Stiffness", "LAnkleRoll", 1.00)
                robot.command("Stiffness", "RHipRoll", 1.00)
                robot.command("Stiffness", "RHipPitch", 1.00)
                robot.command("Stiffness", "RKneePitch", 1.00)
                robot.command("Stiffness", "RAnklePitch", 1.00)
                robot.command("Stiffness", "RAnkleRoll", 1.00)
                robot.command("Stiffness", "RShoulderPitch", 1.00)
                robot.command("Stiffness", "RShoulderRoll", 1.00)
                robot.command("Stiffness", "RElbowYaw", 1.00)
                robot.command("Stiffness", "RElbowRoll", 1.00)
                robot.command("Stiffness", "RWristYaw", 1.00)
                robot.command("Stiffness", "LHand", 1.00)
                robot.command("Stiffness", "RHand", 1.00)
            else:
                print ("No standing")
                robot.command("Chest", "Green", 0.0)
                robot.command( "Chest", "Red", abs (math.sin( 2 *time.time())))
                robot.command( "Chest", "Blue", abs (math.sin( 2 *time.time())))

                robot.command("Stiffness", "LShoulderPitch", 0.00)
                robot.command("Stiffness", "LShoulderRoll", 0.00)
                robot.command("Stiffness", "LElbowYaw", 0.00)
                robot.command("Stiffness", "LElbowRoll", 0.00)
                robot.command("Stiffness", "LWristYaw", 0.00)
                robot.command("Stiffness", "LHipYawPitch", 0.00)
                robot.command("Stiffness", "LHipRoll", 0.00)
                robot.command("Stiffness", "LHipPitch", 0.00)
                robot.command("Stiffness", "LKneePitch", 0.00)
                robot.command("Stiffness", "LAnklePitch", 0.00)
                robot.command("Stiffness", "LAnkleRoll", 0.00)
                robot.command("Stiffness", "RHipRoll", 0.00)
                robot.command("Stiffness", "RHipPitch", 0.00)
                robot.command("Stiffness", "RKneePitch", 0.00)
                robot.command("Stiffness", "RAnklePitch", 0.00)
                robot.command("Stiffness", "RAnkleRoll", 0.00)
                robot.command("Stiffness", "RShoulderPitch", 0.00)
                robot.command("Stiffness", "RShoulderRoll", 0.00)
                robot.command("Stiffness", "RElbowYaw", 0.00)
                robot.command("Stiffness", "RElbowRoll", 0.00)
                robot.command("Stiffness", "RWristYaw", 0.00)
                robot.command("Stiffness", "LHand", 0.00)
                robot.command("Stiffness", "RHand", 0.00)
            robot.send()
    except KeyboardInterrupt :
        print "Exit"
    finally :
        robot.close()

if __name__ == "__main__" :
    main()
