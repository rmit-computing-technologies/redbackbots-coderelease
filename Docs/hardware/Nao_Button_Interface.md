# Nao Button Interface

The Nao v6 has mutliple touch sensors on the head, hands and feet as well as a button on the chest.

## Button Names

>- "ChestBoard/Button"
>- "Head/Touch/Front"
>- "Head/Touch/Middle"
>- "Head/Touch/Rear"
>- "LFoot/Bumper/Left"
>- "LFoot/Bumper/Right"
>- "LHand/Touch/Back"
>- "LHand/Touch/Left"
>- "LHand/Touch/Right"
>- "RFoot/Bumper/Left"
>- "RFoot/Bumper/Right"
>- "RHand/Touch/Back"
>- "RHand/Touch/Left"
>- "RHand/Touch/Right"

## Turning Nao On and Off

### Turning On

To turn the robot on, press the chest button once.
After the robot has finished booting, the chest button will flash red and the robot will say “Ognak gnouk”.

> **Warning!**
>
>- Don’t move the robot while its starting up, as the IMU is calibrated during boot.
>- Don’t hold down the chest button, as this will cause the robot to start doing a factory reset!

### Turning Off

To shutdown the robot, press and hold the chest button for 3 seconds or until the robot says "Gnuk gnuk". Once all LEDs are off, the shutdown is complete.

If shutdown is not working, an emergency shutdown can be performed by holding the chest button for 8 seconds.

## Starting and Stopping Redbackbots

### Daemon

When the robot first boots (and after redbackbots quits) a simple daemon script runs that listens for button presses among other things.

While this daemon is running, redbackbots can be started by pressing the chest button 3 times or by swiping the head from front to rear.

### LoLATouch

While redbackbots is running, button presses are handled by LolaTouch (part of the motion thread).

When redbackbots starts, the robot begins in an unstiff state, to stiffen the robot single press the chest button. The robot will then stand up and run the default/selected skill.

To enter the unstiff state again, press all head buttons at the same time. The robot will sit down and unstiffen.

To kill redbackbots, swipe the head buttons from front to back. The robot will sit down and unstiffen. The daemon script will start running again.

## Other Button Presses

Manually Penalise/Unpenalise Robot:

- Single press chest button

Restart Naoqi:

- Triple press the chest button at the same time as one of the foot bumpers

Restart WiFi:

- Quadruple press the chest button'
