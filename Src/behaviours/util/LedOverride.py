# flake8: noqa
"""LED Overrides.

Allows a behaviour to request an override of the default eye behaviours during testing.
To use this, simple import this module, and then call the override function with a key and a color. Like so

You can use .override for the chestButton
LedOverride.override(LedOverride.chestButton, LEDColors.playing)

For the eyes, you can specify which segments you want to change colour
LedOverride.override_eye_segment(LedOverride.rightEye, [1,2,3], LEDColour.dark_green)

# Note that LEDColors is an enum in Constants.
"""

import robot

rightEye = "rEye"
leftEye = "lEye"
rightFoot = "rFoot"
leftFoot = "lFoot"
chestButton = "chestButton"
roleSegments = [2,3,4,5,6]
positionSegments = [7,0,1]
allSegments = [0,1,2,3,4,5,6,7,8]

NUM_EYE_SEGMENTS = 8

_overrides = {}
def reset_led_override():
   global _overrides
   _overrides = {}

def override_request(request):
   """Override Request.

   Overrides parts of a request with the singleton in this module.
   @param request A behaviour request
   """
   global _overrides
   if rightEye in _overrides:
      request.actions.leds.rightEye = robot.rgbSegments(_overrides[rightEye])
   if leftEye in _overrides:
      request.actions.leds.leftEye = robot.rgbSegments(_overrides[leftEye])
   if rightFoot in _overrides:
      request.actions.leds.rightFoot = robot.rgb(_overrides[rightFoot][0], _overrides[rightFoot][1], _overrides[rightFoot][2])
   if leftFoot in _overrides:
      request.actions.leds.leftFoot = robot.rgb(_overrides[leftFoot][0],_overrides[leftFoot][1],_overrides[leftFoot][2])
   if chestButton in _overrides:
      request.actions.leds.chestButton = robot.rgb(_overrides[chestButton][0], _overrides[chestButton][1], _overrides[chestButton][2])

   # TODO(Ritwik): Add more leds.

def override(key, value):
   global _overrides
   _overrides[key] = value

def override_eye_segment(eye, segments, value):
   # print(f"Setting segment {segment} to {value}")
   global _overrides
   new_colour = []
   if eye in _overrides:
      # print(f"Current {eye} value: ")
      new_colour = _overrides[eye]
      for i in range(NUM_EYE_SEGMENTS):
         if i in segments:
            new_colour[i*3] = value[0]
            new_colour[i*3 +1] = value[1]
            new_colour[i*3 +2] = value[2]
   else:
      # Otherwise set all segments to white, then change the specified colour
      # print(f"Eye is not set")
      for i in range(NUM_EYE_SEGMENTS):
         if i in segments:
            new_colour += value
         else:
            new_colour += [.5,.5,.5]
   # print(f"New colour: {new_colour}")
   _overrides[eye] = new_colour