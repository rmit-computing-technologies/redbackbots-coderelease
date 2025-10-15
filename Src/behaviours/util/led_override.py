# flake8: noqa
"""LED Overrides.

Allows a behaviour to request an override of the default eye behaviours during testing.
To use this, simple import this module, and then call the override function with a key and a color

You can use .override for the chestButton
led_override.override(led_override.chestButton, LEDColors.playing)

For the eyes, you can specify which segments you want to change colour
led_override.override_eye_segment(led_override.RIGHT_EYE, [1,2,3], LEDColour.dark_green)

# Note that LEDColors is an enum in Constants.
"""

import robot

RIGHT_EYE = "rEye"
LEFT_EYE = "lEye"
RIGHT_FOOT = "rFoot"
LEFT_FOOT = "lFoot"
CHEST_BUTTON = "chestButton"

# Right Eye Segments
POSITION_SEGMENTS = [2,3,4,5,6]
ROLE_SEGMENTS = [7,0,1]

# Left Eye Segments
BALL_SEEN_SEGMENTS = [3,4,5]
TEAM_BALL_SEGMENTS = [0,1,2]
HEAD_TRACK_SEGMENTS = [6,7]

allSegments = [0,1,2,3,4,5,6,7,8]

NUM_EYE_SEGMENTS = 8

_overrides = {}
_tick = 0
def reset_led_override():
    global _overrides
    _overrides = {}

def override_request(request):
    """Override Request.

    Overrides parts of a request with the singleton in this module.
    @param request A behaviour request
    """
    global _overrides
    if RIGHT_EYE in _overrides:
        request.actions.leds.rightEye = robot.rgbSegments(_overrides[RIGHT_EYE])
    if LEFT_EYE in _overrides:
        request.actions.leds.leftEye = robot.rgbSegments(_overrides[LEFT_EYE])
    if RIGHT_FOOT in _overrides:
        request.actions.leds.rightFoot = robot.rgb(_overrides[RIGHT_FOOT][0], _overrides[RIGHT_FOOT][1], _overrides[RIGHT_FOOT][2])
    if LEFT_FOOT in _overrides:
        request.actions.leds.leftFoot = robot.rgb(_overrides[LEFT_FOOT][0],_overrides[LEFT_FOOT][1],_overrides[LEFT_FOOT][2])
    if CHEST_BUTTON in _overrides:
        request.actions.leds.chestButton = robot.rgb(_overrides[CHEST_BUTTON][0], _overrides[CHEST_BUTTON][1], _overrides[CHEST_BUTTON][2])


def override(key, value):
    global _overrides
    _overrides[key] = value

def override_eye_pattern(eye, pattern, speed=20):
    """
    Override the selected eye with a given pattern
    that is a list of 8 LEDColours.
    This pattern will then shift back and forward
    according to the supplied speed
    """
    global _overrides
    new_colour = []
    global _tick

    if _tick > speed/2:
        pattern = pattern[1:] + pattern[:1]
    if _tick == speed:
        _tick = 0
    _tick += 1

    for i in range(NUM_EYE_SEGMENTS):
        new_colour += pattern[i]

    _overrides[eye] = new_colour

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