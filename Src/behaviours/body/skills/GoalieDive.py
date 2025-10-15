from util.actioncommand import goalieDiveLeft,goalieDiveRight, goalieCentre, stand
from util import log, led_override
from util.Constants import LEDColour
from util.TeamStatus import GOALIE_PLAYER_NUMBERS, my_player_number

from BehaviourTask import BehaviourTask


class GoalieDive(BehaviourTask):
    """
    The GoalieDive class represents the behavior for the goalie to dive in different directions 
    based on a given parameter. The goalie can dive to the left, right, or center, depending 
    on the given direction and allowed conditions.

    The goalie will stand if the given direction is not allowed.

    Attributes:
        ALLOW_SIDE_DIVES (bool): Determines if the goalie is allowed to dive to the left or right.
        ALLOW_CENTER_DIVE (bool): Determines if the goalie is allowed to dive to the center.

    Parameters:
        direction (str): Specifies the direction of the dive ("Left", "Right", or ""). 
            Defaults to an empty string for a center dive.
    """

    # Center dives are somewhat safe in the lab,
    # side dives should only be used at competition if at all
    ALLOW_SIDE_DIVES = True
    ALLOW_CENTER_DIVE = True

    DIVE_CENTRE_WIDTH = 450 # Rough measurement
    DIVE_SIDE_WIDTH = 1000

    def _tick(self, direction = ""):
        if my_player_number not in GOALIE_PLAYER_NUMBERS:
            led_override.override(led_override.LEFT_FOOT, LEDColour.orange)
            led_override.override(led_override.RIGHT_FOOT, LEDColour.orange)
            self.world.b_request.actions.body = stand()
            return
        if direction == "Left":
            led_override.override(led_override.LEFT_FOOT, LEDColour.orange)
            log.warning("DIVE LEFT", say=True)
            self.world.b_request.actions.body = goalieDiveLeft() if self.ALLOW_SIDE_DIVES else stand()
            return
        if direction == "Right":
            led_override.override(led_override.RIGHT_FOOT, LEDColour.orange)
            log.warning("DIVE RIGHT", say=True)
            self.world.b_request.actions.body = goalieDiveRight() if self.ALLOW_SIDE_DIVES else stand()
            return
        else:
            led_override.override(led_override.LEFT_FOOT, LEDColour.orange)
            led_override.override(led_override.RIGHT_FOOT, LEDColour.orange)
            log.warning("DIVE CENTRE", say=True)
            self.world.b_request.actions.body = goalieCentre() if self.ALLOW_CENTER_DIVE else stand()
            return
