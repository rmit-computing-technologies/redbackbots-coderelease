import robot

from BehaviourTask import BehaviourTask
from body.roles.PositionSelector import PositionSelector
from body.skills.TurnToBall import TurnToBall

from util.Constants import LEDColour
import util.led_override as led_override
from util.Timer import Timer
from util.Global import is_ball_lost

class Playing(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "PositionSelector": PositionSelector(self),
        }

    def _reset(self):
        self._current_sub_task = "PositionSelector"
        
    def _tick(self):

        led_override.override(led_override.CHEST_BUTTON, LEDColour.playing)

        self._tick_sub_task()
