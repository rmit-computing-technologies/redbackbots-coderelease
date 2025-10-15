from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand

from util.Constants import LEDColour

import util.led_override as led_override


class Standby(BehaviourTask):

    """ 
        Description:
        This skill is associated with the standby state
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "Stand"

    def _transition(self):
        self._current_sub_task = "Stand"


    def _tick(self):
        led_override.override(led_override.CHEST_BUTTON, LEDColour.standby)
        self._tick_sub_task()