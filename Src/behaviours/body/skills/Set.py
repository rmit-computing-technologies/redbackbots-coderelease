from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand

from util.Constants import LEDColour

import util.led_override as led_override

class Set(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "Stand"


    def _tick(self):

        led_override.override(led_override.CHEST_BUTTON, LEDColour.set)

        self._tick_sub_task()