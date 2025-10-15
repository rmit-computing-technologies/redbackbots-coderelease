from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand
from util.Constants import LEDColour

import util.led_override as led_override


class Finished(BehaviourTask):

    """ 
        Description:
        A skill associated with the status of the robot once the game is finished.
    """
    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "Stand"


    def _tick(self):
        led_override.override(led_override.CHEST_BUTTON, LEDColour.finished)
        self._tick_sub_task()