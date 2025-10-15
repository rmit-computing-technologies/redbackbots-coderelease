from math import radians

from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch


class HeadCentreDown(BehaviourTask):

    """ 
        Description:
        Fixing the center position of the head
        while looking down at the field
    """

    YAW = 0
    # PITCH = radians(19)
    PITCH = radians(0)
    # PITCH = radians(-10)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "HeadFixedYawAndPitch": HeadFixedYawAndPitch(self)
        }

    def _reset(self):
        self._current_sub_task = "HeadFixedYawAndPitch"

    def _tick(self):
        self._tick_sub_task(self.YAW, self.PITCH)
