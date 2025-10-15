from BehaviourTask import BehaviourTask
from body.skills.RoleSelector import RoleSelector
from util.Vector2D import Vector2D
from util.Constants import LEDColour, HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, CENTER_CIRCLE_DIAMETER

class Superstar(BehaviourTask):

    """ 
        Description:
        The superstar role only occurs when there is 1 remaining field robot. 
    """

    OUT_OF_FIELD_BUFFER_SIZE = CENTER_CIRCLE_DIAMETER / 2.0 #mm

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "RoleSelector": RoleSelector(self)
        }

    def _reset(self):
        self._current_sub_task = "RoleSelector"

    def _tick(self):
        _max_positions = {'north': HALF_FIELD_WIDTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'east': HALF_FIELD_LENGTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'south': -HALF_FIELD_WIDTH - self.OUT_OF_FIELD_BUFFER_SIZE, 'west': -HALF_FIELD_LENGTH - self.OUT_OF_FIELD_BUFFER_SIZE}
        _default_position = Vector2D(0, 0)

        self._tick_sub_task(field_position = ("Superstar", None), default_position = _default_position, max_positions = _max_positions)