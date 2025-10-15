from BehaviourTask import BehaviourTask
from body.skills.RoleSelector import RoleSelector
from util.Constants import LEDColour, HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, OUT_OF_FIELD_BUFFER_SIZE, IN_FIELD_BUFFER_SIZE, FIELD_LENGTH
from util.Vector2D import Vector2D
from util import log

class Upfielder(BehaviourTask):

    """ 
        Description:
        This class contains the actioins that the upfielder would carry out. 
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "RoleSelector": RoleSelector(self)
        }

    def _reset(self):
        self._current_sub_task = "RoleSelector"

    def _tick(self, lr='Center'):
        if lr == 'Center':
            _max_positions = {'north': HALF_FIELD_WIDTH + OUT_OF_FIELD_BUFFER_SIZE, 'east': HALF_FIELD_LENGTH + OUT_OF_FIELD_BUFFER_SIZE, 'south': -HALF_FIELD_WIDTH - OUT_OF_FIELD_BUFFER_SIZE, 'west': HALF_FIELD_LENGTH/3.0 - IN_FIELD_BUFFER_SIZE}
            _default_position = Vector2D(FIELD_LENGTH/3.0, 0)
        
        self._tick_sub_task(field_position = ("Upfielder", lr), default_position = _default_position, max_positions = _max_positions)
