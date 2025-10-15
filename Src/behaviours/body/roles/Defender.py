from BehaviourTask import BehaviourTask
from body.skills.RoleSelector import RoleSelector
from util.Constants import HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, CENTER_CIRCLE_DIAMETER
from util.Vector2D import Vector2D
from util import log

class Defender(BehaviourTask):

    """ 
        Description:
        This class contains the actions that the defender would carry out. 
    """

    OUT_OF_FIELD_BUFFER_SIZE = CENTER_CIRCLE_DIAMETER / 2.0  #mm
    IN_FIELD_BUFFER_SIZE = CENTER_CIRCLE_DIAMETER / 4.0 #mm

    # Slightly reduce defender quadrant in center to avoid collison with attackers
    DEFENDER_MIDFIELD_OFFSET = CENTER_CIRCLE_DIAMETER / 4.0 #mm

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "RoleSelector": RoleSelector(self)
        }

    def _reset(self):
        self._current_sub_task = "RoleSelector"

    def _tick(self, lr='Center'):
        _default_position = Vector2D(0, 0)
        _max_positions = None
        if lr == 'Left':
            _max_positions = {
                'north': HALF_FIELD_WIDTH + self.OUT_OF_FIELD_BUFFER_SIZE,
                'east': 0 - self.DEFENDER_MIDFIELD_OFFSET,
                'south': 0 - self.IN_FIELD_BUFFER_SIZE,
                'west': -HALF_FIELD_LENGTH - self.OUT_OF_FIELD_BUFFER_SIZE
            }
            _default_position = Vector2D(-HALF_FIELD_LENGTH/2.0, HALF_FIELD_WIDTH/2.0)
        elif lr == 'Center':
            _max_positions = {
                'north': HALF_FIELD_WIDTH + self.OUT_OF_FIELD_BUFFER_SIZE,
                'east': 0 - self.DEFENDER_MIDFIELD_OFFSET,
                'south': -HALF_FIELD_WIDTH - self.OUT_OF_FIELD_BUFFER_SIZE,
                'west': -HALF_FIELD_LENGTH - self.OUT_OF_FIELD_BUFFER_SIZE
            }
            _default_position = Vector2D(-HALF_FIELD_LENGTH/2.0, 0)
        elif lr == 'Right':
            _max_positions = {
                'north': 0 + self.IN_FIELD_BUFFER_SIZE,
                'east': 0 - self.DEFENDER_MIDFIELD_OFFSET,
                'south': -HALF_FIELD_WIDTH - self.OUT_OF_FIELD_BUFFER_SIZE,
                'west': -HALF_FIELD_LENGTH - self.OUT_OF_FIELD_BUFFER_SIZE
            }
            _default_position = Vector2D(-HALF_FIELD_LENGTH/2.0, -HALF_FIELD_WIDTH/2.0)
        else:
            log.error(f"Invalid lr value for Defender: {lr}", say=True)

        self._tick_sub_task(
            field_position = ("Defender", lr),
            default_position = _default_position,
            max_positions = _max_positions
        )
