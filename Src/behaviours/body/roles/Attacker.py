from BehaviourTask import BehaviourTask
from body.skills.RoleSelector import RoleSelector
from util.Vector2D import Vector2D
from util.Constants import LEDColour, HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, CENTER_CIRCLE_DIAMETER

class Attacker(BehaviourTask):

    """ 
        Description:
        This class contains the actioins that the attacker would carry out. 
    """

    OUT_OF_FIELD_BUFFER_SIZE = CENTER_CIRCLE_DIAMETER / 2.0 #mm
    IN_FIELD_BUFFER_SIZE = CENTER_CIRCLE_DIAMETER / 4.0

    # Slightly increase attacker quadrant in center to avoid collison with defenders
    ATTACKER_MIDFIELD_OFFSET = CENTER_CIRCLE_DIAMETER / 4.0 #mm

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "RoleSelector": RoleSelector(self)
        }

    def _reset(self):
        self._current_sub_task = "RoleSelector"

    def _tick(self, lr='Center'):
        if lr == 'Left':
            _max_positions = {'north': HALF_FIELD_WIDTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'east': HALF_FIELD_LENGTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'south': 0 - self.IN_FIELD_BUFFER_SIZE, 'west': 0 - self.ATTACKER_MIDFIELD_OFFSET}
            _default_position = Vector2D(HALF_FIELD_LENGTH/2.0, HALF_FIELD_WIDTH/2.0)
        elif lr == 'Center':
            _max_positions = {'north': HALF_FIELD_WIDTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'east': HALF_FIELD_LENGTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'south': -HALF_FIELD_WIDTH - self.OUT_OF_FIELD_BUFFER_SIZE, 'west': 0 - self.ATTACKER_MIDFIELD_OFFSET}
            _default_position = Vector2D(HALF_FIELD_LENGTH/2.0, HALF_FIELD_WIDTH/4.0)
        elif lr == 'Right':
            _max_positions = {'north': 0 + self.IN_FIELD_BUFFER_SIZE, 'east': HALF_FIELD_LENGTH + self.OUT_OF_FIELD_BUFFER_SIZE, 'south': -HALF_FIELD_WIDTH - self.OUT_OF_FIELD_BUFFER_SIZE, 'west': 0 - self.ATTACKER_MIDFIELD_OFFSET}
            _default_position = Vector2D(HALF_FIELD_LENGTH/2.0, -HALF_FIELD_WIDTH/2.0)

        self._tick_sub_task(field_position = ("Attacker", lr), default_position = _default_position, max_positions = _max_positions)
