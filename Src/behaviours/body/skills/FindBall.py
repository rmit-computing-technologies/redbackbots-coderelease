from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from util.Global import myPos
from util.Vector2D import Vector2D
from util import log

from util.Constants import HALF_FIELD_LENGTH, HALF_FIELD_WIDTH

from body.skills.Stand import Stand
from util.Global import close_to_position, myPos, team_ball_world_pos, time_since_last_team_ball_update, set_look_target

class FindBall(BehaviourTask):
    """
    Tries a bunch of techniques to find the ball if lost.
    First tries spinning around. If that doesn't work,
    tries walking to a bunch of points and doing the same thing.
    """
    
    SCALING_FACTOR = 0.5

    _posistions_sorted = False
    _ordered_posistions = []
    _default_posistion = Vector2D(0, 0)
    _walk_to_posistion = Vector2D(0, 0)
    _final_heading = None
    _close_point_at_final = False
    _max_positions = None

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Stand": Stand(self)
        }

    def _transition(self):
        points = [
            Vector2D((self._max_positions['east'] + self._max_positions['west']) / 2 * self.SCALING_FACTOR, self._max_positions['north'] * self.SCALING_FACTOR),   # left middle
            Vector2D((self._max_positions['east'] + self._max_positions['west']) / 2 * self.SCALING_FACTOR, self._max_positions['south'] * self.SCALING_FACTOR),   # right middle
            Vector2D(self._max_positions['east'] * self.SCALING_FACTOR, (self._max_positions['north'] + self._max_positions['south']) / 2 * self.SCALING_FACTOR),  # bottom middle
            Vector2D(self._max_positions['west'] * self.SCALING_FACTOR, (self._max_positions['north'] + self._max_positions['south']) / 2 * self.SCALING_FACTOR)   # top middle
        ]
        
        if time_since_last_team_ball_update() > 5 and not self._posistions_sorted:
            # If no team ball update for a while, use the default positions
            self._ordered_posistions = points
            self._posistions_sorted = True
        
        if self._posistions_sorted and close_to_position(self._walk_to_posistion):
            # TODO: Check to see if that pop the most recent posistion
            if len(self._ordered_posistions) == 0:
                self._current_sub_task = "Stand"
                self._posistions_sorted = False
                return
                
            self._walk_to_posistion = self._ordered_posistions.pop()
        
        if close_to_position(self._walk_to_posistion):
            self._final_heading = (team_ball_world_pos().minus(myPos())).heading()
            
            # This needs to be reset by the parent task, currently only RoleSelector does this
            # However, this is "team find ball" so we should only be calling in one place 
            set_look_target(team_ball_world_pos())
        
        self._current_sub_task = "WalkToPoint"


    def _reset(self):
        self._current_sub_task = "Stand" # Stand for two seconds on reset
        
        self._posistions_sorted = False
        self._ordered_posistions = []
        self._default_posistion = Vector2D(0, 0)
        self._walk_to_posistion = Vector2D(0, 0)
        self._final_heading = None
        self._close_point_at_final = False
        self._max_positions = {'north': HALF_FIELD_WIDTH, 'east': HALF_FIELD_LENGTH, 'south': -HALF_FIELD_WIDTH, 'west': -HALF_FIELD_LENGTH}

    def _tick(self, max_positions=None, default_position=None):
        self._default_posistion = default_position
        self._max_positions = max_positions

        # self._tick_sub_task() # Tick stand
        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=self._walk_to_posistion, speed=0.2, final_heading=self._final_heading)
        else:
            self._tick_sub_task() #TODO: Should tick both stand and turn as neither is walktopoint
