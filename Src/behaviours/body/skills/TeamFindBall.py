from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Walk import Walk
from body.skills.TurnToBall import TurnToBall
from util.Global import myPos
from util.Vector2D import Vector2D
from util.Timer import Timer
from util import log
from math import radians
from util.MathUtil import normalisedTheta

from util.Constants import HALF_FIELD_LENGTH, HALF_FIELD_WIDTH

from body.skills.Stand import Stand
from util.Global import close_to_position, myPos, team_ball_world_pos, set_look_target, myHeading, is_ball_lost, time_since_last_team_ball_update, ego_ball_world_pos, ball_lost_time, ball_world_pos, ego_see_ball, ball_seen_in_last_n_frames

class TeamFindBall(BehaviourTask):
    """
    This skill searches the ball field if all robots have lost the ball.
    This means is team_ball hasn't been updated for 10 seconds and we can't see the ball.
    
    Gross desmos https://www.desmos.com/geometry/b6vf0hmkwe
    """
    
    # Used as a temporary solution before we implement BROWNIESSSSS!!!!!!!
    SHRINKING_FACTOR = 0.4
    # Time in seconds to wait for next posision
    SEARHCING_TIMEOUT = 10
    # The turn rate we use when turning towards the team ball
    TURN_RATE = 1.0
    
    ANGLE_ERROR_ACCEPTANCE = radians(45)  # degrees

    _posistions_sorted = False
    _ordered_posistions = []
    _default_posistion = Vector2D(0, 0)
    _walk_to_posistion = Vector2D(0, 0)
    _final_heading = None
    _close_point_at_final = False
    _max_positions = None
    _timer_running = False
    _last_seen_posistion = ball_world_pos()

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Stand": Stand(self),
            "TurnToLastBallPosistion": Walk(self),
            "TurnToBall": TurnToBall(self),
            "WalkToBall": WalkToPoint(self)
        }

    def _transition(self):
        shink_of_x = (abs((self._max_positions['east']) - (self._max_positions['west'])) / 2) * self.SHRINKING_FACTOR
        shink_of_y = (abs((self._max_positions['north']) - (self._max_positions['south'])) / 2) * self.SHRINKING_FACTOR
        
        points = [
            Vector2D(self._max_positions['west'] + shink_of_x, self._max_positions['north'] - shink_of_y),   # top left
            Vector2D(self._max_positions['east'] - shink_of_x , self._max_positions['north'] - shink_of_y),   # top right
            Vector2D(self._max_positions['west'] + shink_of_x , self._max_positions['south'] + shink_of_y),  # bottom left
            Vector2D(self._max_positions['east'] - shink_of_x, self._max_positions['south'] + shink_of_y)   # bottom right
        ]
        
        # TODO: add a variable that determines if team ball was most recently updated or team ball
        
        if ball_lost_time() > time_since_last_team_ball_update():
            # Use team ball posistion here
            self._last_seen_posistion = team_ball_world_pos()
        else:
            # Use ego ball posistion here
            self._last_seen_posistion = ego_ball_world_pos()
        
        # If the ball has been seen recently, we go into else and walk to it
        if not ball_seen_in_last_n_frames(10):
            if not self._posistions_sorted:
                log.debug("Looking for the ball", say=True)
                # Sort points so the closest Vector2D point to myPos() is last (so it will be popped first)
                self._ordered_posistions = sorted(points, key=lambda p: (p.minus(self._last_seen_posistion).length()))
                self._walk_to_posistion = self._ordered_posistions.pop()
                self._posistions_sorted = True
            
            if close_to_position(self._walk_to_posistion):
                # This needs to be reset by the parent task, currently only RoleSelector does this
                # However, this is "team find ball" so we should only be calling in one place 
                set_look_target(self._last_seen_posistion)
                
                if not self._timer_running:
                    self._time_to_look.start().restart()
                    self._timer_running = True
                
            if self._timer_running:
                if self._time_to_look.running_finished():
                    # TODO: Check to see if that pop the most recent posistion
                    if len(self._ordered_posistions) == 0:
                        self._current_sub_task = "TurnToBall"
                        self._posistions_sorted = False
                        return
                else:
                    if abs(self._ball_angle_difference(self._last_seen_posistion)) < self.ANGLE_ERROR_ACCEPTANCE:
                        self._current_sub_task = "TurnToBall"
                    else:
                        self._current_sub_task = "TurnToBall"
                        set_look_target(self._last_seen_posistion)
                        log.debug(f"Turning towards ball", say=True)
                    return

                log.debug("Moving to next objective", say=True)
                self._timer_running = False
                self._time_to_look.stop().restart()
                self._walk_to_posistion = self._ordered_posistions.pop()
                    
            
            self._current_sub_task = "WalkToPoint"
            
        else:
            self._current_sub_task = "WalkToBall"
            log.info(msg="Walk to ball", say=True)
            self._last_seen_posistion = ego_ball_world_pos()


    def _reset(self):
        self._current_sub_task = "TurnToBall" # Stand for two seconds on reset
        
        self._posistions_sorted = False
        self._ordered_posistions = []
        self._default_posistion = Vector2D(0, 0)
        self._walk_to_posistion = Vector2D(0, 0)
        self._final_heading = None
        self._close_point_at_final = False
        self._max_positions = {'north': HALF_FIELD_WIDTH, 'east': HALF_FIELD_LENGTH, 'south': -HALF_FIELD_WIDTH, 'west': -HALF_FIELD_LENGTH}
        self._time_to_look = Timer(target_seconds=self.SEARHCING_TIMEOUT).restart().stop()
        self._timer_running = False
        self._last_seen_posistion = team_ball_world_pos()
        

    def _tick(self, max_positions={'north': HALF_FIELD_WIDTH, 'east': HALF_FIELD_LENGTH, 'south': -HALF_FIELD_WIDTH, 'west': -HALF_FIELD_LENGTH}, default_position=Vector2D(0, 0)):
        self._default_posistion = default_position
        self._max_positions = max_positions

        # self._tick_sub_task() # Tick stand
        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=self._walk_to_posistion, speed=0.2, final_heading=self._final_heading)
        elif self._current_sub_task == "TurnToLastBallPosistion":
            self._tick_sub_task(
                turn=self.TURN_RATE if self._ball_angle_difference(self._last_seen_posistion) > 0 else -self.TURN_RATE
            )
        elif self._current_sub_task == "WalkToBall":
            self._tick_sub_task(
                final_pos=self._last_seen_posistion,
            )
        else:
            self._tick_sub_task() #TODO: Should tick both stand and turn as neither is walktopoint

    def _ball_angle_difference(self, position=Vector2D):
        return normalisedTheta(
                position.minus(myPos()).heading() - myHeading())