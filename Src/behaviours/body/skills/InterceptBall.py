from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.TurnToBall import TurnToBall
from body.skills.Walk import Walk
from body.skills.Lineup import Lineup, AlignmentVectors
from util.Global import ball_world_pos, myPos, we_see_ball, ball_distance, is_ball_lost, close_to_position, not_close_to_position
from util.Constants import CENTER_CIRCLE_DIAMETER, MIN_DISTANCE_TO_BALL
from util.GameStatus import own_goal
from math import radians

class InterceptBall(BehaviourTask):
    """
    A skill associated with walking to a defending point during a Free Kick. The defending
    point is half the diameter of the centre circle away from the ball, preferably with the robot's back facing their own goal.
    This is made with the assumption that the opponent will kick the ball towards the goal in a free
    kick, and not do a fancy pass to a teammate.
    
    The robot will also try to face the ball's direction and stand still to wait
    for the signal to proceed with the game.
    """
    
    HEADING_ERROR = radians(30)
    TURN_RATE = 1.5 

    BUFFER = 150
    THREE_QUARTER_CENTRE_CIRCLE = CENTER_CIRCLE_DIAMETER * 0.75
    MIN_DIST_TO_BALL_WITH_BUFFER = MIN_DISTANCE_TO_BALL + BUFFER
    
    _position_close = False
    _ball_found = False
    
    reference_point = None

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "TurnToBall": TurnToBall(self),
            "Stand": Walk(self),
            "Lineup": Lineup(self)
        }

    def _reset(self):
        self._ball_found = we_see_ball()
        self._ball_dist = ball_distance()
        self._target = AlignmentVectors(defensive=True).get_target()
        self._position_close = False

        self._current_sub_task = "Lineup"

    def _transition(self):
        if (self.reference_point is not None):
            self._target = AlignmentVectors(defensive=True, reference_point=self.reference_point).get_target()
        else:
            self._target = AlignmentVectors(defensive=True).get_target()

        if is_ball_lost():
            self._ball_found = False

        if we_see_ball():
            self._ball_found = True

        if self._ball_found:
            
            if self._position_close:
                self._current_sub_task = "TurnToBall"
                return
            else:
                self._current_sub_task = "Lineup"
                return

        self._current_sub_task = "TurnToBall"

    def _tick(self, reference_point=None, avoid_radius = None):
        self._ball_dist = ball_distance()
        self._too_close = self._ball_dist < self.MIN_DIST_TO_BALL_WITH_BUFFER
        self._ball_pos = ball_world_pos()
        self._my_pos = myPos()
        self.reference_point = reference_point

        if not self._position_close and close_to_position(self._target):
            self._position_close = True
        elif self._position_close and not_close_to_position(self._target):
            self._position_close = False

        if self._current_sub_task == "Lineup":
            self._tick_sub_task(defensive=True, reference_point=self.reference_point if self.reference_point is not None else own_goal(), avoid_radius=avoid_radius)
        else:
            self._tick_sub_task()
