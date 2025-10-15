from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.TurnToBall import TurnToBall
from util.Vector2D import Vector2D
from util.Global import (
    is_ball_lost,
    close_to_position,
    myPos,
    ball_world_pos,
    set_goalie_stop_kick_in_check
)
from util.FieldGeometry import (
    OUR_LEFT_PENALTY_BOX_CORNER,
    OUR_RIGHT_PENALTY_BOX_CORNER,
    OUR_LEFT_CORNER,
    OUR_RIGHT_CORNER,
    OUR_RIGHT_GOAL_BOX_CORNER,
    OUR_LEFT_GOAL_BOX_CORNER,
    FIELD_CENTER,
    OUR_MIDLLE_GOAL_POST_BOX_CORNER
)
from util.GameStatus import just_outside_own_goal
from util import log
from util.Timer import Timer


class GoalieCornerKickIn(BehaviourTask):
    """
    This skill is used when the opposing team has a corner kick-in.
    The goalie searches for the ball, moves to the appropriate corner of the goal box,
    and positions itself to block the ball from entering the goal.
    """

    # The distance which we consider the robot to be close to a position
    CLOSE_TO_POSITION_DISTANCE = 250  # mm

    # Variance of the ball position from when found
    MOVED_BALL_VARIANCE = 450  # mm

    # position to walk to before kicking the ball
    _walk_to_position = just_outside_own_goal()
    _is_going_left = False
    _find_ball_attempts = 0
    # The direction the robot should look at when walking to the position
    _target_look = FIELD_CENTER
    _reached_goal = False
    _tracking_ball = False
    _found_ball_position = None  # The position of the ball once it's found

    def _initialise_sub_tasks(self):
        # Set sub-tasks
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "TurnToBall": TurnToBall(self),
        }

    def _transition(self):
        self._tracking_ball = not is_ball_lost(3)

        if (self._tracking_ball or self._find_ball_attempts > 2):
            ball_pos = ball_world_pos()

            if self._found_ball_position is None:
                # Log the first seen position
                self._found_ball_position = ball_pos
            else:
                distance = ball_pos.minus(self._found_ball_position).length()
                # If the ball is moved more than the variance this check will terminate this skill
                if distance > self.MOVED_BALL_VARIANCE:
                    set_goalie_stop_kick_in_check(True)

            # Goto the corner point that the ball was last seen in.
            if ball_pos.distanceTo(OUR_LEFT_PENALTY_BOX_CORNER) < ball_pos.distanceTo(OUR_RIGHT_PENALTY_BOX_CORNER):
                self._walk_to_position = Vector2D(
                    OUR_MIDLLE_GOAL_POST_BOX_CORNER, OUR_LEFT_GOAL_BOX_CORNER.y)
                self._target_look = OUR_LEFT_CORNER
            else:
                self._walk_to_position = Vector2D(
                    OUR_MIDLLE_GOAL_POST_BOX_CORNER, OUR_RIGHT_GOAL_BOX_CORNER.y)
                self._target_look = OUR_RIGHT_CORNER

            if close_to_position(position=self._walk_to_position) and self._tracking_ball:
                self._current_sub_task = "TurnToBall"

        else:
            # Find the ball that is lost
            self._look_for_ball()

    def _look_for_ball(self):
        """
        Description:
        This function will look for the ball when the ball is in a lost state. It does this by walking to each corner of the goalie box till it finds the ball.
        """

        # Get last known position of the ball
        # Checks to see if the ball is lost and it's close to the position that it wants to find
        # Then goes to the other side of the penalty box to find the ball
        if (close_to_position(self._walk_to_position, distance=self.CLOSE_TO_POSITION_DISTANCE) or self._find_ball_attempts == 0) and not self._reached_goal:
            self._reached_goal = True
            self._corner_wait_timer.start().restart()

        # Checks to see if the timer has finihsed and if the robot has reached the corner goal
        if self._reached_goal and self._corner_wait_timer.running_finished():
            self._find_ball_attempts += 1
            self._reached_goal = False
            if self._is_going_left:
                self._walk_to_position = OUR_RIGHT_GOAL_BOX_CORNER
                self._target_look = OUR_RIGHT_CORNER
                self._is_going_left = False
            else:
                self._walk_to_position = OUR_LEFT_GOAL_BOX_CORNER
                self._target_look = OUR_LEFT_CORNER
                self._is_going_left = True

    def _reset(self):
        self._is_going_left = False
        self._find_ball_attempts = 0
        self._tracking_ball = False
        self._corner_wait_timer = Timer(target_seconds=4)
        self._current_sub_task = "WalkToPoint"

    def _tick(self):
        if self._current_sub_task == "WalkToPoint":
            if self._tracking_ball:
                self._tick_sub_task(final_pos=self._walk_to_position, always_look_at_final=True, final_heading=(
                    self._target_look.minus(myPos())).heading())
            else:
                self._tick_sub_task(final_pos=self._walk_to_position, final_heading=(
                    self._target_look.minus(myPos())).heading())
        else:
            self._tick_sub_task()
