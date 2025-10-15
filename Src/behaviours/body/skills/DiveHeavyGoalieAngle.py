from BehaviourTask import BehaviourTask
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from body.skills.WalkToGoalBox import WalkToGoalBox
from body.skills.Walk import Walk
from body.skills.Stand import Stand
from body.skills.Localise import Localise
from head.HeadAware import HeadAware
from util.Global import ball_world_pos, myPos, myHeading, ego_ball_lost, ego_ball_distance, ego_ball_heading, we_see_ball
from util.Constants import FIELD_LENGTH, PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH, GOAL_BOX_LENGTH, HALF_FIELD_LENGTH
from util.GameStatus import enemy_goal, just_outside_own_goal, is_ball_in_attacking_half
from util.MathUtil import normalisedTheta
from math import radians


class GoalieAngle(BehaviourTask):
    """
    The GoalieAngle skill handles the behavior for adjusting the goalie's position and orientation 
    relative to the ball and the goal. It focuses on positioning the goalie inside the goal box and 
    aligning it towards the ball or the enemy goal, while considering the ball's visibility, location, 
    and game context.

    Attributes:
        HEADING_ERROR (float): The maximum heading error (in radians) allowed for the goalie to consider itself aligned.
        _position_close (bool): A flag indicating whether the goalie is close to its target position.
        SCALE_FACTOR (float): Scaling factor for positioning adjustments, calculated from field dimensions.
        TURN_RATE (float): The rate at which the goalie turns when adjusting its orientation.
        _tracking_ball (bool): A flag indicating if the goalie is actively tracking the ball.
        _reference_point (Vector2D): The default position near the goal where the goalie should align itself.
        _target_pos (Vector2D): The current target position where the goalie is trying to move.
        _go_home (bool): A flag indicating whether the goalie should return to its default goal position.

    Methods:
        _heading_error():
            Calculates and returns the heading error between the goalie's current orientation and the direction 
            to the enemy goal. The error is normalized to ensure it falls within valid heading boundaries.

    Decision Tree: 'Docs/behaviours/body/skills/GoalieAngle.png'
    Markdown:      'Docs/behaviours/body/skills/GoalieAngle.md'
    """
    CLOSE_DISTANCE = 50  # mm
    NOT_CLOSE_DISTANCE = 150  # mm
    HEADING_ERROR = radians(30)

    _position_close = False
    SCALE_FACTOR = (PENALTY_BOX_LENGTH / FIELD_LENGTH) * 0.75
    HALF_GOAL_BOX_LENGTH = GOAL_BOX_LENGTH/2

    TURN_RATE = 1.5
    _tracking_ball = False

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "TurnToBall": TurnToBall(self),
            "WalkWithoutTurning": Walk(self),
            "Localise": Localise(self),
            "WalkToGoalBox": WalkToGoalBox(self),
            "FaceForward": Walk(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "WalkWithoutTurning"
        self._position_close = False
        self._reference_point = just_outside_own_goal()
        self._target_pos = self._reference_point
        self._go_home = True

    def _transition(self):
        if ego_ball_lost(7): # 7 seconds
            self._tracking_ball = False

        if we_see_ball() or self._tracking_ball:
            self._tracking_ball = True
            self._go_home = False
            if self._position_close:
                self._current_sub_task = "TurnToBall"
                return  
            self._current_sub_task = "WalkWithoutTurning"
            return
        
        # If Lost -> relocalise  
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return
        
        # Ball has been lost for some time and we're not close to `just_outside_own_goal()`
        if ego_ball_lost() and self._target_pos.minus(myPos()).length2() > self.NOT_CLOSE_DISTANCE**2:
            self._go_home = True

        # We've finished walking home
        if self._current_sub_task == "WalkToGoalBox" and self._sub_tasks[self._current_sub_task]._is_finished:
            self._go_home = False

        # Outside goalbox or told to realign back to centre
        if (abs(myPos().y) > PENALTY_BOX_WIDTH/2 or abs(myPos().x) < (HALF_FIELD_LENGTH-PENALTY_BOX_LENGTH)) or self._go_home:
            self._current_sub_task = "WalkToGoalBox"
            return
        
        # Currently walking to goalbox -> keep going
        if self._current_sub_task == "WalkToGoalBox" and not self._sub_tasks[self._current_sub_task]._is_finished:
            self._current_sub_task = "WalkToGoalBox"
            return

        if abs(self._heading_error()) < self.HEADING_ERROR:
            self._current_sub_task = "Stand"
            return
        
        self._current_sub_task = "FaceForward"

    def _tick(self):
        self._ball_pos = ball_world_pos()
        self._target_pos = self._reference_point

        if self._tracking_ball:
            self._target_pos = self._reference_point.minus((self._reference_point.minus(self._ball_pos)).scale(self.SCALE_FACTOR))
            if not is_ball_in_attacking_half():
                self._target_pos.x = self._reference_point.x

        if not self._position_close and self._target_pos.minus(myPos()).length2() < self.CLOSE_DISTANCE**2:
            self._position_close = True
        elif self._position_close and self._target_pos.minus(myPos()).length2() > self.NOT_CLOSE_DISTANCE**2:
            self._position_close = False 
        
        if self._current_sub_task == "FaceForward":
            self._tick_sub_task(turn=self.TURN_RATE if self._heading_error() > 0
                else -self.TURN_RATE)
        elif self._current_sub_task == "WalkWithoutTurning":
            x_diff = myPos().x - self._target_pos.x
            y_diff = myPos().y - self._target_pos.y
            
            ball_distance = ego_ball_distance()
            if ball_distance < 1000:
                walk_modifier = 1
            else:
                walk_modifier = (ball_distance/500)-1

            if ball_distance < 4500: # 4500/3000 = 1.5 so reverts back to standard 
                turn_rate = ball_distance/3000
            else:
                turn_rate = self.TURN_RATE

            if abs(x_diff) < WalkToPoint.WALK_SPEED:
                walk_speed = abs(x_diff)
            else:
                walk_speed = WalkToPoint.WALK_SPEED / walk_modifier
            
            self._tick_sub_task(
                forward = -walk_speed if x_diff > 0 else walk_speed, 
                left = -WalkToPoint.WALK_SPEED/walk_modifier if y_diff > 0 else WalkToPoint.WALK_SPEED/walk_modifier, 
                speed = 1/walk_modifier, 
                turn=turn_rate if ego_ball_heading() > 0 else -turn_rate
                )
                
        elif self._current_sub_task == "WalkToGoalBox":
            self._tick_sub_task(self._target_pos)
        else:
            self._tick_sub_task()

    def _heading_error(self):
        # Face enemy goal
        return normalisedTheta(
            enemy_goal().minus(myPos()).heading() - myHeading())
            