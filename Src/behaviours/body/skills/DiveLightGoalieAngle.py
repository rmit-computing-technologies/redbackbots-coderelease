from BehaviourTask import BehaviourTask
from body.skills.GoalieDive import GoalieDive
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from body.skills.WalkToGoalBox import WalkToGoalBox
from body.skills.Walk import Walk
from body.skills.Stand import Stand
from body.skills.Localise import Localise
from head.HeadAware import HeadAware
from util.Global import ball_world_pos, myPos, myHeading, ball_distance, ball_heading, we_see_ball, is_ball_lost, not_close_to_position
from util.Constants import FIELD_LENGTH, PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH, GOAL_BOX_LENGTH, HALF_FIELD_WIDTH, HALF_FIELD_LENGTH, GOAL_POST_ABS_Y
from util.GameStatus import enemy_goal, just_outside_own_goal
from util.MathUtil import normalisedTheta, intersectLinesPosAndDir
from util.Vector2D import Vector2D
from util.FieldGeometry import OUR_LEFT_POST, OUR_RIGHT_POST, heading_error
from math import radians, copysign, isfinite, sqrt, sin

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

    Decision Tree: 'Docs/behaviours/body/skills/GoalieAngle.png'
    Markdown:      'Docs/behaviours/body/skills/GoalieAngle.md'
    """
    HEADING_ERROR = radians(30)

    TURN_RATE = 1.5
    WALK_MODIFIER_DEFAULT = 1
    _tracking_ball = False

    # Aggressive Goalie positioning
    AGGRESSIVE_OFFSET_AT_MIN_X = 50
    AGGRESSIVE_OFFSET_AT_MAX_X = 0
    AGGRESSIVE_GOALIE_POSITION_ANGLE = radians(130)
    
    # Default Goalie positioning
    DEFAULT_OFFSET_AT_MIN_X = 50
    DEFAULT_OFFSET_AT_MAX_X = -50
    DEFAULT_GOALIE_POSITION_ANGLE = radians(110)

    ENEMY_GOAL = enemy_goal()

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
        self._reference_point = just_outside_own_goal()
        self._target_pos = self._reference_point
        self._go_home = True
        
        # Default to non aggressive goalie positioning
        # Default offset from the goal line forward
        self._offset_at_min_x = self.DEFAULT_OFFSET_AT_MIN_X
        # Default offset from the penalty box line forward
        self._offset_at_max_x = self.DEFAULT_OFFSET_AT_MAX_X
        self._goalie_position_angle = self.DEFAULT_GOALIE_POSITION_ANGLE

    def _transition(self):
        if is_ball_lost(7): # 7 seconds
            self._tracking_ball = False

        if we_see_ball() or self._tracking_ball:
            self._tracking_ball = True
            self._go_home = False
            
            self._current_sub_task = "WalkWithoutTurning"
            return
        
        # If Lost -> relocalise  
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return

        # Ball has been lost for some time and we're not close to `just_outside_own_goal()`
        if is_ball_lost() and not_close_to_position(self._target_pos):
            self._go_home = True

        # We've finished walking home
        if self._current_sub_task == "WalkToGoalBox" and self._sub_tasks[self._current_sub_task]._is_finished:
            self._go_home = False

        # Outside max x or told to realign back to centre
        if (abs(myPos().y) > PENALTY_BOX_WIDTH/2 or myPos().x > (-HALF_FIELD_LENGTH + PENALTY_BOX_LENGTH + self._offset_at_max_x)) or self._go_home:
            self._current_sub_task = "WalkToGoalBox"
            return
        
        # Currently walking to goalbox -> keep going
        if self._current_sub_task == "WalkToGoalBox" and not self._sub_tasks[self._current_sub_task]._is_finished:
            self._current_sub_task = "WalkToGoalBox"
            return

        if abs(heading_error(self.ENEMY_GOAL)) < self.HEADING_ERROR:
            self._current_sub_task = "Stand"
            return
        
        self._current_sub_task = "FaceForward"

    def _tick(self, aggressive_goalie = False):
        # Updating the offset values for aggressive goalie
        if aggressive_goalie:
            self._offset_at_min_x = self.AGGRESSIVE_OFFSET_AT_MIN_X
            self._offset_at_max_x = self.AGGRESSIVE_OFFSET_AT_MAX_X
            self._goalie_position_angle = self.AGGRESSIVE_GOALIE_POSITION_ANGLE
        else:
            self._offset_at_min_x = self.DEFAULT_OFFSET_AT_MIN_X
            self._offset_at_max_x = self.DEFAULT_OFFSET_AT_MAX_X
            self._goalie_position_angle = self.DEFAULT_GOALIE_POSITION_ANGLE

        self._target_pos = self._reference_point

        if self._tracking_ball:
            self._target_pos = self._calculate_optimal_position_limited()[0]
        if self._current_sub_task == "FaceForward":
            self._tick_sub_task(turn=self.TURN_RATE if heading_error(self.ENEMY_GOAL) > 0
                else -self.TURN_RATE)
        elif self._current_sub_task == "WalkWithoutTurning":
            x_diff = myPos().x - self._target_pos.x
            y_diff = myPos().y - self._target_pos.y
            
            ball_dist = ball_distance()
            if ball_dist < 1000:
                walk_modifier = self.WALK_MODIFIER_DEFAULT
            else:
                walk_modifier = (ball_dist/500) - 1

            if ball_dist < 4500: # 4500/3000 = 1.5 so reverts back to standard 
                turn_rate = ball_dist/3000
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
                turn=turn_rate if ball_heading() > 0 else -turn_rate
                )
        elif self._current_sub_task == "WalkToGoalBox":
            self._tick_sub_task(self._target_pos)
        else:
            self._tick_sub_task()

    # Calculate position where the goalie's centre dive can cover the whole goal and heading is towards the ball
    def _calculate_cover_point(self):
        # Get the ball's location
        ball_pos = ball_world_pos()
        restricted_ball_pos = ball_pos

        # Restrict the ball to within bounds
        restricted_ball_pos.x = max(ball_pos.x, -HALF_FIELD_LENGTH + self._offset_at_min_x)
        restricted_ball_pos.y = restricted_ball_pos.y if abs(restricted_ball_pos.y) < HALF_FIELD_WIDTH else copysign(1, restricted_ball_pos.y) * HALF_FIELD_WIDTH

        assert isfinite(restricted_ball_pos.x)
        assert isfinite(restricted_ball_pos.y)

        left_post = OUR_LEFT_POST
        right_post = OUR_RIGHT_POST

        # Get the vectors from the left and right post to the ball
        ball_to_left_post = left_post.minus(restricted_ball_pos)
        ball_to_right_post = right_post.minus(restricted_ball_pos)

        # Get angle between the 2 vectors
        ball_goal_angle = ball_to_left_post.thetaTo(ball_to_right_post)
        assert (ball_goal_angle > 0)

        # Formula to calculate the distance from the ball to the chord
        # This is the location that the goalie will be able to interept the ball most effectively
        # The centre dive width covers the ball's available path towards the goal
        distance = GoalieDive.DIVE_CENTRE_WIDTH / (2 * sin(ball_goal_angle/2))

        # The optimal line is a Vector2D that is *distance* away from the ball towards the goal
        optimal_line = restricted_ball_pos.plus(Vector2D(distance, 0).rotate(ball_to_left_post.heading() + ball_goal_angle / 2))
        line_heading = restricted_ball_pos.minus(optimal_line)

        assert isfinite(optimal_line.x)
        assert isfinite(optimal_line.y)
        assert isfinite(line_heading.x)
        assert isfinite(line_heading.y)

        return optimal_line, line_heading
    
    # Calculate the optimal position for the goalie to be based on a defined angle
    def _calculate_optimal_position(self):
        optimal_line, line_heading = self._calculate_cover_point()
        if line_heading.heading() > 0:
            position_adjust_line = Vector2D(-HALF_FIELD_LENGTH, GOAL_POST_ABS_Y)
            position_adjust_line_heading = line_heading.rotated(self._goalie_position_angle)
        else:
            position_adjust_line = Vector2D(-HALF_FIELD_LENGTH, -GOAL_POST_ABS_Y)
            position_adjust_line_heading = line_heading.rotated(-self._goalie_position_angle)

        intersection = intersectLinesPosAndDir(optimal_line, line_heading, position_adjust_line, position_adjust_line_heading)

        return intersection, line_heading

    # Limit the optimal position to be within the defined space
    def _calculate_optimal_position_limited(self):
        optimal_line, line_heading = self._calculate_optimal_position()
        assert isfinite(optimal_line.x)
        assert isfinite(optimal_line.y)

        # Offset away from goal line
        min_x = -HALF_FIELD_LENGTH + self._offset_at_min_x
        # Offset away from the goal from the penalty box line
        max_x = -HALF_FIELD_LENGTH + PENALTY_BOX_LENGTH + self._offset_at_max_x

        position_adjust_line = Vector2D(max_x, 0)
        position_adjust_line_heading = Vector2D(0, 1)

        intersection = optimal_line
        if optimal_line.x > max_x:
            intersection = intersectLinesPosAndDir(optimal_line, line_heading, position_adjust_line, position_adjust_line_heading)
        
        if optimal_line.x < min_x:
            intersection.x = min_x
        
        return intersection, line_heading
