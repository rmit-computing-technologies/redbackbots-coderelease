from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk
from body.skills.WalkToPoint import WalkToPoint
from util.GameStatus import enemy_goal, own_goal, is_ball_between_goal_posts
from util.Global import ball_distance, ball_world_pos, myHeading, myPos, ball_heading, ego_ball_lost
from util.MathUtil import normalisedTheta
from util.Vector2D import Vector2D
from math import degrees, acos, radians, cos, sin
from util.Hysteresis import Hysteresis
from util.Constants import CENTER_CIRCLE_DIAMETER, HALF_FIELD_LENGTH, HALF_FIELD_WIDTH, GOAL_BOX_WIDTH, FIELD_LENGTH, GOAL_POST_DIAMETER
from util import log

class Lineup(BehaviourTask):
    KICKING_FOOT = Hysteresis(-60, 60)
    GOAL_VECTOR = enemy_goal()
    REFERENCE_POINT = own_goal()
    GOAL_KICK_RANGE = GOAL_BOX_WIDTH/2-GOAL_POST_DIAMETER

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Lineup": Walk(self)
        }

    def _reset(self):
        self._current_sub_task = "WalkToPoint"
        self.KICKING_FOOT.set(0)
        self.defensive = False
        # self.kick_with_right_foot = True

    def _transition(self):
        if ball_distance() > 500 and not self.defensive:
            self._current_sub_task = "WalkToPoint"
        else:
            self._current_sub_task = "Lineup"

    def _tick(self, target=None, dribble=False, defensive=False, reference_point=None, avoid_radius=None, max_range=None):
        self.defensive = defensive

        if target is None:
            target = self.GOAL_VECTOR

        if reference_point is None:
            reference_point = self.REFERENCE_POINT

        if max_range is None:
            max_range = self.GOAL_KICK_RANGE

        alignment_vectors = AlignmentVectors(goal_vec=target, dribble=dribble, defensive=self.defensive, avoid_radius=avoid_radius,
                                                reference_point=reference_point, max_range=max_range)

        target = alignment_vectors.get_target()
        move_vector = alignment_vectors.get_move_vector()
        lineup_error = alignment_vectors.get_lineup_error()

        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=target, speed=1)
        else:
            self._tick_sub_task(move_vector.x, move_vector.y, lineup_error, speed=0)

class AlignmentVectors:
    """
    Calculates everything you need to
    KICKING: align the robot to the ball and kick to the goal
    DEFENDING: align the robot between the ball and goal
    Before amending anything here, please see implementation at https://www.desmos.com/geometry/b6vf0hmkwe
    :return:
    move_vector: fine-tuning vector to get the robot to line up with the ball (changes as robot moves)
    ball_to_goal_vec: Unit vector pointing from ball to goal (constant if ball doesn't move)
    """

    CLOSE_TO_BALL_OFFSET = 300
    FOOT_OFFSET = 30
    IDEAL_BALL_OFFSET = 50
    NINETY_DEGREES = 90
    ONE_EIGHTY_DEGREES = 180
    THREE_SIXTY_DEGREES = 360
    FIFTEEN_DEGREES = 15
    FOURTY_FIVE_DEGREES = 45
    ONE_FOURTY_DEGREES = 140
    ONE_FIFTY_DEGRESS = 150

    # Seconds of ball lost before we zero the kicking foot
    RESET_FOOT_BALL_LOST_TIME = 2

    # Max error between predicted ball kick and my heading
    # How sharp can indirect kicks be
    # 30 degrees is a kick that calculates the ball to be kicked at 30
    # degrees to the left or right of the robot
    # 0 degrees is a kick straight forward from the front of the robot
    MAX_KICKABLE_INDIRECT_KICK_ERROR = radians(30)

    # Min and max kicking distances
    MIN_KICKING_DISTANCE = 100
    MAX_KICKING_DISTANCE = 230

    # Smaller value means the range is bigger, means that AT LEAST the range is (ACCURACY*100)% of the max range
    ACCURACY = 0.2

    # Abort kick factor
    # This is used to determine if the robot is too far away from the ball to kick
    # The kickable distance is multiplied by this factor to determine the abort kick distance
    ABORT_KICK_FACTOR = 2

    # Distances and angles for when to lock the kicking foot
    BEHIND_BALL_ANGLE_SCOPE = 60 # degrees for when to lock the kicking foot (30 either side)
    CLOSE_TO_BALL_DIST = 150 # mm for when to lock the kicking foot

    def __init__(self, goal_vec=None, dribble=False, defensive=False, reference_point=None, avoid_radius=None, max_range=None):

        # These are the lower and upper limits to the range of when the multiplier of the move vector kicks in. Uses the distance of the robot to the target
        # IE: when not dribble. Once the robot is 25mm away from target, the move vector that is calculated is unaffected.
        # It scales between 25 and 250, and from 250mm onwards, the move vector is multiplied by 3 - making the robots movements bigger and thus faster
        LOWER_MULTIPLIER_LIMIT = 25 if not dribble else 150
        UPPER_MULTIPLIER_LIMIT = 250 if not dribble else 450

        if goal_vec is None:
            goal_vec = Lineup.GOAL_VECTOR

        if reference_point is None:
            reference_point = Lineup.REFERENCE_POINT

        if max_range is None:
            max_range = Lineup.GOAL_KICK_RANGE

        BALL_WORLD_POS = ball_world_pos()
        BALL_HEADING = ball_heading()
        MY_POS = myPos()
        ENEMY_GOAL = enemy_goal()
        MY_HEADING = myHeading()

        if defensive:
            goal_vec = reference_point # Swap to our goal so we're on defending side
        elif dribble:
            # If we're in between the goal posts, just dribble down the field
            # We don't need it to go towards the center of the goal
            if is_ball_between_goal_posts():
                goal_vec = BALL_WORLD_POS.clone()
                goal_vec.x = goal_vec.x + (ENEMY_GOAL.x*1000)

        ball_dist = MY_POS.distanceTo(BALL_WORLD_POS)

        ball_to_pos_vec = BALL_WORLD_POS.minus(MY_POS).normalise()

        ball_to_goal_vec = goal_vec.minus(BALL_WORLD_POS).normalise()

        dot_product = ball_to_goal_vec.dot_product(ball_to_pos_vec)

        cross_product = ball_to_goal_vec.cross_product(ball_to_pos_vec)

        # If positive, robot is above the ball/goal line
        # 0 means robot is perfectly lined up behind the ball kicking towards the goal
        # -180/180 means the robot is in front of the ball and must now walk back and around
        pos_to_ball_to_goal_angle_abs = degrees(acos(dot_product))
        pos_to_ball_to_goal_angle = pos_to_ball_to_goal_angle_abs if cross_product > 0 else -pos_to_ball_to_goal_angle_abs

        if defensive:
            # If positive, robot is below the ball/goal line
            # 0 means robot is perfectly lined up between the ball and goal
            # -180/180 means the robot is behing the ball and must now walk around
            defensive_pos_to_ball_to_goal_angle_abs = (self.ONE_EIGHTY_DEGREES-pos_to_ball_to_goal_angle_abs)
            pos_to_ball_to_goal_angle = -defensive_pos_to_ball_to_goal_angle_abs if cross_product > 0 else defensive_pos_to_ball_to_goal_angle_abs

        # If we're close to the ball, we want to apply an offset to ensure the robot walks around the ball when in close proximity
        close_to_ball_angle_offset = 0 if ball_dist > self.CLOSE_TO_BALL_OFFSET else ((self.CLOSE_TO_BALL_OFFSET-ball_dist)/100)

        angle_divisor = 1

        if defensive and abs(pos_to_ball_to_goal_angle) > self.FIFTEEN_DEGREES:
            if abs(pos_to_ball_to_goal_angle) > self.ONE_FIFTY_DEGRESS:
                angle_divisor = abs(pos_to_ball_to_goal_angle)/(abs(pos_to_ball_to_goal_angle)-self.ONE_FOURTY_DEGREES)
            else:
                angle_divisor = abs(pos_to_ball_to_goal_angle) * 0.1 # 10% of the angle
        else:
            if abs(pos_to_ball_to_goal_angle) > self.NINETY_DEGREES: # If we're more than 90 degrees off, we want to apply an offset to ensure the robot walks around the ball
                angle_divisor = (abs(pos_to_ball_to_goal_angle)/self.NINETY_DEGREES) + close_to_ball_angle_offset

        angle_offset = -(pos_to_ball_to_goal_angle/angle_divisor)

        cos_theta = cos(radians(pos_to_ball_to_goal_angle+angle_offset))
        sin_theta = sin(radians(pos_to_ball_to_goal_angle+angle_offset))

        rotated_x = (cos_theta*ball_to_goal_vec.x)-(sin_theta*ball_to_goal_vec.y)
        rotated_y = (sin_theta*ball_to_goal_vec.x)+(cos_theta*ball_to_goal_vec.y)

        rotated_vector = Vector2D(rotated_x, rotated_y)

        lineup_offset = self.get_lineup_offset(ball_dist, defensive, avoid_radius, abs(pos_to_ball_to_goal_angle))

        if defensive:
            if abs(MY_POS.x) > HALF_FIELD_LENGTH or abs(MY_POS.y) > HALF_FIELD_WIDTH: # is robot off field?
                dist_off_end_of_field = min(lineup_offset, max(0, abs(BALL_WORLD_POS.x)+lineup_offset-HALF_FIELD_LENGTH))
                y_multiplier = -1 if BALL_WORLD_POS.y > 0 else 1 # minus if were in pos y, etc
                if MY_POS.x > BALL_WORLD_POS.x:
                    # ROBOT BEHIND THE BALL
                    if (BALL_WORLD_POS.x + lineup_offset) > HALF_FIELD_LENGTH: # moving target down towards goal box so robot doesn't walk off field
                        target = Vector2D(BALL_WORLD_POS.x+lineup_offset-dist_off_end_of_field, BALL_WORLD_POS.y+(dist_off_end_of_field*y_multiplier))
                    else:
                        target = Vector2D(BALL_WORLD_POS.x+lineup_offset, BALL_WORLD_POS.y)
                else:
                    # ROBOT IN FRONT OF BALL
                    if (BALL_WORLD_POS.x - lineup_offset) < -HALF_FIELD_LENGTH: # moving target down towards goal box so robot doesn't walk off field
                        target = Vector2D(BALL_WORLD_POS.x-lineup_offset+dist_off_end_of_field, BALL_WORLD_POS.y+(dist_off_end_of_field*y_multiplier))
                    else:
                        target = Vector2D(BALL_WORLD_POS.x-lineup_offset, BALL_WORLD_POS.y)
            else:
                target = BALL_WORLD_POS.plus(rotated_vector.multiply(lineup_offset))
        else:
            target = BALL_WORLD_POS.minus(rotated_vector.multiply(lineup_offset))

        # Foot offset will be 0,0 for dribbling and defense
        foot_offset = Vector2D(0, 0)
        kick_with_right_foot = False

        if Lineup.KICKING_FOOT.is_max():
            kick_with_right_foot = True
            #log.info(msg="right", say=True)
        elif Lineup.KICKING_FOOT.is_min():
            kick_with_right_foot = True
            #log.info(msg="left", say=True)

            # pass
        # else:
        #     if kick_with_right_foot:
        #         log.info(msg="Keep Right", say=True)
        #     else:
        #         log.info(msg="Keep Left", say=True)

        if not dribble and not defensive:
            # Locks in the current foot once we're behind the ball (<30deg either side) and within close distance (300mm), then reset hysteresis
            if (pos_to_ball_to_goal_angle_abs < (self.BEHIND_BALL_ANGLE_SCOPE/2) and ball_dist < self.CLOSE_TO_BALL_DIST) or (ego_ball_lost(self.RESET_FOOT_BALL_LOST_TIME)):
                # Reset to not keep preference for current foot
                if kick_with_right_foot:
                    Lineup.KICKING_FOOT.set(0)
                else:
                    Lineup.KICKING_FOOT.set(-1)
            else:
                if cross_product < 0:
                    # Prefer right foot
                    Lineup.KICKING_FOOT.up()
                else:
                    # Prefer left foot
                    Lineup.KICKING_FOOT.down()

            if kick_with_right_foot:
                foot_offset = Vector2D(0, -self.FOOT_OFFSET)
            else:
                foot_offset = Vector2D(0, self.FOOT_OFFSET)

        # Position of the kicking foot
        foot_pos = MY_POS.minus(foot_offset.rotate(MY_HEADING+radians(self.ONE_EIGHTY_DEGREES)))

        # A point IDEAL_BALL_OFFSET mm in front of the kicking foot
        ideal_ball_pos = foot_pos.plus(Vector2D(self.IDEAL_BALL_OFFSET, 0).rotate(MY_HEADING))

        # How far is the ball from the ideal point in front of the kicking foot
        target_distance = ideal_ball_pos.distanceTo(BALL_WORLD_POS)

        # If we're not behind the ball, we want to walk to the ideal ball position (not considering foot pos)
        if dribble or defensive or abs(pos_to_ball_to_goal_angle) > self.FOURTY_FIVE_DEGREES:
            move_vector = target.minus(MY_POS).rotate(-MY_HEADING)
        else:
            move_vector = target.minus(foot_pos).rotate(-MY_HEADING)

        # Dribble wants to walk into the ball (this is how we `dribble` the ball) - must be within 30deg either side of ball to goal line
        if dribble and pos_to_ball_to_goal_angle_abs < 30:
            move_vector = BALL_WORLD_POS.minus(MY_POS).rotate(-MY_HEADING)

        look_heading = BALL_HEADING

        # Range gets smaller as we get further away from the goals to ensure the kick accounts for the distance slightly
        range = max_range * (1 - ((MY_POS.distanceTo(goal_vec) * self.ACCURACY) / FIELD_LENGTH))

        range_left_edge = Vector2D(goal_vec.x, goal_vec.y + range)
        range_right_edge = Vector2D(goal_vec.x, goal_vec.y - range)

        left_edge_heading = normalisedTheta(range_left_edge.minus(BALL_WORLD_POS).heading())
        right_edge_heading = normalisedTheta(range_right_edge.minus(BALL_WORLD_POS).heading())

        foot_to_ball_heading = normalisedTheta(BALL_WORLD_POS.minus(foot_pos).heading())

        # If pos_to_ball_heading is in between the left and right edge headings
        # Test for figuring out if the robot is in the "kicking zone" TODO: implement in place of pos_to_ball_to_goal_angle_abs < something (?)
        left_edge_pos_error = normalisedTheta(left_edge_heading - foot_to_ball_heading)
        right_edge_pos_error = normalisedTheta(right_edge_heading - foot_to_ball_heading)

        pos_to_kicking_range_diff = min(abs(left_edge_pos_error), abs(right_edge_pos_error))

        if left_edge_pos_error < 0 and right_edge_pos_error > 0:
            pos_to_kicking_range_diff = 0

        multiplier = 3

        # Update multiplier when we're getting close to the ball and within range
        if pos_to_kicking_range_diff < 20 and target_distance < UPPER_MULTIPLIER_LIMIT:
            if target_distance < LOWER_MULTIPLIER_LIMIT:
                multiplier = 1
            elif LOWER_MULTIPLIER_LIMIT <= target_distance <= UPPER_MULTIPLIER_LIMIT:
                multiplier = ((target_distance - LOWER_MULTIPLIER_LIMIT) / ((UPPER_MULTIPLIER_LIMIT - LOWER_MULTIPLIER_LIMIT)/2)) + 1

            if not dribble:
                look_heading = ball_to_goal_vec.heading()

        foot_to_ball_heading = normalisedTheta(BALL_WORLD_POS.minus(foot_pos).heading())

        lineup_error = look_heading - MY_HEADING
        lineup_error = normalisedTheta(lineup_error)

        # Keep looking at the ball UNTIL it is <90 degrees away from the ball to goal line
        # if look heading is not to the ball, and heading error is large, just look at ball.
        if look_heading is not BALL_HEADING and abs(lineup_error) > radians(90):
            look_heading = BALL_HEADING
            lineup_error = look_heading - MY_HEADING
            lineup_error = normalisedTheta(lineup_error)

        # If the robot is turning away from the ball, we want to turn the other way
        # This prevents turning around, away from the ball, even if that is the "shortest"
        # turning direction
        if (BALL_HEADING - MY_HEADING) * lineup_error < 0:
            lineup_error = lineup_error + radians(self.THREE_SIXTY_DEGREES) if lineup_error < 0 else lineup_error - radians(self.THREE_SIXTY_DEGREES)

        move_vector = move_vector.multiply(multiplier)

        self.target = target
        self.move_vector = move_vector
        self.ball_to_goal_vec = ball_to_goal_vec
        self.kick_with_right_foot = kick_with_right_foot
        self.lineup_error = lineup_error
        self.target_distance = target_distance
        self.foot_to_ball_heading = foot_to_ball_heading
        self.left_edge_heading = left_edge_heading
        self.right_edge_heading = right_edge_heading

        # Calculate kickable_distance and kick_in_target_range

        # How far the ball can be from the foot to be kickable
        # First, find the error in the heading of the foot to the ball versus the heading of the
        # robot i.e. How different is the predicted ball kick heading from facing forward and
        # kicking forward
        # Second, interpolate the kickable distance based on the error and the max kickable error
        # for indirect kicks and the min and max kicking distances
        # See https://www.desmos.com/calculator/npsx92five for further details
        self.kickable_distance = self.MAX_KICKING_DISTANCE - (
            ((self.MAX_KICKING_DISTANCE - self.MIN_KICKING_DISTANCE) / self.MAX_KICKABLE_INDIRECT_KICK_ERROR)
            * abs(foot_to_ball_heading - MY_HEADING)
        )
        self.kick_in_target_range = left_edge_heading > foot_to_ball_heading > right_edge_heading

        self.abort_kick_distance = self.kickable_distance * self.ABORT_KICK_FACTOR

    def get_lineup_offset(self, ball_dist, defensive, avoid_radius=None, pos_to_ball_to_goal_angle_abs=None):
        BUFFER = 150
        OFFSET_MAX = 360
        OFFSET_MIN = 180

        if defensive:
            if avoid_radius is None:
                return CENTER_CIRCLE_DIAMETER/2 + BUFFER
            else:
                return avoid_radius

        # If we're outside of 60 degrees either side (ie: not behind the ball), we want to apply an offset to ensure the robot walks around the ball
        if pos_to_ball_to_goal_angle_abs > 60:
            return min(max(ball_dist/3 * (1 + pos_to_ball_to_goal_angle_abs/self.ONE_EIGHTY_DEGREES), OFFSET_MIN), OFFSET_MAX)
        else:
            return min(max(ball_dist/3, OFFSET_MIN), OFFSET_MAX)

    def get_target(self) -> Vector2D:
        """
        Returns the target point we need to walk to. This is the point behind the ball for kicking, or in front of for defending.
        Changes as robot moves
        """
        return self.target

    def get_move_vector(self) -> Vector2D:
        """
        This is the move vector (forward, left) to get the robot to the target and facing the right way.
        Usually used in conjunction with lineup error to ensure the robot is facing the right way.
        Changes as robot moves
        """
        return self.move_vector

    def get_ball_to_goal_vec(self) -> Vector2D:
        """
        Unit vector pointing from ball to goal - constant if ball doesn't move
        """
        return self.ball_to_goal_vec

    def get_kick_with_right_foot(self) -> bool:
        """
        Returns if robot should kick with right foot. False is to use left foot
        """
        return self.kick_with_right_foot

    def get_lineup_error(self) -> float:
        """
        Calculate how far off the robot is from the target heading.
        Changes as robot moves
        """
        return self.lineup_error

    def get_target_distance(self) -> float:
        """
        Returns how far away the robot is from the target point. This is the point behind the ball for kicking, or in front of for defending
        """
        return self.target_distance

    def get_foot_to_ball_heading(self) -> float:
        """
        Returns the heading from the foot to the ball.
        This heading is used to ensure the foot is behind the ball, and once kicked, will kick between the left/right edge headings
        """
        return self.foot_to_ball_heading

    def get_left_edge_heading(self) -> float:
        """
        Returns the heading of the left edge of the kicking range
        Constant if ball doesn't move
        """
        return self.left_edge_heading

    def get_right_edge_heading(self) -> float:
        """
        Returns the heading of the right edge of the kicking range
        Constant if ball doesn't move
        """
        return self.right_edge_heading

    def get_kickable_distance(self) -> float:
        """
        Returns the calculated kickable distance based on the heading difference between the ball and the robot.
        This value is used to determine if the ball is within a kickable range.
        """
        return self.kickable_distance

    def get_abort_kick_distance(self) -> float:
        """
        Returns the distance to abort a kick. This is used to determine if the robot is too far away from the ball to kick.
        """
        return self.abort_kick_distance

    def get_kick_in_target_range(self) -> bool:
        """
        Returns whether the estimated kick trajectory is between the evaluated left and right edge headings.
        This boolean indicates if the ball expected to be kicked within the target range.
        """
        return self.kick_in_target_range