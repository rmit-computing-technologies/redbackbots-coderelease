from math import radians
from util import log

from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk
from util.Global import myPos, myHeading, close_to_position, not_close_to_position, myHeading
from util.Constants import FIELD_LENGTH, FIELD_WIDTH, CENTER_CIRCLE_DIAMETER
from util.MathUtil import normalisedTheta
from util.ObstacleAvoidance import walk_vec_with_avoidance
from util.GameStatus import we_are_kicking_team, in_kick_in, in_penalty_kick, in_corner_kick
from util.Vector2D import Vector2D
from util.Constants import FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET
import robot

class WalkToPoint(BehaviourTask):
    """
    Description:
    A skill associated with walking to a global point on the field.
    Hading error is the angle between the robot's heading and the vector to
    the target.

    If heading error is large, robot will first purely turn, to face
    the final_pos. If heading error is moderate, robot walks forwards,
    adjusting the turn depending on the error, correcting over one second.
    If heading error is small, robot will walk purely forwards, to reach
    a walk speed as fast as possible, and as stable as possible.
    """

    WALK_SPEED = 300  # mm / s
    TURN_RATE = 1.5  # rad / s

    # Percentage of the green space around the field the robot is allowed to walk in.
    # For SPL field, ~400mm, for home, ~350 (Depends on if X or Y, and SPLDefs.hpp)
    # Smaller: Robot will not move as far from the field when localised and instructed to do so
    # Larger: Robot will move further from the field when localised and instructed to do so
    POS_BUFFER = 0.5

    # X and Y distance beyond the marked field the robot can walk
    X_POS_BUFFER = POS_BUFFER * FIELD_LENGTH_OFFSET
    Y_POS_BUFFER = POS_BUFFER * FIELD_WIDTH_OFFSET

    HEADING_BUFFER = 10 # Deg

    HEADING_ERROR_TO_ONLY_TURN = radians(40)  # rad
    HEADING_ERROR_TO_ADJUST = radians(15)  # rad

    HEADING_LEFT_SMALL = radians(-(90-HEADING_BUFFER))
    HEADING_LEFT_LARGE = radians(-(90+HEADING_BUFFER))

    HEADING_RIGHT_SMALL = radians(90-HEADING_BUFFER)
    HEADING_RIGHT_LARGE = radians(90+HEADING_BUFFER)

    HEADING_BEHIND_SMALL = radians(150) #rad
    HEADING_BEHIND_LARGE = radians(180) #rad

    TIME_TO_FIX_HEADING = 1.0  # how much time we allow to align heading (s)

    MAX_POS_X = (FIELD_LENGTH/2)+X_POS_BUFFER
    MAX_NEG_X = -MAX_POS_X

    MAX_POS_Y = (FIELD_WIDTH/2)+Y_POS_BUFFER
    MAX_NEG_Y = -MAX_POS_Y

    QUARTER_CENTER_CIRCLE = CENTER_CIRCLE_DIAMETER / 4.0
    
    FINAL_HEADING_ERROR = radians(10)
    
    FINAL_ALWAYS_LOOK_ERROR = radians(20)
    
    FINAL_HEADING_DISTANCE = 150

    DISTANCE_TO_TURN_TO_FINAL = 200
    
    LOWER_MULTIPLIER_LIMIT = 25
    UPPER_MULTIPLIER_LIMIT = 250
    DEFAULT_MULTIPLIER = 3

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self)
        }

    def _reset(self):
        self._current_sub_task = "Walk"
        self._heading_close = False
        self._position_close = False
        self._final_heading = None
        self._always_look_at_final = False

    def _tick(self,
              final_pos=Vector2D(0, 0),
              final_heading=None,
              always_look_at_final=False,
              use_avoidance=True,
              speed=1.0,
              prevent_leaving_field=True,
              walk_backwards=False,
              walk_sideways=True
            ):
        self._look_at_final = self._final_heading is not None or always_look_at_final
        self._final_heading = final_heading if final_heading is not None else final_pos.minus(myPos()).heading()
        self._always_look_at_final = always_look_at_final
        if self._always_look_at_final:
            log.info("look final", say=False)
        
        self._my_pos_to_final_pos = final_pos.minus(myPos())
        self._dist_to_final_pos = final_pos.distanceTo(myPos())

        # StateEstimation for the walkToPoint
        # This is used to send information to offnao for debugging purposes
        self.world.blackboard.stateEstimation.walkToPoint = robot.AbsCoord(final_pos.x,final_pos.y,0)

        if prevent_leaving_field and self._current_sub_task == "Walk":
            # If the ball is fully off the field this sets the ball position to a point just beyond the field boundary
            if (final_pos.x < self.MAX_NEG_X or final_pos.x > self.MAX_POS_X):
                final_pos.x = self.MAX_NEG_X if final_pos.x < 0 else self.MAX_POS_X
                log.debug("WalkToPoint: Ball has left the field on the X axis")

            if (final_pos.y < self.MAX_NEG_Y or final_pos.y > self.MAX_POS_Y):
                final_pos.y = self.MAX_NEG_Y if final_pos.y < 0 else self.MAX_POS_Y
                log.debug("WalkToPoint: Ball has left the field on the Y axis")

            if not (we_are_kicking_team() and (in_kick_in() or in_penalty_kick() or in_corner_kick())):
                # Used to prevent the robot from leaving the field while chasing the ball
                # NOTE: There is nothing to prevent the robot leaving the field if it's kicking in
                if (myPos().x < self.MAX_NEG_X or myPos().x > self.MAX_POS_X):
                    final_pos.x = self.MAX_NEG_X if final_pos.x < 0 else self.MAX_POS_X
                    log.debug("Robot is leaving the field (x), not for kick in")

                if (myPos().y < self.MAX_NEG_Y or myPos().y > self.MAX_POS_Y):
                    final_pos.y = self.MAX_NEG_Y if final_pos.y < 0 else self.MAX_POS_Y
                    log.debug("Robot is leaving the field (y), not for kick in")

        if (not self._position_close and close_to_position(final_pos)):
            self._position_close = True
        elif (self._position_close and not_close_to_position(final_pos)):
            self._position_close = False

        if self._position_close:
            forward = 0
            left = 0
            turn = 0
        elif always_look_at_final or (self._dist_to_final_pos < self.DISTANCE_TO_TURN_TO_FINAL and final_heading is not None):
            move_vector = final_pos.minus(myPos()).rotate(-myHeading())

            multiplier = self.DEFAULT_MULTIPLIER

            # Update multiplier when we're getting close to the ball and within range
            if self._dist_to_final_pos < self.UPPER_MULTIPLIER_LIMIT:
                if self._dist_to_final_pos < self.LOWER_MULTIPLIER_LIMIT:
                    multiplier = 1
                elif self.LOWER_MULTIPLIER_LIMIT <= self._dist_to_final_pos <= self.UPPER_MULTIPLIER_LIMIT:
                    multiplier = ((self._dist_to_final_pos - self.LOWER_MULTIPLIER_LIMIT) / ((self.UPPER_MULTIPLIER_LIMIT - self.LOWER_MULTIPLIER_LIMIT)/2)) + 0.2

            move_vector = move_vector.scale(multiplier)

            forward = move_vector.x
            left = move_vector.y
            
            log.debug(f"WalkToPoint: forward={forward}, left={left}, multiplier={multiplier}")

            # Turn towards the target heading
            turn = (self.TURN_RATE / 2) if self._heading_error() > self.FINAL_HEADING_ERROR else -(self.TURN_RATE / 2)
        elif self._dist_to_final_pos < self.QUARTER_CENTER_CIRCLE and abs(self._heading_error()) > self.HEADING_BEHIND_SMALL and abs(self._heading_error()) <= self.HEADING_BEHIND_LARGE and walk_backwards:
            forward = -150
            left = 0
            turn = 0
        elif self._dist_to_final_pos < self.QUARTER_CENTER_CIRCLE and self._heading_error() > self.HEADING_LEFT_SMALL and self._heading_error() <= self.HEADING_LEFT_LARGE and walk_sideways:
            forward = 0
            left = 200
            turn = 0
        elif self._dist_to_final_pos < self.QUARTER_CENTER_CIRCLE and self._heading_error() > self.HEADING_RIGHT_SMALL and self._heading_error() <= self.HEADING_RIGHT_LARGE and walk_sideways:
            forward = 0
            left = -200
            turn = 0
        elif abs(self._heading_error()) > self.HEADING_ERROR_TO_ONLY_TURN:
            # If heading is very off, just turn, walking forwards with a big
            # turn is very unstable
            forward = 0
            left = 0
            turn = (self.TURN_RATE if self._heading_error() > 0
                    else -self.TURN_RATE)
        else:
            walk_vector = Vector2D(self.WALK_SPEED, 0)

            # Slow down if we're close to the final position
            # (to prevent overshooting)
            if self._pos_error_sq() < 200 ** 2:
                walk_vector.scale(0.5)

            if use_avoidance:
                walk_vector = walk_vec_with_avoidance(walk_vector)

            forward = walk_vector.x
            left = walk_vector.y

            if  (self._look_at_final and abs(self._heading_error()) > self.FINAL_HEADING_ERROR):
                # If we are looking at the final position, and the heading error is large,
                # we want to correct the heading error over an amount of time
                turn = self._heading_error() / self.TIME_TO_FIX_HEADING
            elif abs(self._heading_error()) > self.HEADING_ERROR_TO_ADJUST:
                # Aim to correct the heading error over an amount of time
                turn = self._heading_error() / self.TIME_TO_FIX_HEADING
            else:
                # If the error is small, just keep walking straight,
                # constantly changing turn causes instability
                turn = 0

        self._tick_sub_task(forward, left, turn, speed=speed)

    def _heading_error(self):
        
        if (self._dist_to_final_pos < self.DISTANCE_TO_TURN_TO_FINAL and self._look_at_final) or self._always_look_at_final:
            # log.debug("looking at final position")
            return normalisedTheta(
                self._final_heading - myHeading())
        else:
            # log.debug("face walk")
            return normalisedTheta(
                self._my_pos_to_final_pos.heading() - myHeading())


    def _pos_error_sq(self):
        return self._my_pos_to_final_pos.length2()
    
def calculate_heading_smoothness(heading_error: float):
    """
    Calculates a smooth turn rate that decreases as the heading error approaches zero.
    """
    # Use a scaling factor so that turn rate is high for large errors, low for small errors
    max_turn = WalkToPoint.TURN_RATE
    min_turn = 0.5  # Minimum turn rate to avoid stalling

    # Normalize error to [0, 1] based on a reasonable max error (e.g., 90 deg)
    normalized_error = min(abs(heading_error) / radians(90), 1.0)
    # Use a non-linear scaling (e.g., quadratic) for smoother slowdown
    turn_rate = min_turn + (max_turn - min_turn) * (normalized_error ** 2)
    # Preserve direction
    return turn_rate if heading_error >= 0 else 0