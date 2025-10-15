from math import radians, degrees, sin, cos, pi

#TODO: fix the case (make snake_case) in the Constants and Global
import util.led_override as led_override
from BehaviourTask import BehaviourTask
from util.BehaviourHierarchy import get_behaviour_hierarchy
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from head.HeadTrackBall import HeadTrackBall
from head.HeadCentre import HeadCentre
from util.Constants import (
    LEDColour,
    LEDSegments,
    LEDEyePatterns,
    HALF_FIELD_WIDTH,
    FIELD_LENGTH,
    HALF_FIELD_LENGTH
)
from util.Global import (
    myPosUncertainty,
    ego_ball_lost,
    num_balls_seen,
    ego_see_ball,
    time_since_last_team_ball_update,
    ball_distance,
    ball_heading,
    myPos,
    myHeading,
    look_target,
    team_ball_world_pos,
    ego_ball_world_pos,
    team_ball_heading,
    usingGameSkill
)
from util.GameStatus import (
    penalised,
    in_finished,
    in_ready,
    in_set,
    in_initial,
    in_standby
)
from util.Hysteresis import Hysteresis
from util.Timer import Timer, WallTimer
from util.Vector2D import Vector2D
from util import log

# For reading current yaw
from robot import Joints
from util.Sensors import angles

class HeadAware(BehaviourTask):
    """
    Description:
    A Headskill associated with localising, and finding the ball (aware of surroundings)
    If position confidence is low, we do a wide scan to try localise (position of the robot).
    If position confidence is high, we do a narrow scan to try find the ball.
    If we find the ball, we stare at it.
    
    Now uses continuous sinusoidal movement for smooth head sweeping.
    """

    # Dictionary of sequences from str to dictionary of scope, base_frequency and speed_multiplier
    _sequences = {
        "Wide": {
            "scope": 90,           # degrees of total far left to far right movement
            "speed_multiplier": 0.7,  # Speed multiplier for the sequence
            # Uses default speed range
        },
        "SlowWide": {
            "scope": 80,
            "speed_multiplier": 0.4,
            "slow_at_edges": True,  # Flip direction of sweep to be slow at edges
        },
        "FastWide": {
            "scope": 90,
            "speed_multiplier": 1.2,
        },
        "Medium": {
            "scope": 70,
            # Uses default speed range
        },
        "Narrow": {
            "scope": 30,
            "speed_multiplier": 0.5,
        },
        "Tiny": {
            "scope": 20,
            "speed_multiplier": 0.5,
        }
    }
    
    # Sequence info for targeted find ball based on the context of the ball (is it a team ball, or goal kick, or corner kick, etc.)
    _targeted_find_ball_sequence = {
        "JustLostBall": {
            "growth": 10, # How much to grow the scope by each second we don't find the ball
            "initial_scope": 10, # Initial scope to look for the ball
            "max_scope": 50,
            "speed_multiplier": 0.4, # Speed multiplier for the sequence
        },
        "TeamBall": {
            "growth": 20, # How much to grow the scope by each second we don't find the ball
            "initial_scope": 20, # Initial scope to look for the ball
            "max_scope": 100,
            "speed_multiplier": 0.6
        },
        "GoalKick": {
            "growth": 10, # How much to grow the scope by each second we don't find the ball
            "initial_scope": 10, # Initial scope to look for the ball
            "max_scope": 40,
            # Start by looking at the closest goal box corner to us
        },
        "CornerKick": {
            "growth": 10, # How much to grow the scope by each second we don't find the ball
            "initial_scope": 10, # Initial scope to look for the ball
            "max_scope": 120,
            # Start by looking at the cloest kicking corner to us
        },
        "MissLocalisedTrackBall": {
            "growth": 5, # How much to grow the scope by each second we don't find the ball
            "initial_scope": 30, # Initial scope to look for the ball
            "max_scope": 60,
            "speed_multiplier": 1
            # Start by looking at the cloest kicking corner to us
        }
    }
    
    EGO_SEE_BALL_FRAMES = 2 # Number of frames to start staring at the ball when we see it
    JUST_LOST_BALL_SECONDS = 4 # Time to look at the last known position of the ball after we lose it
    TEAM_SEE_BALL_SECONDS = 4 # Age of teamball (in secs) to consider the team ball as seen

    DEFAULT_SPEED = 0.25 # default speed for head movement was 0.25
    SHOULDER_SPEED = 0.3 # When we are looking at our shoulder we speed up
    STARE_SECONDS = 0.20 # Time to stare in between positions was 0.4
    PRE_LOOK_LOCALISE_SECONDS = 8  # Localise duration in standby
    LOOKING_AT_REF_PITCH = -0.1    # Value to tilt head up for ref
    PITCH_DEG = 19
    PITCH = radians(PITCH_DEG) # TODO: Make dynamic, to both look up and down as we sweep across,
                        #       but also use pitch to look where the ball WAS when we lose it
    REF_ANGLE_DOWNSIZE_MULTIPLIER = 1 # Used to shrink calculated angle so ref T-junction stays centered and visible
    REF_T_JUNCTION = Vector2D(0,HALF_FIELD_WIDTH)
    REF_ANGLE_DOWNSIZE_MULTIPLIER = 1 # Used to shrink calculated angle so ref T-junction stays centered and visible
    REF_T_JUNCTION = Vector2D(0,HALF_FIELD_WIDTH)
    MAX_UNCERTAINTY = 130_000 # was 100_000 and most recently was 130_000, 70_000 was the comp value

    high_uncertainty = Hysteresis(min_value=0, max_value=150) # was 50 and at home was 300

    LOOKING_AT_SHOULDER_ANGLE = 65 # Angle at which we increase speed to avoid spending too much 
                                   # time on the edge of our vision

    MAX_BALL_SEEN_DISTANCE = FIELD_LENGTH
    MAX_TEAM_BALL_SEEN_DISTANCE = HALF_FIELD_LENGTH
    
    CLOSE_ANGLE = radians(3) # Angle at which we declare we are "close enough" to the target scope
                             # range to start turning the other way

    STARE_SCOPE = radians(20) # If the scope is less than this, we will stare at the target instead of sweeping

    # Speed range parameters
    DEFAULT_MIN_SPEED = 0.08
    DEFAULT_MAX_SPEED = 0.18

    # Find ball parameters
    DEFAULT_GROWTH = 10  # How much to grow the scope by each second we don't find the ball
    DEFAULT_INITIAL_SCOPE = 20  # Initial scope to look for the ball
    DEFAULT_MAX_SCOPE = 70  # Maximum scope to look for the ball
    
    # Amount of movement required to be considered "walking"
    WALKING_MOVEMENT_THRESHOLD = 2

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walking": HeadFixedYawAndPitch(self),
            "Localise": HeadFixedYawAndPitch(self),
            "FindBall": HeadFixedYawAndPitch(self),
            "TargetedFindBall": HeadFixedYawAndPitch(self),
            "TrackBall": HeadTrackBall(self),
            "Penalised": HeadCentre(self),
            "LookAtRef" : HeadFixedYawAndPitch(self)
        }

    def _transition(self):
        if penalised() or in_finished():
            self._current_sub_task = "Penalised"
            return

        if in_standby():
            if not (self._pre_look_localise_timer.running):
                self._pre_look_localise_timer.start().restart()
            if self._pre_look_localise_timer.finished():
                self._current_sub_task = "LookAtRef"
            else:
                self._current_sub_task = "Localise"
            return

        if myPosUncertainty() > self.MAX_UNCERTAINTY:
            HeadAware.high_uncertainty.up()
        else:
            HeadAware.high_uncertainty.reset()
            
        # if (HeadAware.high_uncertainty.is_max()):
            # log.debug("lost", say=True)

        forward_diff = abs(self.world.blackboard.motion.odometry.forward - self._prev_forward)
        left_diff = abs(self.world.blackboard.motion.odometry.left - self._prev_left)
        # take the turn difference in tenths of a radian to avoid floating point issues
        turn_diff = abs(int(self.world.blackboard.motion.odometry.turn * 10) - int(self._prev_turn * 10))

        self._prev_forward = self.world.blackboard.motion.odometry.forward
        self._prev_left = self.world.blackboard.motion.odometry.left
        self._prev_turn = self.world.blackboard.motion.odometry.turn

        walking = left_diff + forward_diff + turn_diff > self.WALKING_MOVEMENT_THRESHOLD

        # Only look for a ball if we are not in ready state or initial or standby or finished
        # Check for ball tracking first (highest priority when ball is seen)
        if (ego_see_ball(self.EGO_SEE_BALL_FRAMES)
            and ball_distance() < self.MAX_BALL_SEEN_DISTANCE
            and self.is_yaw_in_scope(ball_heading())
            and HeadAware.in_find_ball_game_state()):

            if HeadAware.high_uncertainty.is_max():
                self._current_sub_task = "TargetedFindBall"
                self._current_find_ball_sequence = "MissLocalisedTrackBall"
                led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.red)

            else:
                self._current_sub_task = "TrackBall"
                led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.red)


        # If we just lost the ball, we will look at the last known position of the ball
        elif (not ego_ball_lost(self.JUST_LOST_BALL_SECONDS)
              and ball_distance() < self.MAX_TEAM_BALL_SEEN_DISTANCE
              and HeadAware.in_find_ball_game_state()):
            self._look_target = ego_ball_world_pos()
            self._current_sub_task = "TargetedFindBall"
            self._current_find_ball_sequence = "JustLostBall"
            
            self._current_camera = self._sub_tasks["TrackBall"].get_prev_camera()  # Use the last camera used for head tracking
            
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.blue)

        # Check for specific look targets (manual overrides and lost ball tracking)
        elif (look_target() is not None
              and HeadAware.in_find_ball_game_state()):
            self._look_target = look_target()  # Get the current look target from the global state
            self._current_sub_task = "TargetedFindBall"
            self._current_find_ball_sequence = "TeamBall"  # Default to corner kick sequence TODO: Make this dynamic or input from global
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.purple)

        # If the team sees the ball, we will look at the team ball position
        elif ((time_since_last_team_ball_update() < self.TEAM_SEE_BALL_SECONDS)
              and self.is_yaw_in_scope(team_ball_heading()) 
              and HeadAware.in_find_ball_game_state()):
            self._look_target = team_ball_world_pos()
            self._current_sub_task = "TargetedFindBall"
            self._current_find_ball_sequence = "TeamBall"
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.green)

        # Check for localisation needs (high uncertainty or we should localise based on game state)
        elif (HeadAware.high_uncertainty.is_max()):
            self._current_sub_task = "Localise"
            led_override.override_eye_pattern(led_override.RIGHT_EYE, LEDEyePatterns.localise)
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.dim_white)
            
            # Check for walking (movement-specific behavior)
        elif walking:
            self._current_sub_task = "Walking"
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.yellow)
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.dim_white)

            # Default fallback to localisation
        else:
            self._current_sub_task = "Localise"
            led_override.override_eye_pattern(led_override.RIGHT_EYE, LEDEyePatterns.localise)
            led_override.override_eye_segment(led_override.LEFT_EYE, led_override.HEAD_TRACK_SEGMENTS, LEDColour.dim_white)

        # Decide the scope
        if in_initial() or in_ready() or in_set():
            self._current_sequence = "Wide"
        elif self._current_sub_task == "FindBall":
            self._current_sequence = "Wide"
        elif self._current_sub_task == "Localise":
            self._current_sequence = "SlowWide"
        elif self._current_sub_task == "Walking":
            self._current_sequence = "Narrow"
        else:
            self._current_sequence = "Medium"

    def _reset(self):
        self.PITCH = radians(self.PITCH_DEG + self.world.blackboard.kinematics.parameters.cameraPitchBottom)

        self._current_sub_task = "FindBall"
        self._current_sequence = "Wide"
        self._current_find_ball_sequence = "JustLostBall"  # Default to ego ball sequence

        HeadAware.high_uncertainty.resetMax()
        self._timer_since_start = WallTimer()
        self._targeted_find_ball_timer = WallTimer()
        
        # Sinusoidal movement state
        self._phase = 0.0  # Current phase in the sinusoidal cycle
        self._yaw_target = 0.0
        self._current_speed = self.DEFAULT_SPEED
        self._sweep_direction = 1 # 1 for right, -1 for left
        self._current_camera = HeadTrackBall.BOTTOM_CAMERA
        self._prev_sweep_direction = 0  # Initial invalid prev sweep direction to detect if we need to update
        
        self._prev_forward = self.world.blackboard.motion.odometry.forward
        self._prev_left = self.world.blackboard.motion.odometry.left
        self._prev_turn = self.world.blackboard.motion.odometry.turn
        self._pre_look_localise_timer = Timer(target_seconds = self.PRE_LOOK_LOCALISE_SECONDS) # localise for the first 3 seconds in standby
        
        self._look_target = None  # Reset the look target
        self._prev_scope = 0

    def _tick(self):
        if self._current_sub_task != "TargetedFindBall":
            self._targeted_find_ball_timer.restart()
            self._prev_sweep_direction = 0  # Reset when not in targeted find ball to know we can reset

        if self._current_sub_task == "TrackBall" or self._current_sub_task == "Penalised":
            # No head movement required
            self._tick_sub_task()
            return
        
        if self._current_sub_task == "LookAtRef" :
            # Look at the ref's T-Junction based on our position from localisation
            # Robots on the same side as the ref will look at opposite T-Junction (GC location)
            opposite_t_junction = Vector2D(0,HALF_FIELD_WIDTH) if myPos().y < 0 else Vector2D(0,-HALF_FIELD_WIDTH)

            self._tick_sub_task(yaw = self.look_at_yaw(opposite_t_junction), pitch = self.LOOKING_AT_REF_PITCH, pitch_speed=0.1, yaw_speed=0.1)
            return



        if self._current_sub_task == "TargetedFindBall":
            sequence_params = self._targeted_find_ball_sequence[self._current_find_ball_sequence]
            initial_scope = sequence_params.get("initial_scope", self.DEFAULT_INITIAL_SCOPE)
            max_scope = sequence_params.get("max_scope", self.DEFAULT_MAX_SCOPE)
            growth = sequence_params.get("growth", self.DEFAULT_GROWTH)
            
            # Use the growth parameter to expand scope over time
            elapsed_time = self._targeted_find_ball_timer.elapsedSeconds()
            scope = min(initial_scope + (growth * elapsed_time), max_scope)
            
            
            # Center the sweep around the last known ball heading
            center_yaw = self.look_at_yaw(self._look_target)

            # If the scope is too small to sweep, just look at the center
            if scope < self.STARE_SCOPE:
                self._yaw_target = center_yaw
            else:
                # Determine speed range for the current sequence
                max_speed, min_speed = self.get_speeds(sequence_params)

                curr_yaw = angles(self.world.blackboard)[Joints.HeadYaw]

                # Check if we need to reverse direction
                if self._sweep_direction == 1 and curr_yaw >= (self.limit_to_head_scope(self._yaw_target) - self.CLOSE_ANGLE) or self._prev_sweep_direction == 0:
                    self._sweep_direction = -1
                elif self._sweep_direction == -1 and curr_yaw <= (self.limit_to_head_scope(self._yaw_target) + self.CLOSE_ANGLE):
                    self._sweep_direction = 1

                # Only update yaw target when sweep direction changes or this is the first tick
                if self._sweep_direction != self._prev_sweep_direction or self._prev_sweep_direction == 0:
                    self._prev_scope = scope
                    self._prev_sweep_direction = self._sweep_direction
                
                half_scope_rad = radians(self._prev_scope / 2)
                self._yaw_target = self.limit_to_head_scope(center_yaw + self._sweep_direction * half_scope_rad)

                # Calculate speed
                normalized_pos = min(1.0, abs(curr_yaw - center_yaw) / half_scope_rad) if half_scope_rad > 0 else 0
                if sequence_params.get("slow_at_edges", False):
                    # Fast in middle, slow at edges
                    speed_factor = cos(normalized_pos * pi / 2.0)
                else:
                    # Slow in middle, fast at edges
                    speed_factor = sin(normalized_pos * pi / 2.0)
                speed_range = max_speed - min_speed
                self._current_speed = min_speed + (speed_factor * speed_range)

            # self.log_sweep_info()

            target_dist = myPos().distanceTo(self._look_target)
            pitch_deg, self._current_camera = HeadTrackBall.get_pitch_and_camera_from_distance(target_dist, self._current_camera)
            pitch = radians(pitch_deg)

            self._tick_sub_task(
                yaw=self._yaw_target,
                pitch=pitch,
                yaw_speed=self._current_speed,
                pitch_speed=0.5)
            return

        # Get current sequence parameters
        sequence_params = self._sequences[self._current_sequence]
        scope = sequence_params["scope"]
        half_scope_rad = radians(scope / 2)


        # Determine speed range for the current sequence
        min_speed, max_speed = self.get_speeds(sequence_params)

        # Get current head yaw
        curr_yaw = angles(self.world.blackboard)[Joints.HeadYaw]

        # Reverse direction when close to edges
        if self._sweep_direction == 1 and curr_yaw >= (half_scope_rad - self.CLOSE_ANGLE):
            self._sweep_direction = -1
        elif self._sweep_direction == -1 and curr_yaw <= (-half_scope_rad + self.CLOSE_ANGLE):
            self._sweep_direction = 1
        
        self._yaw_target = self._sweep_direction * half_scope_rad

        # Calculate speed: slowest in middle, fastest at edges, smoothed with sin
        normalized_pos = min(1.0, abs(curr_yaw) / half_scope_rad) if half_scope_rad > 0 else 0
        if sequence_params.get("slow_at_edges", False):
            # Fast in middle, slow at edges
            speed_factor = cos(normalized_pos * pi / 2.0)
        else:
            # Slow in middle, fast at edges
            speed_factor = sin(normalized_pos * pi / 2.0)

        speed_range = max_speed - min_speed
        self._current_speed = min_speed + (speed_factor * speed_range)

        # self.log_sweep_info()
        
        # Limit yaw target to head scope (just in case)
        self._yaw_target = HeadAware.limit_to_head_scope(self._yaw_target)

        # Execute the movement
        self._tick_sub_task(
            yaw=self._yaw_target,
            pitch=self.PITCH,
            yaw_speed=self._current_speed,
            pitch_speed=0.05)


    def get_speeds(self, sequence_params=None):
        """
        Returns the current speed and yaw target for the head movement.
        
        Args:
            sequence_params (dict): Parameters for the sequence to adjust speed.
        
        Returns:
            tuple: (min_speed, max_speed)
        """
        if sequence_params is None:
            sequence_params = self._sequences[self._current_sequence]
        
        if "min_speed" in sequence_params and "max_speed" in sequence_params:
            min_speed = sequence_params["min_speed"]
            max_speed = sequence_params["max_speed"]
        elif "speed_multiplier" in sequence_params:
            min_speed = self.DEFAULT_MIN_SPEED * sequence_params["speed_multiplier"]
            max_speed = self.DEFAULT_MAX_SPEED * sequence_params["speed_multiplier"]
        else:
            min_speed = self.DEFAULT_MIN_SPEED
            max_speed = self.DEFAULT_MAX_SPEED

        return min_speed, max_speed

    def looking_at_shoulder(self):
        currAngles = angles(self.world.blackboard)
        curr_yaw = currAngles[Joints.HeadYaw]
        return(abs(curr_yaw) > radians(HeadAware.LOOKING_AT_SHOULDER_ANGLE))

    def log_sweep_info(self):
        """
        Logs the current sweep information for debugging/monitoring.
        """
        # Get current head yaw
        curr_yaw = angles(self.world.blackboard)[Joints.HeadYaw]
        
        log.debug(f"Sweep Info: Yaw Target (degrees): {degrees(self._yaw_target)}, " +
                    f"Current Yaw (degrees): {degrees(curr_yaw)}, " +
                    f"Current Speed: {self._current_speed}, " +
                    f"Sweep Direction: {self._sweep_direction}" +
                    (f", Find Ball Sequence: {self._current_find_ball_sequence}, " +
                     f"Look Target: {self._look_target}, " +
                     f"Elapsed Time: {self._targeted_find_ball_timer.elapsedSeconds():.2f}s" 
                     if self._current_sub_task == "TargetedFindBall" else f"Sequence: {self._current_sequence}, "), say=False)

    @staticmethod
    def in_find_ball_game_state() -> bool:
        """
        Checks if the robot is in a game state where it should be looking for the ball.
        
        Returns:
            bool: True if in a game state that requires finding the ball, False otherwise.
        """
        return not (in_ready() or in_initial() or in_standby() or in_finished()) or not usingGameSkill()

    @staticmethod
    def look_at_yaw(target=ball_heading()) -> float:
        """
        Calculates the yaw angle to look at a target position.
        
        Args:
            target (Vector2D): The target position to look at.
        
        Returns:
            float: The yaw angle in radians.
        """
        return HeadAware.limit_to_head_scope(myPos().headingTo(target) - myHeading())

    @staticmethod
    def is_yaw_in_scope(yaw: float) -> bool:
        """
        Checks if the given yaw angle is within the maximum yaw range.
        
        Args:
            yaw (float): The yaw angle to check.
        
        Returns:
            bool: True if within range, False otherwise.
        """
        return abs(yaw) <= HeadTrackBall.MAX_YAW

    @staticmethod
    def limit_to_head_scope(yaw: float) -> float:
        """
        Limits the given yaw angle to the maximum yaw range.
        
        Args:
            yaw (float): The yaw angle to limit.
        
        Returns:
            float: The limited yaw angle in radians.
        """
        if HeadAware.is_yaw_in_scope(yaw):
            return yaw
        return HeadTrackBall.MAX_YAW if yaw > 0 else -HeadTrackBall.MAX_YAW