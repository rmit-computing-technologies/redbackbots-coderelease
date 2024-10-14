from math import radians, degrees

import util.LedOverride as LedOverride
from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from head.HeadTrackBall import HeadTrackBall
from head.HeadCentre import HeadCentre
from util.Constants import LEDColour, LEDSegments
from util.Global import canSeeBall, myPosUncertainty, isBallLost, numBallsSeenInLastXFrames
from util.GameStatus import penalised, in_finished, in_ready, in_set, in_initial
from util.Hysteresis import Hysteresis
from util.Timer import Timer, WallTimer
from util import Log

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

    May want to combine this with a body skill that turns around if we're lost.

    TODO - support sharing/reading information with our teammates?
    """
    
    # Dictionary of sequences from str to tuple of scope and divisions in format (scope, divisions)
    _sequences = { "Wide": {"scope": 160, "divisions": 6, "speed": 0.20},
                  "SlowWide": {"scope": 160, "divisions": 8, "speed": 0.20},
                  "FastWide": {"scope": 160, "divisions": 5, "speed": 0.20},
                  "Medium": {"scope": 110, "divisions": 6, "speed":0.20},
                  "Narrow": {"scope": 40, "divisions": 3},
                  "Tiny": {"scope": 20, "divisions": 2}}

    DEFAULT_SPEED = 0.25 # default speed for head movement was 0.25
    SHOULDER_SPEED = 0.5
    STARE_SECONDS = 0.20 # Time to stare in between positions was 0.4
    PITCH = radians(19)
    # NARROW_PERIOD = 2.0  # seconds
    MAX_UNCERTAINTY = 70_000 # was 100000 and most recently was 130_000

    _ball_lost = Hysteresis(min_value=0, max_value=70) # was 50
    BALL_LOST_TIME = 3 # seconds
    high_uncertainty = Hysteresis(min_value=0, max_value=70) # was 50 and at home was 300
    localise = True
    KEEP_TRACKING_BALL_FRAMES = 45 # Number of frames to keep tracking the ball after it's lost
    
    LOOKING_AT_SHOULDER_ANGLE = 45 # Angle at which we increase speed to avoid spending too much time on the edge of our vision

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Localise": HeadFixedYawAndPitch(self),
            "FindBall": HeadFixedYawAndPitch(self),
            "TrackBall": HeadTrackBall(self),
            "Penalised": HeadCentre(self)
        }

    def _transition(self):
        if penalised() or in_finished():
            self._current_sub_task = "Penalised"
            return
    
        #Log.info(f"Position Uncertainty: {myPosUncertainty()}")
    
        if myPosUncertainty() > self.MAX_UNCERTAINTY:
            HeadAware.high_uncertainty.up()
        else:
            HeadAware.high_uncertainty.reset()

        if HeadAware.localise and HeadAware.high_uncertainty.is_max():
            LedOverride.override_eye_segment(LedOverride.leftEye, LedOverride.roleSegments, LEDColour.yellow)
        
        if not canSeeBall():
            HeadAware._ball_lost.up()
        else:
            HeadAware._ball_lost.reset()

        if (canSeeBall() or HeadAware.JustLostBall()) and not in_ready(): # TODO: TEST
            self._current_sub_task = "TrackBall"
            #Log.debug("Head Tracking Ball")
            LedOverride.override_eye_segment(LedOverride.leftEye, LedOverride.roleSegments, LEDColour.red)
        elif (HeadAware.high_uncertainty.is_max()) or (in_initial() or in_ready() or in_set()):
            self._current_sub_task = "Localise"
            #Log.debug("Head Localising")
            LedOverride.override_eye_segment(LedOverride.leftEye, LedOverride.roleSegments, LEDColour.yellow)
        elif isBallLost(self.BALL_LOST_TIME):
                self._current_sub_task = "FindBall"
                #Log.debug("Head Finding Ball")
                LedOverride.override_eye_segment(LedOverride.leftEye, LedOverride.roleSegments, LEDColour.blue)
        else:
            self._current_sub_task = "Localise"
            #Log.debug("Head Localising")
            LedOverride.override_eye_segment(LedOverride.leftEye, LedOverride.roleSegments, LEDColour.green)  

        # Decide the scope
        if in_initial() or in_ready() or in_set():
            self._current_sequence = "Wide"
        elif self._current_sub_task == "FindBall":
            self._current_sequence = "Medium"
        elif self._current_sub_task == "Localise":
            self._current_sequence = "Wide" # was "Wide" 
        else:
            self._current_sequence = "Narrow"


    def _reset(self):
        self.PITCH = radians(19 + self.world.blackboard.kinematics.parameters.cameraPitchBottom)

        self._current_sub_task = "FindBall"
        self._current_sequence = "Wide"
        # self._current_speed = self.DEFAULT_SPEED

        HeadAware.high_uncertainty.resetMax()
        HeadAware._ball_lost.resetMax()
        self._timer = Timer(self.STARE_SECONDS * 1000000)  # convert to micro-seconds  # noqa
        self._yaw_aim = 0
        self._sequence_counter = 0
        self._currently_moving = False
        self._timer_since_start = WallTimer()
        self._current_speed, self._sequence = self._calculate_sequence(self._sequences[self._current_sequence])

    def _tick(self):
        if self._current_sub_task == "TrackBall" or self._current_sub_task == "Penalised":
            # No head movement required
            self._tick_sub_task()
            return
        
        # Load the sequence
        self._current_speed, self._sequence = self._calculate_sequence(self._sequences[self._current_sequence])
        
        #Log.debug(f"Current headaware sequence: {self._current_sequence}")

        can_stop = True
        reached_goal = True

        if self.looking_at_shoulder():
            self.yaw_speed = self.SHOULDER_SPEED
            #Log.warning("Looking at shoulder -- Speeding up and not pausing")
            # self._timer.force_expire() # Set timer to finished
            # self._currently_moving = False
            can_stop = False
        else:
            can_stop = True

        # only restart timer when we've reached the position
        if self._currently_moving and \
                (self._sub_tasks[self._current_sub_task].arrived() or
                 self._sub_tasks[self._current_sub_task].cant_move_more()):
            # We have stopped and should wait until we can start again
            self._timer.restart()
            self._currently_moving = False

        # if not self._currently_moving:
            #Log.error("=================== Paused ===================")
            
        # if not self._timer.finished():
            #Log.info("=================== NOT YET ===================")
            
        # if can_stop:
            #Log.warning("^^^^^^^^^^^^^^^^^^^^ CAN STOP ^^^^^^^^^^^^^^^^^^^^")

        if (not self._currently_moving and self._timer.finished()) or not can_stop:
            if (self._sub_tasks[self._current_sub_task].arrived() or
                 self._sub_tasks[self._current_sub_task].cant_move_more()):
                self._increment_sequence_counter()
                reached_goal = False
            self._yaw_aim = self._sequence[self._sequence_counter]
            self._currently_moving = True

        #Log.info(f"************** Current Goal: {degrees(self._yaw_aim)} **************")

        self._tick_sub_task(
            yaw=self._yaw_aim,
            pitch=self.PITCH,
            yaw_speed=self._current_speed,
            pitch_speed=0.5)

    def _calculate_sequence(self, sequence : dict):
        scope, divisions = sequence["scope"], sequence["divisions"]
        # Check if "speed" key exists in sequence dictionary and return the speed if it does, otherwise return the default speed
        speed = sequence.get("speed", self.DEFAULT_SPEED)
        half_scope = scope // 2
        
        # If we will look at our shoulder mid sequence
        if abs(half_scope) > self.LOOKING_AT_SHOULDER_ANGLE:
            # Ignore the edge divisions
            divisions -= 2
            
            if divisions % 2 == 0:
                increment = int(self.LOOKING_AT_SHOULDER_ANGLE // (divisions/2))
            else:
                increment = int(self.LOOKING_AT_SHOULDER_ANGLE // ((divisions-1)/2))
                
            if increment == self.LOOKING_AT_SHOULDER_ANGLE:
                increment -= 1
            #Log.debug(f"Increment: {increment}")
            
            sequence = [radians(i) for i in range(0, self.LOOKING_AT_SHOULDER_ANGLE, increment)] + \
                        [radians(half_scope), radians(self.LOOKING_AT_SHOULDER_ANGLE)] + \
                        [radians(i) for i in range(self.LOOKING_AT_SHOULDER_ANGLE - increment, -self.LOOKING_AT_SHOULDER_ANGLE, -increment)] + \
                        [radians(-half_scope), radians(-self.LOOKING_AT_SHOULDER_ANGLE)] + \
                        [radians(i) for i in range(-self.LOOKING_AT_SHOULDER_ANGLE + increment, 0, increment)]
            #Log.debug(f"Avoid shoulder sequence: {[round(degrees(i)) for i in sequence]}")
        else:
            if divisions % 2 == 0:
                increment = int(half_scope // (divisions/2))
            else:
                increment = int(half_scope // ((divisions-1)/2))
                
            sequence = [radians(i) for i in range(0, half_scope, increment)] + \
                [radians(i) for i in range(half_scope, -half_scope, -increment)] + \
                [radians(i) for i in range(-half_scope + increment, 0, increment)]
            #Log.debug(f"Normal sequence: {[round(degrees(i)) for i in sequence]}")
        return speed, sequence

    def _increment_sequence_counter(self):
        self._sequence_counter += 1
        if (self._sequence_counter >= len(self._sequence)):
            self._sequence_counter = 0
    
    @staticmethod
    def JustLostBall():
        return numBallsSeenInLastXFrames(HeadAware.KEEP_TRACKING_BALL_FRAMES) > 1
    
    def looking_at_shoulder(self):
        currAngles = angles(self.world.blackboard)
        curr_yaw = currAngles[Joints.HeadYaw]
        return(abs(curr_yaw) > radians(HeadAware.LOOKING_AT_SHOULDER_ANGLE))
