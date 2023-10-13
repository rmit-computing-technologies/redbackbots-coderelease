from math import radians

import util.LedOverride as LedOverride
from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from head.HeadTrackBall import HeadTrackBall
from head.HeadCentre import HeadCentre
from util.Constants import LEDColour
from util.Global import canSeeBall, myPosUncertainty
from util.GameStatus import penalised, in_finished
from util.Hysteresis import Hysteresis
from util.Timer import Timer, WallTimer


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

    SEQUENCE_TINY = [
        radians(-15),
        radians(15),
    ]
    SEQUENCE_NARROW = [
        radians(-40),
        radians(-30),
        radians(-20),
        radians(-10),
        radians(0),
        radians(10),
        radians(20),
        radians(30),
        radians(40),
        radians(30),
        radians(20),
        radians(10),
        radians(0),
        radians(10),
        radians(-20),
    ]

    SEQUENCE_WIDE = [
        radians(-40),
        radians(-30),
        radians(-20),
        radians(-10),
        radians(0),
        radians(10),
        radians(20),
        radians(30),
        radians(40),
        radians(30),
        radians(20),
        radians(10),
        radians(0),
        radians(-10),
        radians(-20),
        radians(-30),
        radians(-40),
    ]

    STARE_SECONDS = 0.2 # was 0.4
    PITCH = radians(19)
    NARROW_PERIOD = 2.0  # seconds
    MAX_UNCERTAINTY = 130000 # was 100000

    _ball_lost = Hysteresis(min_value=0, max_value=100) # was 50
    high_uncertainty = Hysteresis(min_value=0, max_value=500) # was 50
    localise = True

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Localise": HeadFixedYawAndPitch(self),
            "TurnToBall": HeadFixedYawAndPitch(self),
            "TrackBall": HeadTrackBall(self),
            "Penalised": HeadCentre(self)
        }

    def _transition(self):
        if penalised() or in_finished():
            self._current_sub_task = "Penalised"
            return

        if myPosUncertainty() > self.MAX_UNCERTAINTY:
            HeadAware.high_uncertainty.up()
        else:
            HeadAware.high_uncertainty.reset()

        # if HeadAware.localise and HeadAware.high_uncertainty.is_max():
        #     self._current_sub_task = "Localise"
        #     LedOverride.override(LedOverride.leftEye, LEDColour.yellow)
        if canSeeBall():
            self._ball_lost.reset()
            self._current_sub_task = "TrackBall"
            LedOverride.override(LedOverride.leftEye, LEDColour.red)
        elif self._ball_lost.is_max():
            self._current_sub_task = "TurnToBall"
            LedOverride.override(LedOverride.leftEye, LEDColour.blue)
        else:
            self._ball_lost.up()

    def _reset(self):
        self.PITCH = radians(19 + self.world.blackboard.kinematics.parameters.cameraPitchBottom)
        
        if HeadAware.localise:
            self._current_sub_task = "Localise"
        else:
            self._current_sub_task = "TurnToBall"

        HeadAware.high_uncertainty.resetMax()
        self._timer = Timer(self.STARE_SECONDS * 1000000)  # convert to micro-seconds  # noqa
        self._yaw_aim = 0
        self._sequence_counter = 0
        self._currently_moving = False
        self._timer_since_start = WallTimer()
        self._sequence = self.SEQUENCE_NARROW

    def _tick(self):
        if self._current_sub_task == 'TrackBall' or self._current_sub_task == "Penalised":
            self._tick_sub_task()
            return
        self._sequence = self._choose_sequence()

        # only restart timer when we've reached the position
        if self._currently_moving and \
                (self._sub_tasks[self._current_sub_task].arrived() or
                 self._sub_tasks[self._current_sub_task].cant_move_more()):
            self._timer.restart()
            self._currently_moving = False

        if not self._currently_moving and self._timer.finished():
            self._increment_sequence_counter()
            self._yaw_aim = self._sequence[self._sequence_counter]
            self._currently_moving = True
        self._tick_sub_task(
            yaw=self._yaw_aim,
            pitch=self.PITCH,
            yaw_speed=0.25,
            pitch_speed=0.5)

    def _choose_sequence(self):
        if self.world.b_request.actions.body.forward != 0:
            return self.SEQUENCE_NARROW # TODO: Was tiny

        # if HeadAware.localise and self._current_sub_task == "Localise":
        #     return self.SEQUENCE_WIDE
        
        else:
            return self.SEQUENCE_WIDE # TODO: Was narrow

    def _increment_sequence_counter(self):
        self._sequence_counter += 1
        if (self._sequence_counter >= len(self._sequence)):
            self._sequence_counter = 0

    def does_localise(self, localise = False): # CHANGED ALL TO FALSE
        """Sets localise to True or False. Defaults to True."""
        HeadAware.localise = False