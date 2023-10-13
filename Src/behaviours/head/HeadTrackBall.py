from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from util.Global import ballHeading, canSeeBall, blackboard
from math import radians, pi
from util.Sensors import angles
from robot import Joints
from util.Hysteresis import Hysteresis

import time


class HeadTrackBall(BehaviourTask):

    """ 
        Description:
        A skill associated with looking at the yaw and the pitch of the head when 
        tracking the ball. 
        
    """
    blackboard = blackboard

    LOWER_CAMERA_VERTICAL_CENTER = 1200
    LOWER_CAMERA_VERTICAL_SIZE = 2400
    PITCH_SCALE = 68
    CAMERA_HEIGHT = 480

    _ball_in_camera = 1200

    _head_yaw = 0
    _head_pitch = 0

    # LOOK RIGHT
    MAX_YAW = radians(40)
    # LOOK LEFT
    MIN_YAW = radians(-40)

    # Dont want pitch Looking up away from field
    MAX_PITCH = radians(0)

    # for testing purposes only
    _start_time = time.time()

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "HeadFixedYawAndPitch": HeadFixedYawAndPitch(self)
        }

    def _reset(self):
        self._current_sub_task = "HeadFixedYawAndPitch"

    def _pitch2rad(self, pitch_diff):
        return ((pitch_diff / self.LOWER_CAMERA_VERTICAL_SIZE) * self.PITCH_SCALE) * pi / 180

    def _tick(self):
        if canSeeBall():

            currAngles = angles(self.world.blackboard)
            self._curr_pitch = currAngles[Joints.HeadPitch]

            self._head_yaw = ballHeading()

            if self._head_yaw < self.MIN_YAW:
                self._head_yaw = self.MIN_YAW
            elif self._head_yaw > self.MAX_YAW:
                self._head_yaw = self.MAX_YAW

            self._ball_in_camera = self.blackboard.vision.balls[0].imageCoords[1]
            self._pitch_diff = self.LOWER_CAMERA_VERTICAL_CENTER - self._ball_in_camera

            if abs(self._pitch_diff) > 100:
                # moving head based on camera 
                self._head_pitch = self._curr_pitch - self._pitch2rad(float(self._pitch_diff))

            # Avoiding negative radians as that is looking up
            if self._head_pitch < self.MAX_PITCH:
                self._head_pitch = self.MAX_PITCH

            # self._log_angle()  
            # self._log_pitch2rad()

            self._tick_sub_task(yaw=self._head_yaw, pitch=self._head_pitch)
        else:
            self._tick_sub_task(yaw=self._head_yaw, pitch=self._head_pitch)

    def _log_angle(self):
        print("[{}] Camera: {}\t Pitch: {}\tYaw: {}\t PD: {}\t NewPitch: {}".format(
            str(round((time.time() - self._start_time), 1)),
            round(self.blackboard.vision.balls[0].imageCoords[1], 5),
            self._head_pitch,
            self._head_yaw,
            self._pitch_diff,
            self._pitch2rad(self._pitch_diff)
        ))

    def _log_pitch2rad(self):
        print("[{}] P:{}, CS:{}, PS:{}, R:{}, P/C:{}, *S:{}, *R:{}, FUNCTION:{}".format(
            str(round((time.time() - self._start_time), 1)),
            self._pitch_diff,
            self.LOWER_CAMERA_VERTICAL_SIZE,
            self.PITCH_SCALE,
            pi / 180,
            self._pitch_diff / self.LOWER_CAMERA_VERTICAL_SIZE,
            (self._pitch_diff / self.LOWER_CAMERA_VERTICAL_SIZE) * self.PITCH_SCALE,
            ((self._pitch_diff / self.LOWER_CAMERA_VERTICAL_SIZE) * self.PITCH_SCALE) * pi / 180,
            self._pitch2rad(float(self._pitch_diff))
        ))
