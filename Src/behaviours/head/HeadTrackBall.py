from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from util.Global import ball_heading, ball_distance, ball_world_vel, ball_world_pos
from math import radians
from util.Hysteresis import Hysteresis

class HeadTrackBall(BehaviourTask):
    """ 
        Description:
        A skill associated with looking at the yaw and the pitch of the head when 
        tracking the ball. 
    """
    
    BOTTOM_CAMERA = 0
    TOP_CAMERA = 1
    _current_camera = BOTTOM_CAMERA

    _head_yaw = 0
    _head_pitch = 0

    # LOOK RIGHT
    MAX_YAW = radians(60)
    # LOOK LEFT
    MIN_YAW = radians(-60)
    
    # Determined by using offnao
    MAX_DEGREES_TO_KEEP_BALL_IN_VIEW = 20

    # We never want the robot looking up and away from the horizon
    MIN_PITCH = 0
    # This is the lowest pitch the robots head joints/motors allow
    # See: Src/robot/include/utils/body.hpp
    MAX_PITCH = 29.5

    # These are the ball distances where the ball will be around 50-75% of bottom camera
    MIN_BALL_DIST_BOTTOM_CAMERA = 250
    MAX_BALL_DIST_BOTTOM_CAMERA = 550
    
    # These are the ball distances where the ball will be around 25-50% of top camera
    MIN_BALL_DIST_TOP_CAMERA = 450
    MAX_BALL_DIST_TOP_CAMERA = 1500

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "HeadFixedYawAndPitch": HeadFixedYawAndPitch(self)
        }

    def _reset(self):
        self._swing = 1
        self._head_swing = Hysteresis(-self.MAX_DEGREES_TO_KEEP_BALL_IN_VIEW, self.MAX_DEGREES_TO_KEEP_BALL_IN_VIEW)
        self._current_sub_task = "HeadFixedYawAndPitch"

    def _tick(self):
        self._ball_heading = ball_heading()
        self._ball_distance = ball_distance()
        self._head_yaw = self._ball_heading

        # BUG: Ball velocity and ball pos are constantly changing with head swing added
        # Removing for now until updated vision is in (which may handle it better)
        
        # self.set_head_swing()
        
        # self._head_yaw = self._ball_heading + radians(self._head_swing.value)
        # self._head_swing.add(self._swing)

        self._head_pitch, self._current_camera = HeadTrackBall.get_pitch_and_camera_from_distance(self._ball_distance, self._current_camera)
        self._head_pitch = radians(self._head_pitch)
        
        # Clamp to stay within shoulders
        if self._head_yaw < self.MIN_YAW:
            self._head_yaw = self.MIN_YAW
        elif self._head_yaw > self.MAX_YAW:
            self._head_yaw = self.MAX_YAW

        self._tick_sub_task(yaw=self._head_yaw, pitch=self._head_pitch, yaw_speed=0.2, pitch_speed= 0.15)
    
    @staticmethod
    def get_pitch_and_camera_from_distance(distance: float, current_camera: int) -> tuple:
        """
            Calculates the pitch angle based on the distance of the target.

            This function first determines if the camera (top or bottom) needs to be changed to track the target based on its distance.
            It then calculates the pitch angle required to keep the target in view of the selected camera.

            Args:
                distance (float): The distance to the target.
                current_camera (int): The currently active camera.

            Returns:
                tuple[float, int]: The calculated pitch angle in degrees, and the updated camera.
        """
        # Ensuring we don't flick constantly between cameras 
        # Both cameras can easily see the ball between 450 and 550s
        if distance > HeadTrackBall.MAX_BALL_DIST_BOTTOM_CAMERA:
            current_camera = HeadTrackBall.TOP_CAMERA
        elif distance < HeadTrackBall.MIN_BALL_DIST_TOP_CAMERA:
            current_camera = HeadTrackBall.BOTTOM_CAMERA

        if current_camera == HeadTrackBall.BOTTOM_CAMERA:
            if distance < HeadTrackBall.MIN_BALL_DIST_BOTTOM_CAMERA:
                return HeadTrackBall.MAX_PITCH, current_camera
            
            pitch = HeadTrackBall.MAX_PITCH - (distance - HeadTrackBall.MIN_BALL_DIST_BOTTOM_CAMERA) * (HeadTrackBall.MAX_PITCH / (HeadTrackBall.MAX_BALL_DIST_BOTTOM_CAMERA - HeadTrackBall.MIN_BALL_DIST_BOTTOM_CAMERA))
            return pitch, current_camera
        
        if current_camera == HeadTrackBall.TOP_CAMERA:
            if distance > HeadTrackBall.MAX_BALL_DIST_TOP_CAMERA:
                return HeadTrackBall.MIN_PITCH, current_camera
            
            pitch = HeadTrackBall.MAX_PITCH - (distance - HeadTrackBall.MIN_BALL_DIST_TOP_CAMERA) * (HeadTrackBall.MAX_PITCH / (HeadTrackBall.MAX_BALL_DIST_TOP_CAMERA - HeadTrackBall.MIN_BALL_DIST_TOP_CAMERA))
            return pitch, current_camera
        
        return 0.0, current_camera
        
    def set_head_swing(self):
        """
            Adjusts the direction of the head swing based on its current position.
            If the head swing is at its maximum position (right), it sets the swing direction to -1.
            If the head swing is at its minimum position (left), it sets the swing direction to 1.
        """
        
        if self._head_swing.is_max():
            self._swing = -1
        elif self._head_swing.is_min():
            self._swing = 1

    def get_prev_camera(self):
        """
            Returns the previous camera used for head tracking.
            This is to ensure when switching to just lost ball tracking, we can use the same camera
            that was used for head tracking.
        """
        return self._current_camera
