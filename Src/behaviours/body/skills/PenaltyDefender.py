import util.led_override as led_override
from BehaviourTask import BehaviourTask
from body.skills.GoalieDive import GoalieDive
from body.skills.Stand import Stand
from head.HeadAware import HeadAware
from util.BallMovement import YWhenReachOurGoalBaseLine, timeToReachOurGoalBaseLineNoFriction
from util.Constants import GOAL_WIDTH, HALF_FIELD_LENGTH, LEDColour
from util.FieldGeometry import calculateTimeToReachPose
from util.Global import myHeading, myPos, ego_ball_lost
from util.Vector2D import Vector2D
from util import log


class PenaltyDefender(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Stand": Stand(self),
            "GoalieDive": GoalieDive(self)
        }

    def _transition(self):
        if not ego_ball_lost():
            if timeToReachOurGoalBaseLineNoFriction() < calculateTimeToReachPose(myPos(),myHeading(),Vector2D(-HALF_FIELD_LENGTH,YWhenReachOurGoalBaseLine())):
                if abs(YWhenReachOurGoalBaseLine()) < GOAL_WIDTH/2 and timeToReachOurGoalBaseLineNoFriction() > 0:
                    log.debug(timeToReachOurGoalBaseLineNoFriction())
                    self._current_sub_task = "GoalieDive"
                    if YWhenReachOurGoalBaseLine() > myPos().y:
                        led_override.override_eye_segment(led_override.LEFT_EYE, [1], LEDColour.magenta)
                        return
                    elif YWhenReachOurGoalBaseLine() < myPos().y:
                        led_override.override_eye_segment(led_override.RIGHT_EYE, [1], LEDColour.magenta)
                        return
                    else:
                        led_override.override_eye_segment(led_override.LEFT_EYE, [1], LEDColour.magenta)
                        led_override.override_eye_segment(led_override.RIGHT_EYE, [1], LEDColour.magenta)
                        return

        self._current_sub_task = "Stand"

    def _reset(self):
        self._current_sub_task = "Stand"

    def _tick(self):
        if self._current_sub_task == "GoalieDive":
            if YWhenReachOurGoalBaseLine() > myPos().y:
                self._tick_sub_task(direction = "Left")
            elif YWhenReachOurGoalBaseLine() < myPos().y:
                self._tick_sub_task(direction = "Right")
            else:
                self._tick_sub_task()
            return
        self._tick_sub_task()
