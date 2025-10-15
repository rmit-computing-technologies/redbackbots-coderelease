from BehaviourTask import BehaviourTask
from body.skills.DefenceAngle import DefenceAngle
from body.skills.Ready import Ready
from util.Constants import (
    CENTER_CIRCLE_DIAMETER,
    GOAL_POST_ABS_Y,
    PENALTY_BOX_LENGTH,
    FIELD_LENGTH
)
from util.GameStatus import own_goal, enemy_goal
from util import log

class KickOffDefence(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "DefenceAngle": DefenceAngle(self)
        }

    def _reset(self):
        self._current_sub_task = "DefenceAngle"

        OWN_GOAL_POSITIVE = own_goal().x >= 0

        QUARTER_FIELD_LENGTH = FIELD_LENGTH/4.0
        HALF_CENTER_CIRCLE_DIAMETER = CENTER_CIRCLE_DIAMETER/2.0
    
        self.UPFIELDER_X = CENTER_CIRCLE_DIAMETER if OWN_GOAL_POSITIVE else -CENTER_CIRCLE_DIAMETER
        self.ATTACKER_X = CENTER_CIRCLE_DIAMETER if OWN_GOAL_POSITIVE else -CENTER_CIRCLE_DIAMETER
        self.MIDFIELDER_X = QUARTER_FIELD_LENGTH if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
        self.DEFENDER_X = QUARTER_FIELD_LENGTH + HALF_CENTER_CIRCLE_DIAMETER if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH

    def _tick(self, reference_point = own_goal(), field_position = ("Superstar", None)):
        if field_position[0] == "Upfielder":
            kick_off_x = self.UPFIELDER_X
        elif field_position[0] == "Attacker" or field_position[0] == "Superstar":
            kick_off_x = self.ATTACKER_X
        elif field_position[0] == "Midfielder":
            kick_off_x = self.MIDFIELDER_X
        else:
            log.error(msg="Failed to set x in kickoff defence", say=True)
            kick_off_x = self.DEFENDER_X

        # Left and right positions have different reference points
        # corresponding to the left and right sides of the goal
        if field_position[1] is not None or field_position[0] == "Midfielder" or field_position[0] == "Upfielder":
            if field_position[1] == "Left" or field_position[0] == "Midfielder":
                reference_point.y = GOAL_POST_ABS_Y
            elif field_position[1] == "Right" or field_position[0] == "Upfielder":
                reference_point.y = -GOAL_POST_ABS_Y

        self._tick_sub_task(
            reference_point=reference_point,
            default_x=kick_off_x,
            dynamic=False
        )
