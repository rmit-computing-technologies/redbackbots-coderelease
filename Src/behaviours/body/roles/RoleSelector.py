from BehaviourTask import BehaviourTask
from head.HeadAware import HeadAware

from body.roles.Default import Default

from util.TeamStatus import my_player_number, get_active_player_numbers, is_first_team, get_teammate_pos, \
    teammate_ego_ball
from util.GameStatus import ballWorldPos
from util.Global import myPos, ballRelPos


class RoleSelector(BehaviourTask):
    """
        Descriptions:
        All robots have a player number assigned to them and a list of role priorities are
        defined. The role selector assigns the role with the highest priorty to the robot with the
        lowest active player number (except for Goalies which have fixed player numbers).
        Appropriate python role are then specified as the subtask
    """

    TESTING_WITH_HALF_FIELD = False

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Default": Default(self)
        }

    def _reset(self):
        self._current_sub_task = "Default"

    def _transition(self):
        self._current_sub_task = "Default"

    def _tick(self):
        self._tick_sub_task()
