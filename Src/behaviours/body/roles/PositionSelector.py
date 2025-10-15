from BehaviourTask import BehaviourTask
from body.roles.Attacker import Attacker
from body.roles.Defender import Defender
from body.roles.Goalie import Goalie
from body.roles.Midfielder import Midfielder
from body.roles.Upfielder import Upfielder
from body.roles.Superstar import Superstar
from util import led_override
from util.Constants import LEDColour
from util.RolePriority import RolePriority
from util import log

from util.TeamStatus import my_player_number, get_active_player_numbers, is_first_team, get_teammate_pos, \
    teammate_ego_ball, player_role, get_non_substituted_player_numbers, GOALIE_PLAYER_NUMBERS
from util.GameStatus import ball_world_pos, testing_with_half_field
from util.Global import myPos

class PositionSelector(BehaviourTask):
    """
        Descriptions:
        All robots have a player number assigned to them and a list of role priorities are
        defined.
        Appropriate python role are then specified as the subtask
    """
    TESTING_WITH_HALF_FIELD = testing_with_half_field()

    POSITION_COLOURS = {
        "Attacker": LEDColour.orange,
        "Defender": LEDColour.pink,
        "Goalie": LEDColour.cyan,
        "Midfielder": LEDColour.purple,
        "Upfielder": LEDColour.dim_white,
        "Superstar": LEDColour.dark_green
    }

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Attacker": Attacker(self),
            "Defender": Defender(self),
            "Goalie": Goalie(self),
            "Midfielder": Midfielder(self),
            "Upfielder": Upfielder(self),
            "Superstar": Superstar(self)
        }

    def _reset(self):
        self._current_sub_task = "Attacker"
        self._position = "Center"
        self._defender_goalie_call = False

    def _half_field_transition(self, active_field_player_numbers, role_priority):
        if my_player_number() in GOALIE_PLAYER_NUMBERS:
            self._current_sub_task = "Goalie"
            return
        if my_player_number() not in active_field_player_numbers:
            self._current_sub_task = "Superstar"
            return
        self._current_sub_task, self._position = role_priority[active_field_player_numbers.index(my_player_number())]
        if len(active_field_player_numbers) == 1 and active_field_player_numbers[0] == my_player_number():
            self._current_sub_task = "Superstar"
        return

    def _transition(self):

        active_field_player_numbers = get_active_player_numbers()
        non_substituted_player_numbers = get_non_substituted_player_numbers()
        # 1 is goalie, so remove it from role consideration
        active_field_player_numbers = [x for x in active_field_player_numbers if x not in GOALIE_PLAYER_NUMBERS]
        non_substituted_player_numbers = [x for x in non_substituted_player_numbers if x not in GOALIE_PLAYER_NUMBERS]
        role_priority = RolePriority.current_role_priorities()

        # Used for testing with half_field in AIIL to guarantee a defender in teams of 2 or less
        if testing_with_half_field():
            self._half_field_transition(active_field_player_numbers, role_priority)
            return

        # If I am a goalie, I should be a goalie
        if my_player_number() in GOALIE_PLAYER_NUMBERS:
            self._current_sub_task = "Goalie"
            return

        # If there is only 1 field player, I should be a superstar
        if len(active_field_player_numbers) == 1:
            self._current_sub_task = "Superstar"
            return

        # If I'm not in the list of players, I should be a superstar temporarily
        if my_player_number() not in non_substituted_player_numbers or my_player_number() not in active_field_player_numbers:
            self._current_sub_task = "Superstar"
            return

        # If there isn't a goalie, the player with the largest number should become a goalie
        if not any(goalie_numbers in get_active_player_numbers() for goalie_numbers in GOALIE_PLAYER_NUMBERS):
            goalie_defender = active_field_player_numbers.pop()
            non_substituted_player_numbers.remove(goalie_defender)
            if my_player_number() == goalie_defender:
                if not self._defender_goalie_call:
                    log.warning(msg="Going into defender goalie", say=True)
                    self._defender_goalie_call = True
                self._current_sub_task = "Goalie"
                return
        
        self._defender_goalie_call = False

        # Get an initial sub_task and position based on the field robots we are using
        self._current_sub_task, self._position = role_priority[non_substituted_player_numbers.index(my_player_number())]

        # If we have 4 active field players, we can keep the assigned roles and positions
        if len(active_field_player_numbers) == 4:
            return

        active_positions = []
        for active_player_num in active_field_player_numbers:
            active_positions.append(role_priority[non_substituted_player_numbers.index(active_player_num)][0])

        # If there are two robots: one is guarenteed attacker and one is guarenteed defender
        # Smallest player number becomes attacker
        if len(active_field_player_numbers) == 2:
            if active_positions.count('Attacker') == 0:
                if active_field_player_numbers.index(my_player_number()) % 2 == 0:
                    self._current_sub_task = 'Attacker'
            elif active_positions.count('Defender') == 0:
                if active_field_player_numbers.index(my_player_number()) % 2 != 0:
                    self._current_sub_task = 'Defender'

            self._position = "Center"
            return

        # If there are three robots: there will always be a midfielder, upfielder and defender
        # Smallest player number becomes midfielder
        if len(active_field_player_numbers) == 3:
            sorted_players = sorted(active_field_player_numbers)

            if my_player_number() == sorted_players[0]:
                self._current_sub_task = 'Midfielder'
            elif my_player_number() == sorted_players[1]:
                self._current_sub_task = 'Upfielder'
            elif my_player_number() == sorted_players[2]:
                self._current_sub_task = 'Defender'

            self._position = "Center"
            return

        # If a position on the field is empty the adjacent quadrant will expand to fill that gap
        if self._current_sub_task == 'Attacker' and active_positions.count('Attacker') < 2:
            self._position = 'Center'
        elif self._current_sub_task == 'Defender' and active_positions.count('Defender') < 2:
            self._position = 'Center'

        # self.world.b_request.behaviourSharedData.role = role_priority.index(self._current_sub_task) + 1 if self._current_sub_task in role_priority else 0

    def _tick(self):
        # Override eye LED to represent Position
        led_override.override_eye_segment(led_override.RIGHT_EYE, led_override.POSITION_SEGMENTS, self.POSITION_COLOURS[self._current_sub_task])
        if self._current_sub_task in ["Attacker", "Midfielder", "Upfielder", "Defender"]:
            self._tick_sub_task(lr=self._position)
        else:
            self._tick_sub_task()
