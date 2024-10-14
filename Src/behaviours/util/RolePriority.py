from util.GameStatus import testing_with_half_field
from util.TeamStatus import is_first_team

class RolePriority:
    half_field_defend = [("Defender", "Center"), ("Defender", "Left"), ("Attacker", "Left"), ("Attacker", "Right")]
    half_field_attack = [("Attacker", "Left"), ("Attacker", "Right"), ("Midfielder", "Center"), ("Defender", "Left")]
    competition = [("Attacker", "Left"), ("Attacker", "Right"), ("Defender", "Left"), ("Defender", "Right")]
    ready_state = [("Attacker", "Left"), ("Attacker", "Right"), ("Defender", "Left"), ("Defender", "Right")]

    def current_role_priorities():
        if testing_with_half_field():
            if is_first_team():
                return RolePriority.half_field_defend
            else:
                return RolePriority.half_field_attack
        else:
            return RolePriority.competition
        
    def current_priorities_for_ready():
        if testing_with_half_field():
            if is_first_team():
                return RolePriority.half_field_defend
            else:
                return RolePriority.half_field_attack
        else:
            return RolePriority.ready_state