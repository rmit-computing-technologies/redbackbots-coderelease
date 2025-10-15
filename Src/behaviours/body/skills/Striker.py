from BehaviourTask import BehaviourTask
from body.skills.Dribble import Dribble
from body.skills.LineupPass import LineupPass
from body.skills.LineupKick import LineupKick
from body.skills.TurnToBall import TurnToBall
from body.skills.CornerKick import CornerKick
from util.GameStatus import in_corner_kick, is_in_goal_kicking_area, we_are_kicking_team, in_penalty_kick, enemy_goal
from util.Global import ego_ball_lost, is_ball_lost, usingGameSkill, is_ball_in_our_box
from util.TeamStatus import robot_can_score
from util.Constants import GOAL_WIDTH, FIELD_WIDTH

class Striker(BehaviourTask):

    """ 
        Description:
        This class contains the actioins that the striker would carry out. 
        Included skills are all applicable to the tasks of locating and kicking the ball 
        towards the goal. 

        If vision of the ball is lost for a certain amount of time, then the stricker will stop 
        setup and look for the ball.
        If outside the penanlty box, the stricker will dribble.
        If inside the penanlty box, the stricker will kick the ball.
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Dribble": Dribble(self),
            "LineupPass": LineupPass(self),
            "LineupKick": LineupKick(self),
            "KickAway": LineupKick(self),
            "CornerKick": CornerKick(self),
            "TurnToBall": TurnToBall(self)
        }

    def _transition(self):
        # Handling for special set plays
        if we_are_kicking_team():
            if in_corner_kick():
                self._current_sub_task = "CornerKick"
                return
            
            elif in_penalty_kick():
                self._current_sub_task = "LineupKick"
                return


        if is_in_goal_kicking_area():
            self._current_sub_task = "LineupKick"

        elif is_ball_in_our_box()[0]:
            self._current_sub_task = "KickAway"

        # If we are allowed to score and are in the correct location then try to score
        else:
            self._current_sub_task = "Dribble"

    def _reset(self):
        self._current_sub_task = "LineupKick"
        self.field_position = ("Superstar", None)


    def _tick(self, field_position = ("Superstar", None)):
        self.field_position = field_position
        
        if in_penalty_kick() and self._current_sub_task == "LineupKick":
            target = enemy_goal()
            target.y = GOAL_WIDTH / 4.0
            self._tick_sub_task(target=target)

        if self._current_sub_task == "KickAway":
            self._tick_sub_task(range=FIELD_WIDTH/2)

        else:   
            self._tick_sub_task()
