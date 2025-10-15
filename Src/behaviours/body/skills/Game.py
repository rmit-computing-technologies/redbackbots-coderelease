from BehaviourTask import BehaviourTask
from body.skills.Standby import Standby
from body.skills.Initial import Initial
from body.skills.Ready import Ready
from body.skills.Set import Set
from body.skills.Playing import Playing
from body.skills.Finished import Finished
from body.skills.Penalised import Penalised
from body.skills.WalkToCorner import WalkToCorner
from body.skills.Walk import Walk
from body.skills.PenaltyDefender import PenaltyDefender
from body.skills.PenaltyKicker import PenaltyKicker
from body.skills.DefenceAngle import DefenceAngle 

from util.Timer import Timer
import util.led_override as led_override
from util.Constants import LEDColour
from util.Vector2D import Vector2D
from util import log
from math import degrees

from util.GameStatus import enemy_goal, GameState, in_initial, in_standby, in_ready, in_set, in_playing, in_finished, penalised, penalised_motion_in_set, in_penaltyshoot_phase, we_are_kicking_team, prev_game_state

from util.Constants import FIELD_LENGTH, FIELD_WIDTH, PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH, GOAL_BOX_LENGTH, GOAL_BOX_WIDTH
class Game(BehaviourTask):
    
    '''
    Description:
    A skill to deal with a game environment. This task should be specific
    to a game of soccer, using a GameController.
    '''

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Initial" : Initial(self),
            "Standby": Standby(self),
            "Ready" : Ready(self),
            "Set" : Set(self),
            "Playing" : Playing(self),
            "Finished" : Finished(self),
            "Penalised" : Penalised(self),
            "CornerKick": WalkToCorner(self),
            "Walk": Walk(self),
            "PenaltyKicker": PenaltyKicker(self),
            "PenaltyDefender": PenaltyDefender(self)
        }

    _just_penalised = False
    _penalised_timer = Timer(5000000) # 5 secs

    def _reset(self):
        
        msg = '\n'.join((
            '',
            '# Congratulations RedbackBots is running.',
            '# Remember to make the robot stiff (or limp) with ',
            '# one chest button press. ',
            '# ',
            '# You might like to try a specific skill, for example: ',
            '#     ./redbackbots -s Demo',
        ))
        log.info(msg)
        self._current_sub_task = "Initial"


    def _transition(self):
        if penalised():
            self._current_sub_task = "Penalised"
            if not penalised_motion_in_set():
                self._just_penalised = True
                self._penalised_timer.start().restart()
        else:
            if in_penaltyshoot_phase():
                if in_set():
                    self._current_sub_task = "Set"
                elif in_finished():
                    self._current_sub_task = "Finished"
                elif in_playing():
                    if we_are_kicking_team():
                        self._current_sub_task = "PenaltyKicker"
                    else:
                        self._current_sub_task = "PenaltyDefender"
                return

            if self._penalised_timer.finished():
                self._just_penalised = False

            if self._just_penalised:
                if in_ready():
                    led_override.override(led_override.CHEST_BUTTON, LEDColour.ready)
                    self._current_sub_task = "Walk"
                    return
                elif in_playing():
                    led_override.override(led_override.CHEST_BUTTON, LEDColour.playing)
                    self._current_sub_task = "Walk"
                    return
            
            if in_initial():
                self._current_sub_task = "Initial"
            elif in_standby():
                self._current_sub_task = "Standby"
            elif in_ready() and prev_game_state() != GameState.READY:
                self._sub_tasks["Ready"] = Ready(self)
                self._current_sub_task = "Ready"
            elif in_ready():
                self._current_sub_task = "Ready"
            elif in_set():
                self._current_sub_task = "Set"
            elif in_playing():               
                self._current_sub_task = "Playing"
            elif in_finished():
                self._current_sub_task = "Finished"


    def _tick(self):
        # for pos in [
        #     Vector2D(0,0),
        #     Vector2D(0, FIELD_WIDTH/2),
        #     Vector2D(0, -FIELD_WIDTH/2),
        #     Vector2D(0, FIELD_WIDTH/10),
        #     Vector2D(0, -FIELD_WIDTH/10),
        #     Vector2D(-FIELD_LENGTH/2 + PENALTY_BOX_LENGTH, FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2 + PENALTY_BOX_LENGTH, -FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH, FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH, -FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2, FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2, -FIELD_WIDTH/2),
        #     Vector2D(-FIELD_LENGTH/2, -FIELD_WIDTH/2),
        #     Vector2D(FIELD_LENGTH/4, -FIELD_WIDTH/2),
        #     Vector2D(FIELD_LENGTH/2, -FIELD_WIDTH/2),
        #     Vector2D(FIELD_LENGTH/2, -FIELD_WIDTH/2),

        # ]:
        #     angle = enemy_goal().minus(pos).heading()
        #     angleTo = enemy_goal().headingTo(pos)
        #     angleToPos = pos.headingTo(enemy_goal())
        #     log.error(f"{angle=}")
        #     log.error(f"{angleTo=}")
        #     log.error(f"{angleToPos=}")
        #     log.error(f"{angleToPos=}")
        #     log.info(f"{degrees(angle)=}")
        #     log.info(f"{pos=}")

        #     if abs(degrees(angle)) < 100/2:
        #         log.info("Could kick by angle")
        #     else:
        #         log.info("Can't kick by angle")

        # log.info("")
        if (self._current_sub_task == "Walk"):
            self._tick_sub_task(forward=1000)
        else:
            self._tick_sub_task()