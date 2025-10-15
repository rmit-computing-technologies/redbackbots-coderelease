from BehaviourTask import BehaviourTask
from body.skills.WalkToCorner import WalkToCorner
from body.skills.LineupKick import LineupKick
from head.HeadAware import HeadAware
from util.Vector2D import Vector2D
from util.Constants import GOAL_BOX_LENGTH
from util.FieldGeometry import ENEMY_GOAL_CENTER



class CornerKick(BehaviourTask):
    
    TARGET_POINT = Vector2D(ENEMY_GOAL_CENTER.x - GOAL_BOX_LENGTH, 0)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToCorner": WalkToCorner(self),
            "LineupKick": LineupKick(self)
        }

    def _reset(self):
        self._current_sub_task = "WalkToCorner"
    
    def _transition(self):
        if self._current_sub_task == "LineupKick" or (self._current_sub_task == "WalkToCorner" and self._sub_tasks[self._current_sub_task]._in_corner):
            self._current_sub_task = "LineupKick"
        else:
            
            self._current_sub_task = "WalkToCorner"
            
    def _tick(self):
        if self._current_sub_task == "LineupKick":
            self._tick_sub_task(target = self.TARGET_POINT)
        else:
            self._tick_sub_task()
            
            
