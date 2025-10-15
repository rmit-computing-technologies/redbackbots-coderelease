from BehaviourTask import BehaviourTask
from util.ObstacleAvoidance import walk_vec_with_avoidance
from body.skills.Walk import Walk
from util.Vector2D import Vector2D


class avoidance_testing(BehaviourTask):

    WALK_SPEED = 300  # mm / s

    def _initialise_sub_tasks(self):
        self._sub_tasks = {"Walk": Walk(self)}

    def _tick(self):
        turn=0
        speed=1.0

        walk_vector = Vector2D(self.WALK_SPEED, 0)
        walk_vector = walk_vec_with_avoidance(walk_vector)

        forward = walk_vector.x
        left = walk_vector.y

        self._tick_sub_task(forward, left, turn, speed=speed)

    def _reset(self):
        self._current_sub_task = "Walk"