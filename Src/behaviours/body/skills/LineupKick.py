from BehaviourTask import BehaviourTask
from body.skills.Kick import Kick
from body.skills.Lineup import Lineup, AlignmentVectors
from util.GameStatus import enemy_goal
from util.Global import myHeading, ego_ball_lost
from math import radians
import robot

class LineupKick(BehaviourTask):
    target = enemy_goal()
    kick_with_right_foot = False

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Kick": Kick(self),
            "Lineup": Lineup(self),
        }

    def _reset(self):
        self._current_sub_task = "Lineup"

    def _transition(self):
        alignment_vectors = AlignmentVectors(goal_vec=self.target)
        self.kick_with_right_foot = alignment_vectors.get_kick_with_right_foot()
        target_distance = alignment_vectors.get_target_distance()
        kickable_distance = alignment_vectors.get_kickable_distance()
        abort_kick_distance = alignment_vectors.get_abort_kick_distance()
        kick_in_target_range = alignment_vectors.get_kick_in_target_range()
        
        if (((target_distance < kickable_distance and kick_in_target_range) or (
                self._current_sub_task == "Kick" and not self._sub_tasks[self._current_sub_task].is_finished)) \
                    and (target_distance < abort_kick_distance)
                    and not ego_ball_lost()):
            self._current_sub_task = "Kick"
        else:
            self._current_sub_task = "Lineup"

    def _tick(self, target = None, range = None):
        if target is not None:
            self.target = target
        
        if self._current_sub_task == "Lineup":
            if range is not None:
                self._tick_sub_task(target = self.target, max_range = range)
            else:
                self._tick_sub_task(target = target)
        else:
            if self.kick_with_right_foot:
                kicking_foot=robot.Foot.RIGHT
            else:
                kicking_foot=robot.Foot.LEFT

            if target is not None:
                self._tick_sub_task(pow=0.02, kicking_foot=kicking_foot)
            else:
                self._tick_sub_task(kicking_foot=kicking_foot)
