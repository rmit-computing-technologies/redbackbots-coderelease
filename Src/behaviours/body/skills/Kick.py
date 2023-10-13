from util.actioncommand import kick
import robot
from BehaviourTask import BehaviourTask
from util.Global import ballRelPos


class Kick(BehaviourTask):

    """ 
        Description: 
        This skill just dictates the aspects of a kick such as the power of the kick
    """
    def _reset(self):
        self.is_finished = False

    def _tick(self,pow=0.85, kicking_foot=robot.Foot.LEFT):
        self.world.b_request.actions.body = kick(power=pow, extraStableKick=True, foot=kicking_foot)

        action = self.world.blackboard.motion.active.body.actionType
        if action is robot.ActionType.KICK:
            self.is_finished = False
        else:
            self.is_finished = True
