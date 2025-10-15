from util.actioncommand import kick
from util.Global import ball_rel_pos
from BehaviourTask import BehaviourTask
import robot


class Kick(BehaviourTask):

    """ 
        Description:
        This skill just dictates the aspects of a kick such as 
        the power of the kick and the foot to kick with.
    """
    
    def _reset(self):
        self.is_finished = False


    def _tick(self, pow=0.85, kicking_foot=None):
        """
            If no kicking_foot as been specified:
            - Assume the ball is left to the robot, KICK using LEFT foot.
            - IF ball on its right side, KICK with RIGHT foot. 
        """
        if kicking_foot == None:
            if ball_rel_pos().y >= 0:
                kicking_foot = robot.Foot.LEFT
            else:
                kicking_foot = robot.Foot.RIGHT

        self.world.b_request.actions.body = kick(power=pow, extraStableKick=True, foot=kicking_foot)

        action = self.world.blackboard.motion.active.body.actionType
        if action is robot.ActionType.KICK:
            self.is_finished = False
        else:
            self.is_finished = True
