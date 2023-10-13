from util.actioncommand import stand, standStraight

from BehaviourTask import BehaviourTask


class Stand(BehaviourTask):

    def _tick(self, straight=False):
        self.world.b_request.actions.body = standStraight() if straight else stand()
