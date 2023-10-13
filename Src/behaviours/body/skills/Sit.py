from util.actioncommand import sit

from BehaviourTask import BehaviourTask

class Sit(BehaviourTask):

    def _tick(self):
        self.world.b_request.actions.body = sit()