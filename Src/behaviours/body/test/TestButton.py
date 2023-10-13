from util.actioncommand import stand

from BehaviourTask import BehaviourTask

from robot import Sensors

class TestButton(BehaviourTask):

    def _tick(self):
        sensorValues = self.world.blackboard.motion.sensors.sensors

        lfoot = sensorValues[Sensors.LFoot_Bumper_Right]
        rfoot = sensorValues[Sensors.RFoot_Bumper_Right]
        
        fhead = sensorValues[Sensors.Head_Touch_Front]
        mhead = sensorValues[Sensors.Head_Touch_Middle]
        bhead = sensorValues[Sensors.Head_Touch_Rear]


        print("Left foot:", lfoot)
        print("Right foot:", rfoot)


        print("Head front:", fhead)
        print("Head middle:", mhead)
        print("Head back:", bhead)

        print(sensorValues)

        self.world.b_request.actions.body = stand()
