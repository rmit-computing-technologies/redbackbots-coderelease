from util.actioncommand import stand
from util import LedOverride
from util.Constants import LEDColour
from util import Log

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


        # Log.debug("Left foot:", lfoot)
        # Log.debug("Right foot:", rfoot)


        # Log.debug("Head front:", fhead)
        # Log.debug("Head middle:", mhead)
        # Log.debug("Head back:", bhead)

        # Log.debug(sensorValues)

        if lfoot:
            LedOverride.override_eye_segment(LedOverride.rightEye, [0,1,2,3], LEDColour.red)
        if rfoot:
            LedOverride.override_eye_segment(LedOverride.rightEye, [5,6,7], LEDColour.red)

        self.world.b_request.actions.body = stand()
