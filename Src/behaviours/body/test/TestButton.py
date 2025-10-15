from util.actioncommand import stand
from util import led_override
from util.Constants import LEDColour
from util import log

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


        # log.debug("Left foot:", lfoot)
        # log.debug("Right foot:", rfoot)


        # log.debug("Head front:", fhead)
        # log.debug("Head middle:", mhead)
        # log.debug("Head back:", bhead)

        # log.debug(sensorValues)

        if lfoot:
            led_override.override_eye_segment(led_override.RIGHT_EYE, [0,1,2,3], LEDColour.red)
            log.info("Left foot", say=True)
        if rfoot:
            led_override.override_eye_segment(led_override.RIGHT_EYE, [5,6,7], LEDColour.red)
            log.info("Right foot", say=True)
        if fhead:
            led_override.override_eye_segment(led_override.LEFT_EYE, [5,6,7], LEDColour.red)
            log.warning("Front head", say=True)
        if mhead:
            led_override.override_eye_segment(led_override.LEFT_EYE, [5,6,7], LEDColour.green)
            log.error("Middle head", say=True)
        if bhead:
            led_override.override_eye_segment(led_override.LEFT_EYE, [5,6,7], LEDColour.blue)
            log.critical("Back head", say=True)

        self.world.b_request.actions.body = stand()
