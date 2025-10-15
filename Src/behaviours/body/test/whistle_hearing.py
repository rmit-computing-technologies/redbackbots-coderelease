from util import led_override
from util.Timer import Timer

from util import log
from util.Constants import LEDColour
from util.GameStatus import whistle_detected
from util.actioncommand import stand, standStraight

from BehaviourTask import BehaviourTask


class whistle_hearing(BehaviourTask):

    _lightsTimer = Timer(5000000)

    def _tick(self, straight = False):
        #Used to test if the robot is able to hear whistles
        self.world.b_request.actions.body = standStraight() if straight else stand()

        if whistle_detected():
            log.info("Whistle has been heard")
            self._lightsTimer.start().restart()
            led_override.override(led_override.RIGHT_FOOT, LEDColour.red)
            led_override.override(led_override.LEFT_FOOT, LEDColour.red)
    
        if self._lightsTimer.finished():
            led_override.override(led_override.RIGHT_FOOT, LEDColour.off)
            led_override.override(led_override.LEFT_FOOT, LEDColour.off)

    def _reset(self):
        pass




    