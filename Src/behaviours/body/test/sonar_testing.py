from util import log, led_override
from util.Constants import LEDColour
from util.ObstacleAvoidance import lfoot_bumper_clear_seconds, rfoot_bumper_clear_seconds
from util.Sonar import hasNearbySonarObject, LEFT, RIGHT

from util.actioncommand import stand, standStraight

from BehaviourTask import BehaviourTask

class sonar_testing(BehaviourTask):

    """   

    Possible logic implementations: 
        If both the bumper and sonar have found an object -> definitely an obstacle
        If bumper has hit an object -> Check for multiple hits -> If multiple on same foot found -> found an obstacle
        If sonar detects an object -> use default avoidance

    """

    _SONAR_CLEAR_MINIMUM_SECONDS = 0.2
    _BUMPER_CLEAR_MINIMUM_SECONDS = 1.0

    def _tick(self, straight = False):
        #   Used to test whether a sonar works
        #   If an obstacle is detected that side's foot will light up a colour depending on input

        #   YELLOW FOOT LIGHT ---> Sonar has detected an obstacle
        #   ORANGE FOOT LIGHT ---> Foot Bumper has hit an obstacle
        #   GREEN FOOT LIGHT ---> Both the Bumper and sensor have found an obstacle   

        self.world.b_request.actions.body = standStraight() if straight else stand()

        rightBumper = rfoot_bumper_clear_seconds() < self._BUMPER_CLEAR_MINIMUM_SECONDS
        leftBumper = lfoot_bumper_clear_seconds() < self._BUMPER_CLEAR_MINIMUM_SECONDS

        rightSonar = hasNearbySonarObject(RIGHT)
        leftSonar = hasNearbySonarObject(LEFT)

        if leftSonar or leftBumper:
            if leftSonar and leftBumper:
                log.debug("Obstacle detected LEFT")
                led_override.override(led_override.LEFT_FOOT, LEDColour.dark_green)

            elif leftSonar:
                log.debug("Obstacle detected SONAR LEFT")
                led_override.override(led_override.LEFT_FOOT, LEDColour.yellow)

            elif leftBumper:
                log.debug("Obstacle detected BUMPER LEFT")
                led_override.override(led_override.LEFT_FOOT, LEDColour.orange)

        else:
            led_override.override(led_override.LEFT_FOOT, LEDColour.off)


        if rightSonar or rightBumper:
            if rightSonar and rightBumper:
                log.debug("Obstacle detected RIGHT")
                led_override.override(led_override.RIGHT_FOOT, LEDColour.dark_green)

            elif rightSonar:
                log.debug("Obstacle detected SONAR RIGHT")
                led_override.override(led_override.RIGHT_FOOT, LEDColour.yellow)

            elif rightBumper:
                log.debug("Obstacle detected BUMPER RIGHT")
                led_override.override(led_override.RIGHT_FOOT, LEDColour.orange)

        else:
            led_override.override(led_override.RIGHT_FOOT, LEDColour.off)


    def _reset(self):
        pass




    