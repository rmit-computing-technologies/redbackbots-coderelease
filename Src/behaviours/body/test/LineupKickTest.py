from math import radians
from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Localise import Localise
from body.skills.Walk import Walk
from body.skills.LineupKick import LineupKick
from body.skills.Stand import Stand
from head.HeadAware import HeadAware
from util.GameStatus import enemy_goal
from util.Global import ballDistance, ballWorldPos, canSeeBall
from util import log
from util.Timer import Timer
from util.FieldGeometry import heading_error


class LineupKickTest(BehaviourTask):
    """
    Localise -> face target goal until a ball is seen -> LineupKick
    """
    TURN_RATE = 1.5
    HEADING_ERROR = radians(30)
    LINEUP_OFFSET = 800  # mm
    TIME_COVERSION = 1_000_000

    _facing_vector = enemy_goal()  # default to facing at the goal

    _lineup_timer = Timer()
    _kick_timer = Timer()

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Localise": Localise(self),
            "WalkToBall": WalkToPoint(self),
            "FacePoint": Walk(self),  # for turning
            "LineupKick": LineupKick(self),
            "Stand": Stand(self),
        }


    def _transition(self):
        # stay standing if already kicked on this test
        if self._test_complete:
            self._current_sub_task = "Stand"
            return


        # still needs to localise or ball is lost
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return
        

        # handle transitions from FacePoint
        if self._current_sub_task == "FacePoint":
            # turn until facing the expected ball position
            if abs(heading_error(self._facing_vector)) > self.HEADING_ERROR:
                return
            
            # stand when facing the expected ball position
            else:
                log.debug("Standing and waiting for a ball.")
                self._current_sub_task = "Stand"
                return
                
        
        # handle transitions from Stand
        if self._current_sub_task == "Stand":
            # keep standing while a ball isn't seen
            if not canSeeBall():
                return
            
            # once a ball is seen, walk to its position in prep for LineupKick
            else:
                log.debug("Ball has been seen. Walking there.")
                self._current_sub_task = "WalkToBall"
                return


        # handle transitions from WalkToBall
        if self._current_sub_task == "WalkToBall":
            # continue walking to ball if not close or within LINEUP_OFFSET
            if not self._sub_tasks[self._current_sub_task]._position_close \
                and ballDistance() > self.LINEUP_OFFSET:
                return
            
            # when within range, transition to LineupKick and begin the _lineup_timer
            else:
                log.debug("Switching to LineupKick.")
                self._current_sub_task = "LineupKick"
                self._lineup_timer.restart().start()
                return
            
        
        # handle transitions from and within LineupKick
        if self._current_sub_task == "LineupKick":
            # handle the Kick subtask within LineupKick
            if self._sub_tasks[self._current_sub_task]._current_sub_task == "Kick":
                # on the first tick of LineupKick->Kick, output the 
                # _lineup_timer elapsed time and start the _kick_timer
                if self._is_initial_kick_transition:
                    log.info(f"Lineup time: {self._lineup_timer.elapsed()/self.TIME_COVERSION:.2f}")
                    self._kick_timer.restart().start()
                    self._sub_tasks[self._current_sub_task]._sub_tasks["Kick"].reset()  # reset Kick so it doesn't phantom kick
                    self._is_initial_kick_transition = False
                    return

                # once the LineupKick->Kick is complete, end this test instance
                if self._sub_tasks[self._current_sub_task]._sub_tasks["Kick"].is_finished:
                    log.info(f"Kick time: {self._kick_timer.elapsed()/self.TIME_COVERSION:.2f}")
                    self._test_complete = True
                    return
            
            return  # don't transition from LineupKick until _test_complete

        # only accessed on first run i.e. entry point
        self._current_sub_task = "FacePoint"


    def _reset(self):
        self._current_sub_task = "Localise"
        self._test_complete = False
        self._is_initial_kick_transition = True


    def _tick(self, facing_vector=_facing_vector):

        # tick FacePoint to turn to face the facing_vector param
        if self._current_sub_task == "FacePoint":
            self._tick_sub_task(turn=self.TURN_RATE if heading_error(facing_vector) > 0
                else -self.TURN_RATE)

        # set the ball's location as WalkToBall target (will be interrupted by LineupKick)
        elif self._current_sub_task == "WalkToBall":
            ball_pos = ballWorldPos()
            self._tick_sub_task(final_pos=ball_pos,
                                speed=0.2, 
                                prevent_leaving_field=False)

        else:
            self._tick_sub_task()


    # can be called by extensions to identify the end of the test
    def is_test_complete(self):
        return self._test_complete
    