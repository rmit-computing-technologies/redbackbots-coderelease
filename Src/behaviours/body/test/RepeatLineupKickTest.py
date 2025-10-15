from BehaviourTask import BehaviourTask
from body.skills.Localise import Localise
from body.skills.WalkToPoint import WalkToPoint
from body.test.LineupKickTest import LineupKickTest
from head.HeadAware import HeadAware
from util.Global import myPos
from util import log
from util.Timer import Timer


class RepeatLineupKickTest(BehaviourTask):
    """
    Mark start location -> LineupKickTest -> Return to marked location.
    """
    _test_pos = None
    _position_marked = False

    MICROSEC_TO_SEC = 1_000_000
    _localise_timer = Timer(10 * MICROSEC_TO_SEC)  # 10 second timer

    def _initialise_sub_tasks(self):        
        # Set sub-tasks
        self._sub_tasks = {
            "Localise": Localise(self),
            "WalkToOrigin": WalkToPoint(self),
            "LineupKickTest": LineupKickTest(self),
        }


    def _transition(self):
            
        # add a timer to force extra localisation
        if not self._localise_timer.finished():
            self._current_sub_task = "Localise"
            log.debug(f"Time elapsed: {self._localise_timer.elapsed()/self.MICROSEC_TO_SEC:.2f}")
            return


        # if the NAO is clueless as to its place in this world, localise
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return
        

        # handle the transition from LineupKickTest
        if self._current_sub_task == "LineupKickTest":
            # if the test is incomplete, don't transition
            if not self._sub_tasks[self._current_sub_task].is_test_complete():
                return

            # once the test is complete, reset the LineupKickTest variables
            # and begin walking to origin
            else:
                log.debug("LineupKickTest said it was done!")
                self._sub_tasks[self._current_sub_task].reset()

                # test done, walk back to origin
                self._current_sub_task = "WalkToOrigin"
                return


        # handle the transition from WalkToOrigin
        if self._current_sub_task == "WalkToOrigin":
            # if the walk is incomplete, don't transition
            if not self._sub_tasks[self._current_sub_task]._position_close:
                return
            
            # once the walk is completed, LineupKickTest can repeat
            else:
                log.debug("Back at the origin, restarting LineupKickTest")
                self._sub_tasks[self._current_sub_task].reset()
                self._current_sub_task = "LineupKickTest"
                return


        # entry point for the testing loop
        if not self._position_marked:
            # on first iteration, mark the test location and set flag to True
            self._test_pos = myPos()
            self._position_marked = True
            log.debug(f"Test position: ({self._test_pos.x:.2f}, {self._test_pos.y:.2f})")

            # start LineupKickTest
            self._current_sub_task = "LineupKickTest"
            return


    def _reset(self):
        self._current_sub_task = "Localise"
        self._localise_timer.restart().start()


    def _tick(self):
        # return to the test origin
        if self._current_sub_task == "WalkToOrigin":
            self._tick_sub_task(final_pos=self._test_pos, 
                                speed=0.2, 
                                prevent_leaving_field=False)

        else:
            self._tick_sub_task()