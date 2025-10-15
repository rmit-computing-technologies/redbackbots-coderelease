from BehaviourTask import BehaviourTask
from body.skills.Localise import Localise
from body.test.LineupKickTest import LineupKickTest
from body.skills.WalkToPoint import WalkToPoint
from head.HeadAware import HeadAware
from util.Vector2D import Vector2D
from util import log
from util.Timer import Timer

from util.Constants import (
    CENTER_CIRCLE_DIAMETER,
    HALF_FIELD_LENGTH,
    HALF_FIELD_WIDTH,
    HALF_PENALTY_BOX_WIDTH,
    PENALTY_BOX_LENGTH,
)

# test position 1.1
PENALTY_BOX_SIDELINE_TOP = Vector2D(
    HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH, 
    HALF_PENALTY_BOX_WIDTH + (HALF_FIELD_WIDTH - HALF_PENALTY_BOX_WIDTH) / 2)
# test position 1.2
MID_PENALTY_BOX_OPP_TOP = Vector2D(
    (HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH) / 2, 
    HALF_PENALTY_BOX_WIDTH)
# test position 1.3
CENTER_CIRCLE_OPP = Vector2D(
    CENTER_CIRCLE_DIAMETER, 
    0)
# test position 1.4
MID_PENALTY_BOX_OPP_BOTTOM = Vector2D(
    (HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH) / 2, 
    -HALF_PENALTY_BOX_WIDTH)
# test position 1.5
PENALTY_BOX_SIDELINE_BOTTOM = Vector2D(
    HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH, 
    -(HALF_PENALTY_BOX_WIDTH + (HALF_FIELD_WIDTH - HALF_PENALTY_BOX_WIDTH) / 2))
# test position 1.6
MID_PENALTY_BOX_DEF_TOP = Vector2D(
    -(HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH) / 2, 
    HALF_PENALTY_BOX_WIDTH)
# test position 1.7
MID_PENALTY_BOX_DEF_BOTTOM = Vector2D(
    -(HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH) / 2, 
    -HALF_PENALTY_BOX_WIDTH)


class LineupKickTestSuite(BehaviourTask):
    """
    Go to each location and do a LineupKickTest.
    """
    MICROSEC_TO_SEC = 1_000_000
    _localise_timer = Timer(10 * MICROSEC_TO_SEC)  # 10 second timer
    
    _test_idx = 0
    _test_positions = [  # relative positions on the field
        PENALTY_BOX_SIDELINE_TOP,
        MID_PENALTY_BOX_OPP_TOP,
        CENTER_CIRCLE_OPP,
        MID_PENALTY_BOX_OPP_BOTTOM,
        PENALTY_BOX_SIDELINE_BOTTOM,
        MID_PENALTY_BOX_DEF_TOP,
        MID_PENALTY_BOX_DEF_BOTTOM
    ]


    def _initialise_sub_tasks(self):
        # Set sub-tasks
        self._sub_tasks = {
            "Localise": Localise(self),
            "WalkToNextTest" : WalkToPoint(self),
            "LineupKickTest": LineupKickTest(self),
        }


    def _transition(self):
   
        # add a timer to force extra localisation
        if not self._localise_timer.finished():
            self._current_sub_task = "Localise"
            log.debug(f"Time elapsed: {self._localise_timer.elapsed()/self.MICROSEC_TO_SEC:.2f}")
            return
        

        # requires localisation
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return
        

        # handle the transition from LineupKickTest
        if self._current_sub_task == "LineupKickTest":
            # if the test is incomplete, don't transition
            if not self._sub_tasks[self._current_sub_task].is_test_complete():
                return

            # once the test is complete, reset the LineupKickTest variables
            # and begin walking to next test
            else:
                log.debug("LineupKickTest said it was done!")
                self._sub_tasks[self._current_sub_task].reset()

                if self._test_idx < len(self._test_positions) - 1:
                    self._test_idx += 1  # move to next test location
                else :
                    self._test_idx = 0  # return to first test location

                # test done, walking to next test
                self._current_sub_task = "WalkToNextTest"
                return


        # handle the transition from WalkToNextTest
        if self._current_sub_task == "WalkToNextTest":
            # if the walk is incomplete, don't transition
            if not self._sub_tasks[self._current_sub_task]._position_close:
                return
            
            # once the walk is completed, LineupKickTest can repeat
            else:
                log.debug("Arrived at test location, restarting LineupKickTest")
                self._sub_tasks[self._current_sub_task].reset()  # this sets _position_close back to false
                self._current_sub_task = "LineupKickTest"
                return


        # entry point
        self._current_sub_task = "WalkToNextTest"

    def _reset(self):
        self._current_sub_task = "Localise"
        self._localise_timer.restart().start()


    def _tick(self):
        next_test_pos = self._test_positions[self._test_idx]

        # while waiting for a ball to be placed, face the point a ball is expected
        if self._current_sub_task == "LineupKickTest":
            # ideally this would stand offset from the ball position, then be 
            # facing the expected ball pos, but for now just stand on the position 
            # and use default behaviour which is face goal.
            self._tick_sub_task()
            # self._tick_sub_task(facing_vector=next_test_pos)

        # walk to the next location in the suite 
        elif self._current_sub_task == "WalkToNextTest":
            self._tick_sub_task(final_pos=next_test_pos,
                                speed=0.2, 
                                prevent_leaving_field=False)

        else:
            self._tick_sub_task()