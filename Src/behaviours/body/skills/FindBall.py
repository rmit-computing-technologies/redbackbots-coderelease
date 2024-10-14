from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Turn import Turn
from util.Global import myPos
from util.Vector2D import Vector2D
from util.Timer import Timer
from util import Log

from util.Constants import GOAL_BOX_WIDTH, FIELD_LENGTH, FIELD_WIDTH, GOAL_BOX_LENGTH
from util.TeamStatus import player_role, my_player_number

from body.skills.Stand import Stand
from util.Hysteresis import Hysteresis
from util.Global import canSeeBall
from head.HeadAware import HeadAware

class FindBall(BehaviourTask):
    """
    Tries a bunch of techniques to find the ball if lost.
    First tries spinning around. If that doesn't work,
    tries walking to a bunch of points and doing the same thing.
    """
    # POSITIONS = [
    #     Vector2D(0, -1000),
    #     Vector2D(0, 0),
    #     Vector2D(0, 1000),
    #     Vector2D(0, 0)
    # ]

    POSITIONS = {
        "Attacker":[Vector2D(FIELD_LENGTH/2,0),
                    Vector2D(FIELD_LENGTH/2,-FIELD_WIDTH/4),
                    Vector2D(FIELD_LENGTH/2,0),
                    Vector2D(FIELD_LENGTH/2,FIELD_WIDTH/4)], # left side of the middle of the attacking half

        "Defender":[Vector2D(-FIELD_LENGTH/2,0),
                    Vector2D(-FIELD_LENGTH/2,FIELD_WIDTH/4),
                    Vector2D(-FIELD_LENGTH/2,0),
                    Vector2D(-FIELD_LENGTH/2,-FIELD_WIDTH/4)], # 

        "Midfielder":[Vector2D(-FIELD_LENGTH/3,0),
                    Vector2D(-FIELD_LENGTH/3,-FIELD_WIDTH/4),
                    Vector2D(-FIELD_LENGTH/3,0),
                    Vector2D(-FIELD_LENGTH/3,FIELD_WIDTH/4)], # left side of slightly attacking side of the defending half

        "Goalie":[Vector2D((-FIELD_LENGTH/2)+(GOAL_BOX_LENGTH/2),0),
                Vector2D((-FIELD_LENGTH/2)+(GOAL_BOX_LENGTH/2),GOAL_BOX_WIDTH/4),
                Vector2D((-FIELD_LENGTH/2)+(GOAL_BOX_LENGTH/2),0),
                Vector2D((-FIELD_LENGTH/2)+(GOAL_BOX_LENGTH/2),-GOAL_BOX_WIDTH/4)]# center of the goal box
    }

    # Turn time in seconds
    TURN_TIME = 4

    # Pause time to veriy ball sighting in seconds
    STAND_TIME = 2

    # Sensitive to pause and see if it can see the ball
    _ball_seen = Hysteresis(min_value=0, max_value=2)

    CLOSE_DISTANCE = 500
    NOT_CLOSE_DISTANCE = 1000

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Turn": Turn(self),
            "Stand": Stand(self)
        }

    def _transition(self):
        Log.debug(self.turn_timer.finished())
        Log.debug("TIME ON TURN: " + str(self.turn_timer.elapsed()))

        if not self.turn_timer.finished():
            Log.debug("TURN TIMER NOT FINISHED -> KEEP TURNING")
            # If timer finished and ball not found, go back to turn
            if self.stand_timer.finished() or not self._ball_seen.is_max():
                self._current_sub_task = "Turn"
                Log.debug("TURN")
            # If timer is running or the ball is confidently seen, stand
            else:
                self._current_sub_task = "Stand"
                Log.debug("STAND")
        else:
            self._current_sub_task = "WalkToPoint"
            Log.debug("WALK TO POINT")

            if self._position_close:
                self.turn_timer.restart().start()
                self._increment_chosen_pos() #TODO: MAY CAUSE ISSUES !!! -> Verify walk to points
                self._position_close = False

    def _reset(self):
        self._current_sub_task = "Stand" # Stand for two seconds on reset
        self._chosenPos = 0
        self._position_close = False
        self.turn_timer = Timer(1000000 * self.TURN_TIME)
        self.stand_timer = Timer(1000000 * self.STAND_TIME).start()

    def _tick(self):
        self.current_role = self.world.b_request.behaviourSharedData.role

        if self.current_role == 0:
            self.current_role = "Goalie"
        else:
            role_priority = ["Attacker", "Defender", "Midfielder"]
            self.current_role = role_priority[self.current_role - 1]
        # Tick hysteresis
        if canSeeBall():
            self._ball_seen.up()
        else:
            self._ball_seen.down()

        if self._current_sub_task == "Turn" and self._ball_seen.is_max():
            self._current_sub_task = "Stand"
            self.stand_timer.restart().start()

            # self._tick_sub_task() # Tick stand

        if self._current_sub_task == "WalkToPoint":

            if (not self._position_close and
                    self._pos_error_sq() < self.CLOSE_DISTANCE ** 2):
                self._position_close = True
            elif (self._position_close and
                  self._pos_error_sq() > self.NOT_CLOSE_DISTANCE ** 2):
                self._position_close = False

            self._tick_sub_task(final_pos=self.role_position_list[self._chosenPos])#, speed=0.2) #FIXME: DOES SPEED NEED TO BE THAT SLOW?
        else:
            self._tick_sub_task() #TODO: Should tick both stand and turn as neither is walktopoint

    def _increment_chosen_pos(self): # should flick between the positions availible as per the role of the robot
        self._chosenPos += 1
        if self._chosenPos >= len(self.POSITIONS[self.current_role]):
            self._chosenPos = 0

    def _pos_error_sq(self):
        self.role_position_list = self.POSITIONS[self.current_role]
        return self.role_position_list[self._chosenPos].minus(myPos()).length2()
