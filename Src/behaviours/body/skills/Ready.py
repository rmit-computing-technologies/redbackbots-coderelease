"""
Simple behaviour for the robot to stand in a ready position
at the start of the game or after a reset.

Initial positions for each role:
           REF
   6  4   2   3    5  6
 ┌──────────┬───────────┐
 ├────┐     │      ┌────┤
 ├─┐  │     │      │  ┌─┤
 │ │  │    ╭┴╮     │  │ │
 │ │  │    ╰┬╯     │  │ │
 ├─┘  │     │      │  └─┤
 ├────┘     │      └────┤
 └──────────┴───────────┘
   1  5   3   2    4  1
           G.C
"""
from math import radians

import util.led_override as led_override
from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand
from body.skills.Walk import Walk
from body.skills.WalkToPoint import WalkToPoint
from body.skills.WalkToGoalBox import WalkToGoalBox
from util.Global import close_to_position, not_close_to_position, myPos
from util.TeamStatus import my_player_number, get_active_player_numbers
from util.Vector2D import Vector2D
from util.RolePriority import RolePriority
from util import log
from util.Timer import Timer
from util.GameStatus import (
    own_goal,
    enemy_goal,
    we_are_kicking_team,
    in_penalty_kick,
    secondary_time,
    testing_with_half_field,
    just_outside_own_goal,
    enemy_goal_heading,
    enemy_goal_heading_backward,
    gc_game_state,
    GameState,
    prev_game_state
)
from util.Constants import (
    LEDColour,
    PENALTY_BOX_LENGTH,
    PENALTY_BOX_WIDTH,
    HALF_CENTER_CIRCLE_DIAMETER,
    CENTER_CIRCLE_DIAMETER,
    HALF_FIELD_WIDTH,
    QUARTER_FIELD_LENGTH,
    HALF_PENALTY_BOX_WIDTH,
    QUARTER_GOAL_BOX_WIDTH,
    HALF_FIELD_LENGTH,
    HALF_GOAL_BOX_WIDTH
)
from util import log

class Ready(BehaviourTask):
    """
    The Ready skill is used to position the robot in a ready position
    at the start of the game or after a reset.
    """
    GOALIE = [1, 20]

    HEADING_ERROR = radians(30)

    TURN_RATE = 1.5

    _position_close = False

    TESTING_WITH_HALF_FIELD = testing_with_half_field()

    KICKOFF_OFFSET = (HALF_CENTER_CIRCLE_DIAMETER * 2.0) / 5.0

    INI_POS = {}
    
    TIME_TO_WALK_IN = 5 # seconds

    def _calcuate_init_positions(self):
        OWN_GOAL_X = self.OWN_GOAL.x
        OWN_GOAL_POSITIVE = OWN_GOAL_X >= 0

        PENALTY_LINE = self.ENEMY_GOAL.x - PENALTY_BOX_LENGTH
        HALF_BETWEEN_PENALTY_AND_CENTER = (HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH) / 2.0

        if we_are_kicking_team():
            UPFIELDER_X = self.KICKOFF_OFFSET  if OWN_GOAL_POSITIVE else -self.KICKOFF_OFFSET
            UPFIELDER_Y = self.KICKOFF_OFFSET

            ATTACKER_X = self.KICKOFF_OFFSET  if OWN_GOAL_POSITIVE else -self.KICKOFF_OFFSET
            ATTACKER_Y = -self.KICKOFF_OFFSET

            MIDFIELDER_X = HALF_BETWEEN_PENALTY_AND_CENTER  if OWN_GOAL_POSITIVE else -HALF_BETWEEN_PENALTY_AND_CENTER
            MIDFIELDER_Y = -HALF_PENALTY_BOX_WIDTH

            DEFENDER_X = HALF_BETWEEN_PENALTY_AND_CENTER  if OWN_GOAL_POSITIVE else -HALF_BETWEEN_PENALTY_AND_CENTER
            DEFENDER_Y = HALF_PENALTY_BOX_WIDTH

            DEFENDER_LEFT_X = QUARTER_FIELD_LENGTH if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
            DEFENDER_LEFT_Y = QUARTER_GOAL_BOX_WIDTH

            DEFENDER_RIGHT_X = QUARTER_FIELD_LENGTH  if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
            DEFENDER_RIGHT_Y = -QUARTER_GOAL_BOX_WIDTH
        else:
            UPFIELDER_X = CENTER_CIRCLE_DIAMETER  if OWN_GOAL_POSITIVE else -CENTER_CIRCLE_DIAMETER
            UPFIELDER_Y =  HALF_GOAL_BOX_WIDTH - self.KICKOFF_OFFSET

            ATTACKER_X = CENTER_CIRCLE_DIAMETER  if OWN_GOAL_POSITIVE else -CENTER_CIRCLE_DIAMETER
            ATTACKER_Y = HALF_GOAL_BOX_WIDTH - self.KICKOFF_OFFSET

            MIDFIELDER_X = CENTER_CIRCLE_DIAMETER  if OWN_GOAL_POSITIVE else -CENTER_CIRCLE_DIAMETER
            MIDFIELDER_Y =  -HALF_GOAL_BOX_WIDTH +  self.KICKOFF_OFFSET

            DEFENDER_X = QUARTER_FIELD_LENGTH  if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
            DEFENDER_Y = -QUARTER_GOAL_BOX_WIDTH

            DEFENDER_LEFT_X = PENALTY_LINE if OWN_GOAL_POSITIVE else -PENALTY_LINE
            DEFENDER_LEFT_Y = QUARTER_GOAL_BOX_WIDTH 

            DEFENDER_RIGHT_X = PENALTY_LINE if OWN_GOAL_POSITIVE else -PENALTY_LINE
            DEFENDER_RIGHT_Y = -QUARTER_GOAL_BOX_WIDTH

        GOALIE_POS = just_outside_own_goal() # Goalie behaviour is defaulting to this spot

        # Updated positions depending if in penalty_kick or if we_are_kicking_team
        if in_penalty_kick():
            if we_are_kicking_team():
                # Upfielder or Attacker center to stand on penalty line
                UPFIELDER_X = -PENALTY_LINE if OWN_GOAL_POSITIVE else PENALTY_LINE
                UPFIELDER_Y = 0

                ATTACKER_X = -PENALTY_LINE if OWN_GOAL_POSITIVE else PENALTY_LINE
                ATTACKER_Y = 0

            else:
                # Defenders to move out of penalty box but inside field boundary
                DEFENDER_X = QUARTER_FIELD_LENGTH if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
                DEFENDER_Y = -QUARTER_GOAL_BOX_WIDTH

                DEFENDER_LEFT_X = QUARTER_FIELD_LENGTH if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
                DEFENDER_LEFT_Y = QUARTER_GOAL_BOX_WIDTH

                DEFENDER_RIGHT_X = QUARTER_FIELD_LENGTH if OWN_GOAL_POSITIVE else -QUARTER_FIELD_LENGTH
                DEFENDER_RIGHT_Y = -QUARTER_GOAL_BOX_WIDTH

                GOALIE_POS = Vector2D(OWN_GOAL_X, 0)

        self.INI_POS["UpfielderCenter"] = Vector2D(UPFIELDER_X, UPFIELDER_Y)
        self.INI_POS["MidfielderCenter"] = Vector2D(MIDFIELDER_X, MIDFIELDER_Y)
        self.INI_POS["DefenderLeft"] = Vector2D(DEFENDER_LEFT_X, DEFENDER_LEFT_Y)
        self.INI_POS["DefenderRight"] = Vector2D(DEFENDER_RIGHT_X, DEFENDER_RIGHT_Y)
        self.INI_POS["AttackerCenter"] = Vector2D(ATTACKER_X, ATTACKER_Y)
        self.INI_POS["DefenderCenter"] = Vector2D(DEFENDER_X, DEFENDER_Y)

        self.INI_POS["Goalie"] = GOALIE_POS

        self.INI_POS["Superstar"] = Vector2D(ATTACKER_X, ATTACKER_Y)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "CorrectDrift": WalkToPoint(self),
            "Walk": Walk(self),
            "FaceForward": Walk(self),
            "FaceBackward": Walk(self),
            "Stand": Stand(self),
            "WalkToGoalBox": WalkToGoalBox(self),
            "WalkStraightIn": Walk(self)
        }

    def _reset(self):
        # Start in Stand position to prevent walking in before the timer starts
        self._current_sub_task = "Stand"

        self.OWN_GOAL = own_goal()
        self.ENEMY_GOAL = enemy_goal()

        self.in_active_players = False
        self._heading_target = enemy_goal_heading()

        self._target_pos = Vector2D(0, 0)
        self.role_priority = RolePriority.current_priorities_for_ready()
        active_player_numbers = get_active_player_numbers()
        active_player_numbers = [x for x in active_player_numbers if x not in self.GOALIE]
        
        self._walk_timer = Timer(target_seconds=self.TIME_TO_WALK_IN).restart().start()

        self._calcuate_init_positions()

        self._current_role = "Superstar"  # Default role if not in active players

    def _transition(self):        
        if not self._walk_timer.running_finished() and prev_game_state() == GameState.STANDBY:
            self._current_sub_task = "WalkStraightIn"
            return
        # For when robot is picked up and/or removed from field
        self.role_priority = RolePriority.current_priorities_for_ready()
        active_player_numbers = get_active_player_numbers()
        active_player_numbers = [x for x in active_player_numbers if x not in self.GOALIE]
        
        if my_player_number() in self.GOALIE:
            self._current_role = "Goalie"
        else:
           self._current_role = ''.join(self.role_priority[active_player_numbers.index(my_player_number())])

        
        gc_state = gc_game_state()

        if gc_state == GameState.READY and secondary_time() < 2:
            # When secondary timer == 0, gc_game_state is SET
            # If the game is in SET or READY with less than 2 seconds remaining,
            # then we should not be walking
            self._current_sub_task = "Stand"
            return

        elif not self._position_close and self._current_sub_task != "CorrectDrift":
            # While secondary timer has not finished, WalkToPoint
            # If coming from playing, WalkTo
            self._current_sub_task = "WalkToPoint"

        elif not_close_to_position(self._target_pos):
            # Oh no we have drfted!
            log.warning(msg="drifted", say=False)
            self._position_close = False
            self._current_sub_task = "CorrectDrift"
        elif my_player_number() in self.GOALIE:
            self._current_sub_task = "WalkToGoalBox"
        # When robot is close to designated point, face goal.
        # Or if 4 seconds remaining and still attempting WalkToPoint, face goal.
        elif (my_player_number() not in self.GOALIE) or ( (gc_state == GameState.READY and secondary_time() < 4) and self._current_sub_task == "WalkToPoint"):
            #if we_are_kicking_team() and self._current_role in ["UpfielderCenter", "AttackerCenter", "Superstar"]:
            #    self._heading_target = enemy_goal_heading_backward()
            #else:
            self._heading_target = enemy_goal_heading()


            # If we are the kicking team and we are the center or left attacker, then face backward
            if abs(self._heading_target) < self.HEADING_ERROR:
                # If we are close to the goal heading, then we can stand
                self._current_sub_task = "Stand"
                return
            if we_are_kicking_team() and (
                    self._current_role == "UpfielderCenter" or
                    self._current_role == "AttackerCenter" or
                    self._current_role == "Superstar"
                ):
                # If we are the kicking team and we are the upfielder or attacker center,
                # then face backward
                self._current_sub_task = "FaceForward"
            else:
                # Otherwise, face forward
                self._current_sub_task = "FaceForward"
        else:
            self._current_sub_task = "WalkToPoint"
            log.error("Invalid Ready Transition", say=True)

    def _tick(self):
        if self._current_sub_task == "WalkStraightIn":
            self._tick_sub_task(forward=200)
        
        # TODO: For some reason midfield y occasionally is set wrong
        # temporarily calcuate init positions EVERY tick until we 
        # can work out what is actually going wrong
        
        self._calcuate_init_positions()
        led_override.override(led_override.CHEST_BUTTON, LEDColour.ready)

        active_player_numbers = get_active_player_numbers()
        active_player_numbers = [x for x in active_player_numbers if x not in self.GOALIE]

        if my_player_number() in self.GOALIE:
            self._target_pos = self.INI_POS["Goalie"]
        else:
            self._target_pos = self.INI_POS[self._current_role]
            log.debug(f"Target position for {self._current_role} is {self._target_pos}")
            if abs(self._target_pos.y) > HALF_FIELD_WIDTH:
                log.error(msg="Trying to set target pos to outside field", say=True)
                self._target_pos = self.INI_POS["Superstar"]

        if not self._position_close and close_to_position(self._target_pos):
            self._position_close = True
        elif self._position_close and not_close_to_position(self._target_pos):
            self._position_close = False

        if self._current_sub_task == "WalkToPoint":
            log.debug(f"Walking to {self._target_pos}")
            self._tick_sub_task(final_pos=self._target_pos, speed=0.2, final_heading=self._heading_target)
        elif self._current_sub_task == "CorrectDrift":
            log.warning("Correcting drift", say=False)
            # If we have drifted, then walk to the target position
            self._tick_sub_task(final_pos=self._target_pos, speed=0.2, always_look_at_final=True, final_heading=self._heading_target)
        elif self._current_sub_task == "FaceForward":
            self._tick_sub_task(
                turn=self.TURN_RATE if enemy_goal_heading() > 0 else -self.TURN_RATE
            )
        elif self._current_sub_task == "FaceBackward":
            self._tick_sub_task(
                turn=self.TURN_RATE if enemy_goal_heading_backward() > 0 else -self.TURN_RATE
            )
        else:
            self._tick_sub_task()
