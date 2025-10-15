from math import radians
import robot

from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.InterceptBall import InterceptBall
from body.skills.Striker import Striker
from body.skills.Kickoff import Kickoff
from body.skills.Turn import Turn
from body.skills.TurnToBall import TurnToBall
from body.skills.TeamFindBall import TeamFindBall
from body.skills.FindBall import FindBall
from body.skills.Localise import Localise
from body.skills.DefenceAngle import DefenceAngle
from body.skills.KickOffDefence import KickOffDefence
from body.skills.Lineup import AlignmentVectors
from head.HeadAware import HeadAware
from util import led_override

from util.Vector2D import Vector2D
from util.Constants import HALF_FIELD_LENGTH, HALF_FIELD_WIDTH, CENTER_CIRCLE_DIAMETER, GOAL_POST_ABS_Y, LEDColour
from util.GameStatus import (
    ball_world_pos, 
    own_goal, 
    we_are_kicking_team, 
    in_kick_in, 
    in_pushing_free_kick, 
    in_corner_kick, 
    in_goal_kick, 
    in_penalty_kick, 
    whistle_detected, 
    prev_game_state, 
    GameState)
from util.TeamStatus import get_active_player_numbers, time_scaled_ball_score_best, robot_can_score
from util.Global import reset_look_target, ball_distance, is_ball_lost, proportion_of_balls_seen, close_to_position, not_close_to_position, ego_ball_lost, has_touched_ball, is_robot_in_box, is_ball_in_box, ego_see_ball, team_ball_lost, ball_seen_in_last_n_frames
from util.FieldGeometry import in_role_position
from util.Timer import Timer
from util import log, led_override

class RoleSelector(BehaviourTask):

    """ 
        Description:
        This class selects which behaviour the robot should follow given the position of themsleves
        teammates and the ball.
    """

    DEFAULT_QUADRANT = {'north': HALF_FIELD_WIDTH, 'east': HALF_FIELD_LENGTH, 'south': -HALF_FIELD_WIDTH, 'west': -HALF_FIELD_LENGTH}

    CLOSE_DISTANCE = 50  # mm
    NOT_CLOSE_DISTANCE = 500  # mm

    HEADING_ERROR = radians(5)

    TURN_RATE = 1.5

    # Factor to change the ball score based on the heading error to lineup to the kick target (currently for the target of the current lineup)
    BALL_SCORE_HEADING_FACTOR = 1

    # Proportionally add this if not seen the ball for BALL_SEEN_BUFFER_SIZE frames seen the ball, add this offset to the ball score
    BALL_SCORE_BALL_LOST_OFFSET = 500 #(Should reduce misslocalised robots having good ball scores from team ball)

    # Will increase the ball score by a % of the ball seen within the last X frames
    # Higher = longer delay in self confidence if seen the ball
    BALL_SEEN_BUFFER_SIZE = 40

    # Large penalty added to robots that have already touched the ball
    TOUCHED_BALL_PENALTY = CENTER_CIRCLE_DIAMETER * 0.75

    # When the robot loses the ball it alternate between turning for 5 seconds and walking back to its default position for 10 seconds
    # This is to ensure the robot does not walk past the ball when returning to its default posistion
    LOST_BALL_TURN_TIME = 5 # seconds

    KICK_OFF_TIME = 10 # seconds

    # How far away from the ball should the supporter go when denied striker
    SUPPORTER_AVOID_RADIUS = CENTER_CIRCLE_DIAMETER * 0.5 #TODO HOME: Changed to be closer (THIS IS LARGE ON COMP FIELD)

    ROLE_COLOURS = {
        "WalkToPoint": LEDColour.orange,
        "InterceptBall": LEDColour.cyan,
        "Striker": LEDColour.yellow,
        "TurnToBall": LEDColour.dark_green,
        "FindBall": LEDColour.pink,
        "TeamFindBall": LEDColour.pink,
        "Turn": LEDColour.green, 
        "Localise": LEDColour.white,
        "DefenceAngle": LEDColour.purple,
        "Supporter": LEDColour.blue,
        "KickOffDefence": LEDColour.dark_rose,
        "Kickoff": LEDColour.dim_white
    }

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "InterceptBall": InterceptBall(self),
            "Striker": Striker(self),
            "TurnToBall": TurnToBall(self),
            "TeamFindBall": TeamFindBall(self),
            "FindBall": FindBall(self),
            "Turn": Turn(self),
            "Localise": Localise(self),
            "DefenceAngle": DefenceAngle(self),
            "Supporter": InterceptBall(self),
            "KickOffDefence": KickOffDefence(self),
            "Kickoff": Kickoff(self)
        }

    def _transition(self):

        if HeadAware.high_uncertainty.is_max() and not ball_seen_in_last_n_frames(60):
            self._current_sub_task = "Localise"

        elif team_ball_lost() and proportion_of_balls_seen(60) <= 0.5:
           self._current_sub_task = "TeamFindBall"

        elif in_penalty_kick():
            if we_are_kicking_team() and \
              (self.field_position == ("Upfielder", "Center") or self.field_position == ("Attacker", "Center") or self.field_position[0] == "Superstar" or \
              ball_distance() < CENTER_CIRCLE_DIAMETER):
                self._current_sub_task = "Striker"
            else:
                self._current_sub_task = "TurnToBall"
        
        elif self.defense_kick_off_timer.running and not self.defense_kick_off_timer.finished() and self.ball_left_center == False:
            self._current_sub_task = "KickOffDefence"

        # If enemy team is kicking in robot will avoid or intercept the ball
        elif (in_kick_in() or in_pushing_free_kick() or in_corner_kick() or in_goal_kick()) and not we_are_kicking_team():
            if self._ball_in_allowed_area or ball_distance() < 2*CENTER_CIRCLE_DIAMETER:
                self._current_sub_task = "InterceptBall"
            else:
                self._current_sub_task = "DefenceAngle"
        elif (in_kick_in() or in_pushing_free_kick() or in_corner_kick() or in_goal_kick()) and we_are_kicking_team():
            self._current_sub_task = "Striker"

        elif self.can_go_into_striker():
            # TODO: ignore can score in striker.py
            # TODO HOME: HOME TEAM TEST THIS
            # if self.attack_kick_off_timer.running and not self.attack_kick_off_timer.finished() and self.ball_left_center == False:
            #     self._current_sub_task = "Kickoff"
            # else:
            self._current_sub_task = "Striker"

        elif (self.field_position[0] == "Defender" or self.field_position[0] == "Midfielder") and ball_world_pos().x > self.max_positions['east']:
            self._current_sub_task = "DefenceAngle"

        elif self.lost_ball_turn_timer.running and not self.lost_ball_turn_timer.finished():
            self._current_sub_task = "TurnToBall"

        elif self._position_close:
            self._current_sub_task = "TurnToBall"

        else:
            self._current_sub_task = "WalkToPoint"

    def _reset(self):
        # Reset Eye Colours
        led_override.override_eye_segment(
            led_override.RIGHT_EYE,
            led_override.ROLE_SEGMENTS,
            LEDColour.off
        )

        self._current_sub_task = "WalkToPoint"
        self._position_close = False
        self._ball_in_allowed_area = False
        self.max_positions = self.DEFAULT_QUADRANT
        self.field_position = ("Superstar", None)
        self.ball_left_center = False

        self.lost_ball_turn_timer = Timer(1000000 * self.LOST_BALL_TURN_TIME)
        self.defense_kick_off_timer = Timer(1000000 * self.KICK_OFF_TIME)
        self.attack_kick_off_timer = Timer(1000000 * self.KICK_OFF_TIME)

    def _tick(self, field_position = None, default_position = Vector2D(0, 0), max_positions = None):
        led_override.override_eye_segment(led_override.RIGHT_EYE, led_override.ROLE_SEGMENTS, self.ROLE_COLOURS[self._current_sub_task])

        # Reset look target
        if self._current_sub_task != "TeamFindBall":
            reset_look_target()

        # process timers
        if self.lost_ball_turn_timer.running and self.lost_ball_turn_timer.finished():
            self.lost_ball_turn_timer.restart().stop()

        # Start kick off timer when starting role selector if whistle has been heard
        # `prev game state == set` ensures that the whistle detected isnt from a goal whistle
        if whistle_detected() and prev_game_state() == GameState.SET:
            if we_are_kicking_team() and not self.attack_kick_off_timer.running:
                self.attack_kick_off_timer.restart().start()
            elif not self.defense_kick_off_timer.running:
                self.defense_kick_off_timer.restart().start()

        # Disable kickoff defence if ball leaves the center circle
        if (ball_world_pos().x > CENTER_CIRCLE_DIAMETER * 1.1 or ball_world_pos().x < -CENTER_CIRCLE_DIAMETER * 1.1 or \
            ball_world_pos().y > CENTER_CIRCLE_DIAMETER * 1.1 or ball_world_pos().y < -CENTER_CIRCLE_DIAMETER * 1.1) and \
            self.ball_left_center == False and self.defense_kick_off_timer.running and not self.defense_kick_off_timer.finished():
            self.ball_left_center = True

        # default max position is entire field
        if max_positions is not None:
            self.max_positions = max_positions
        if field_position is not None:
            self.field_position = field_position

        if in_role_position(ball_world_pos(), self.max_positions['north'], self.max_positions['east'], self.max_positions['south'], self.max_positions['west']):
            self._ball_in_allowed_area = True
        else:
            self._ball_in_allowed_area = False

        if (not self._position_close and close_to_position(default_position)):
            self._position_close = True
        elif (self._position_close and not_close_to_position(default_position)):            
            self._position_close = False

        if self._current_sub_task == "Striker":
            self.world.b_request.behaviourSharedData.playingBallScore = self.get_ball_score()
            # log.debug(f"Updated playingBallScore: {self.world.b_request.behaviourSharedData.playingBallScore}")

            # TODO HOME: Home team test that this leans on the side of OVER engaging the ball. One robot should prettymuch never be with the ball and be overtaken by another robot that steals the ball
            if ball_distance() < CENTER_CIRCLE_DIAMETER:
                if (time_scaled_ball_score_best(self.get_ball_score())) or (self.field_position[0] == "Superstar") or is_ball_in_box()[0] or is_robot_in_box()[0]: # TODO: Do not override the mine call of another robot:
                    self.world.b_request.behaviourSharedData.playingBall = True
                    led_override.override(led_override.LEFT_FOOT, LEDColour.yellow)
                else:
                    # If not the best to play ball, go supporter, TODO: if three robots wanted to go into striker,
                    # the two intercepting may overrun each other as they will both run intercept ball
                    self._current_sub_task = "Supporter"
                    self.world.b_request.behaviourSharedData.playingBall = False
                    # log.debug("Denied", say=False)
                    led_override.override(led_override.LEFT_FOOT, LEDColour.cyan)
            else:
                self._current_sub_task = "Striker"
                led_override.override(led_override.LEFT_FOOT, LEDColour.green)

        else:
            self.world.b_request.behaviourSharedData.playingBall = False

        reference_point = own_goal()
        if self._current_sub_task == "KickOffDefence":
            self._tick_sub_task(reference_point = reference_point, field_position = field_position)
            
        elif self._current_sub_task == "TeamFindBall":
            self._tick_sub_task(max_positions = max_positions, default_position = default_position)
        elif self._current_sub_task == "DefenceAngle":
            if field_position[1] != None:
                # Like in desmos simulation, use goal posts for left or right defenders
                if field_position[1] == 'Left':
                    reference_point.y = GOAL_POST_ABS_Y
                elif field_position[1] == 'Right':
                    reference_point.y = -GOAL_POST_ABS_Y
            self._tick_sub_task(reference_point = reference_point, default_x = default_position.x)
        elif self._current_sub_task == "Supporter":
            if field_position[1] != None:
                # Only used when denied striker
                # Use goal posts for left or right defenders
                if field_position[1] == 'Left':
                    reference_point.y = GOAL_POST_ABS_Y
                elif field_position[1] == 'Right':
                    reference_point.y = -GOAL_POST_ABS_Y

            self._tick_sub_task(reference_point = reference_point, avoid_radius=self.SUPPORTER_AVOID_RADIUS)
        elif self._current_sub_task == "InterceptBall":
            if field_position[1] == 'Left':
                reference_point.y = GOAL_POST_ABS_Y
            elif field_position[1] == 'Right':
                reference_point.y = -GOAL_POST_ABS_Y
            self._tick_sub_task(reference_point = reference_point)

        elif self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=default_position, speed=0.2)

        elif self._current_sub_task == "Striker":
            self.lost_ball_turn_timer.restart().start()
            self._tick_sub_task(field_position = self.field_position)
        else:
            self._tick_sub_task()

    def can_go_into_striker(self):
        # Prevent going into striker if we are in goal kick and goalie is active
        return (
            not ego_ball_lost() and \
            (self.field_position[0] == "Superstar" or (self._ball_in_allowed_area or ball_distance() < 2*CENTER_CIRCLE_DIAMETER))
        )

    def get_ball_score(self) -> float:
        """
        Calculate the "ball score" based on various factors including distance to the ball, 
        heading error, ball lost time, and proportion of ball seen in the last frames.

        The lower the ball score -> the better the robot is to play the ball. (Like golf)

        Returns:
            float: The calculated ball score.
        """

        # Run debug prints
        DEBUG = False

        # Calculate distance score
        dist_score = ball_distance()

        # Calculate heading factor
        # How lined up the robot is with the ball
        # Higher lineup error = worse score
        heading_factor = 1 + abs(AlignmentVectors().get_lineup_error() * self.BALL_SCORE_HEADING_FACTOR)

        # Calculate ball seen additional offset
        ball_lost_offset = (1 - proportion_of_balls_seen(self.BALL_SEEN_BUFFER_SIZE)) * self.BALL_SCORE_BALL_LOST_OFFSET

        touched_ball_penalty = self.TOUCHED_BALL_PENALTY if has_touched_ball() and not robot_can_score() else 0

        # Calculate final ball score
        ball_score = dist_score * heading_factor + ball_lost_offset + touched_ball_penalty

        if DEBUG:
            log.info(f"Ball Score: {ball_score}")
            log.debug(f"Distance Score: {dist_score}")
            log.debug(f"Heading Factor: {heading_factor}")
            log.debug(f"Ball Lost Offset: {ball_lost_offset}")
            log.debug(f"{ball_score} = {dist_score} * {heading_factor} + {ball_lost_offset}")

        return max(ball_score, 0)
