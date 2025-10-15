from BehaviourTask import BehaviourTask
from body.skills.LineupKick import LineupKick
from body.skills.Dribble import Dribble

from util.TeamStatus import get_active_player_numbers, my_player_number, pose_of_player_number, player_number_is_incapacitated
from util.Global import myPos, myHeading, ball_rel_pos
from util.GameStatus import enemy_goal
from util.Constants import ROBOTS_PER_TEAM
from util import EventComms
from util import log

from util.Timer import Timer
from util.Vector2D import Vector2D, makeVector2DFromDistHeading


class LineupPass(BehaviourTask):

    DEFAULT_RANGE = 200  #50
    target_robot = None # Default to passing to the closest robot
    target_position = None

    SHOULD_PASS = False

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "PassBall": LineupKick(self),
            "Dribble": Dribble(self),
        }

    def _reset(self):
        if self.SHOULD_PASS:
            self._current_sub_task = "PassBall"
            self.passing_alignment = PassingAlignmentVectors(target_robot=self.target_robot)

            if self.target_robot is None:
                self.target_robot, self.target_position = self.passing_alignment.get_best_robot()

            else:
                self.target_position = self.passing_alignment.get_target_robot_pos()
        
        else:
            self._current_sub_task = "Dribble"

    def _transition(self):
        # TODO: If we are passing to a specific robot then we should tell the robot to align with us
        if self.SHOULD_PASS:
            # Set the target robot to the closest and best-aligned robot
            if self.target_robot is not None:
                self.target_position = pose_of_player_number(self.target_robot)[0]
                self._current_sub_task = "PassBall"

                log.info(f"{my_player_number()} passing to {self.target_robot}", say=True)

            # If there is no robot to pass to, we will dribble the ball towards the enemy goal
            else:
                self.target_robot, self.target_position = self.passing_alignment.get_best_robot()
                self._current_sub_task = "Dribble"
        
        else:
            self._current_sub_task = "Dribble"

    def _tick(self, target_robot=None):
        if target_robot is not None:
            if target_robot != my_player_number() and target_robot != 1 and not player_number_is_incapacitated(target_robot):
                self.target_robot = target_robot
            else:
                log.warning(f"{my_player_number()} tried to pass to {target_robot}, but it is not a valid target.")

        if self._current_sub_task == "PassBall":
            self._tick_sub_task(target=self.target_position, range=self.DEFAULT_RANGE)
        else:
            self._tick_sub_task()


class PassingAlignmentVectors:
    """
    PassingAlignmentVectors is a class responsible for calculating alignment vectors for passing in a multi-robot system.
    It determines the best teammate to pass to based on distance, alignment, and robot status, and manages communication
    for passing requests.
    """

    # Time to wait before sending a passing request
    TIME_TO_SEND = 2.0
    TIME_WAIT_RESPONSE = 5.0
    TIME_ZERO = 0  # No magic numbers :)
    RESEND_REQUEST_TIMEOUT = 5.0
    ERROR_THRESHOLD = 2.00
    SEND_TO_FULL_TEAM = 0  # If this is sent instead of a specific robot number then all team members will respond

    # Default/nonexistent robot data
    DEFAULT_ROBOT_DATA = (Vector2D(0, 0), 0)  # Default position and heading

    # Hysteresis margins
    DISTANCE_WEIGHT = 1.0
    PROGRESS_WEIGHT = 2.0
    ALIGNMENT_WEIGHT = 1.5
    HYSTERESIS_BONUS = 5.0  # Bias for keeping the current target

    def __init__(self, target_robot=None):
        self.target_robot = target_robot
        self.target_position = None
        self.use_specific_robot = False  # If True, only pass to the target robot
        self.robot_data = [None] * ROBOTS_PER_TEAM  # Store the robot data for each player
        self.response_wait_timer = Timer(target_seconds=self.TIME_WAIT_RESPONSE)

        if self.target_robot is not None:
            self.use_specific_robot = True

        # Get the positions and headings of the team
        self.read_in_robot_data()

    def get_best_robot(self):
        """
        Determines and returns the player number of the closest eligible robot to pass to, considering both distance and alignment.
        Returns None if no suitable robot is found.
        """
        self.read_in_robot_data()
        
        best_score = float("-inf")
        best_robot = None
        ideal_position = None

        for playerNum in range(2, ROBOTS_PER_TEAM + 1):  # Never pass to goalie
            current_robot = self.robot_data[playerNum - 1]
            
            if current_robot is not None \
                and not player_number_is_incapacitated(playerNum) \
                and playerNum != my_player_number():

                # Compute hysteresis score
                score, pass_target = self._apply_hysteresis(playerNum, current_robot)
                log.warning(f"Player {playerNum} score: {score}, pass target: {pass_target}")

                if score > best_score:
                    best_score = score
                    best_robot = playerNum
                    ideal_position = pass_target

        self.target_robot = best_robot
        self.target_position = ideal_position

        return self.target_robot, self.target_position
        
    def _apply_hysteresis(self, player_num, robot_pose):
        """
        Computes a hysteresis-based score for passing to a given robot.
        Note: This is chatGPT code, not mine. Wanted to see if it works or not.
        Returns (score, ideal_pass_position).
        """
        robot_pos, robot_heading = robot_pose

        # Distance to the robot
        distance = myPos().distanceTo(robot_pos)

        # Progress down the field (closer to enemy goal is better)
        field_direction = (enemy_goal() - myPos()).normalised()
        vector_to_robot = (robot_pos - myPos())
        progress = vector_to_robot.dot_product(field_direction)

        # Alignment: how well robot is facing the ball
        vector_robot_to_ball = (myPos() - robot_pos).normalised()
        robot_facing_vector = makeVector2DFromDistHeading(1, robot_heading)

        # alignment = 1 means facing ball, -1 means facing away
        alignment = robot_facing_vector.dot_product(vector_robot_to_ball)

        # Base score: weighted sum
        score = (-distance * self.DISTANCE_WEIGHT) + \
                (progress * self.PROGRESS_WEIGHT) + \
                (alignment * self.ALIGNMENT_WEIGHT)

        # Apply hysteresis bonus if this is the current target
        if self.target_robot == player_num:
            score += self.HYSTERESIS_BONUS

        # Ideal pass target is slightly ahead of the robot in its heading direction
        receive_offset = makeVector2DFromDistHeading(100, robot_heading)  # 100mm ahead
        ideal_pass_position = robot_pos + receive_offset

        return score, ideal_pass_position

    def read_in_robot_data(self):
        """
        Reads and updates the position and heading data of all active teammates (excluding the goalie and incapacitated robots).
        Also manages the sending of passing requests based on ball proximity and communication timeouts.
        """
        if self.response_wait_timer.finished() or not self.response_wait_timer.running:
            self._send_passing_request()
        
        for playerNum in range(2, ROBOTS_PER_TEAM + 1):  # Never pass to goalie
            last_received = EventComms.seconds_since_received_by_player_event(playerNum, "POSITION_UPDATE")

            if playerNum in get_active_player_numbers() \
                and not player_number_is_incapacitated(playerNum) \
                and playerNum != my_player_number():

                if last_received is not None and self.TIME_ZERO <= last_received <= self.RESEND_REQUEST_TIMEOUT:
                    # Get the other robot's information from the received events
                    other_robot_position, other_robot_heading = pose_of_player_number(playerNum)
                    self.robot_data[playerNum - 1] = (other_robot_position, other_robot_heading)

    def _send_passing_request(self):
        """
        Sends a passing request event if the robot is near the ball and the request timeout has elapsed.

        If a target robot is set, it will only send the request to that robot
        """

        self.response_wait_timer.restart().start()
        last_pass_request = EventComms.seconds_since_received_by_player_event(my_player_number(), "PASSING_REQUEST")

        if last_pass_request is None or last_pass_request > self.RESEND_REQUEST_TIMEOUT:
            if self.use_specific_robot:
                EventComms.raise_event("PASSING_REQUEST", self.target_robot, self.TIME_TO_SEND)
            else:
                EventComms.raise_event("PASSING_REQUEST", self.SEND_TO_FULL_TEAM, self.TIME_TO_SEND)

    def get_target_robot_pos(self):
        """
        Returns the position of the closest eligible robot to pass to, or None if no such robot exists.
        """
        if self.target_robot is not None:
            self.target_position = pose_of_player_number(self.target_robot)[0]

        return self.target_position
