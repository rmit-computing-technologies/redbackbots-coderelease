from BehaviourTask import BehaviourTask
from util.Hysteresis import Hysteresis
from body.skills.LineupKick import LineupKick
from body.skills.DiveLightGoalieAngle import GoalieAngle as DiveLightGoalieAngle
from body.skills.DiveHeavyGoalieAngle import GoalieAngle as DiveHeavyGoalieAngle
from body.skills.GoalieDive import GoalieDive
from body.skills.Stand import Stand
from body.skills.GoalieCornerKickIn import GoalieCornerKickIn
from util.Global import myPos, myHeading, is_ball_in_box, ego_see_ball, ego_ball_distance, ego_see_ball, get_goalie_stop_kick_in_check, set_goalie_stop_kick_in_check, num_balls_seen, proportion_of_balls_seen
from util.Constants import HALF_FIELD_LENGTH, GOAL_WIDTH, GOAL_POST_DIAMETER, FIELD_WIDTH, CENTER_CIRCLE_DIAMETER
from util.GameStatus import in_goal_kick, we_are_kicking_team, in_penalty_kick, in_kick_in, in_corner_kick
from util.BallMovement import YWhenReachCoronalPlane, YWhenReachOurGoalBaseLine, timeToReachOurGoalBaseLineNoFriction
from util.FieldGeometry import calculateTimeToReachPose
from util.Vector2D import Vector2D
from util.TeamStatus import get_active_player_numbers, my_player_number
from util import log

class Goalie(BehaviourTask):
    """
    The Goalie is responsible for defending the goal, determining when to dive, kick the ball away,
    or maintain positioning based on the current game context and ball position.

    Attributes:
        DIVE_CENTER_WIDTH (int): How wide a range the centre dive is able to block

    Decision Tree: 'Docs/behaviours/body/roles/Goalie.png'
    Markdown:      'Docs/behaviours/body/roles/Goalie.md'
    """

    # Robot interception range
    BALL_DISTANCE_DIVE_TRIGGER = HALF_FIELD_LENGTH
    GOALIE_WALK_TIME_BUFFER = 0.5   # in seconds
    GOALIE_DIVE_RANGE_BUFFER = 50  # in mm

    # DIVE PARAMS:
    DIVE_EGO_BALL_SEEN = { # number of ball frames seen before we can dive
        "LIGHT": 35,
        "HEAVY": 16
    }
    DIVE_BALL_VEL_HYSTERESIS = 7 # amount of frames in a row where the ball is coming to the goal before we can dive

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "DiveLightGoalieAngle": DiveLightGoalieAngle(self),
            "DiveHeavyGoalieAngle": DiveHeavyGoalieAngle(self),
            "AggressiveGoalieAngle": DiveLightGoalieAngle(self),
            "KickAway" : LineupKick(self),
            "GoalieDive": GoalieDive(self),
            "Stand": Stand(self),
            "GoalieCornerKickIn": GoalieCornerKickIn(self)
        }

    def dive_light_transition(self):
        active_field_player_numbers = get_active_player_numbers()
        time_to_reach_pos = calculateTimeToReachPose(
            myPos=myPos(),
            myHeading=myHeading(),
            targetPos=Vector2D(-HALF_FIELD_LENGTH, YWhenReachOurGoalBaseLine())
        )

        # If the ball will cross our base line before we can walk to it
        goalie_cant_reach_in_time = abs(timeToReachOurGoalBaseLineNoFriction()) < (time_to_reach_pos + self.GOALIE_WALK_TIME_BUFFER)
        # If the ball is rolling towards our goal
        ball_rolling_to_goal = timeToReachOurGoalBaseLineNoFriction() > 0
        # If this ball would enter our goal on the y axis
        ball_rolling_to_between_goal_posts = abs(YWhenReachOurGoalBaseLine()) < (GOAL_WIDTH+GOAL_POST_DIAMETER)/2
        ball_close_to_goalie = ego_ball_distance() < self.BALL_DISTANCE_DIVE_TRIGGER
        # If the ball is within diving range when it crosses the Goalie's Y axis
        ball_within_reach = abs(YWhenReachCoronalPlane()) < GoalieDive.DIVE_SIDE_WIDTH/2 + self.GOALIE_DIVE_RANGE_BUFFER
        # If the ball is within a certain distance
        is_in_penalty_box, _ = is_ball_in_box()
        is_ball_close = ego_ball_distance() < CENTER_CIRCLE_DIAMETER / 2 or is_in_penalty_box

        if in_goal_kick() and we_are_kicking_team():
            self._current_sub_task = "KickAway"
            return

        # If we can see the ball, we can determine if we need to dive or not
        # Also if ball close or within penalty - kick away
        if ego_see_ball():
            # Only dive if the ball had been seen for a while
            if (ego_see_ball(self.DIVE_EGO_BALL_SEEN["LIGHT"]) and
                goalie_cant_reach_in_time and
                ball_rolling_to_goal and
                ball_rolling_to_between_goal_posts and
                ball_close_to_goalie and
                ball_within_reach):
                self._current_sub_task = "GoalieDive"
                return

            # If ball is in front of goalie, or in the penalty box, kick it away,
            # but don't kick it away if the other team is kicking in
            if is_ball_close and (we_are_kicking_team() or not (in_penalty_kick() or in_kick_in())):
                log.debug("KickAway")
                self._current_sub_task = "KickAway"
                return

        # If the goalie is the only active player, move up to kick the ball away
        if len(active_field_player_numbers) == 1 and active_field_player_numbers[0] == my_player_number():
            self._current_sub_task = "AggressiveGoalieAngle"
            log.debug("Aggressive goalie activated")
            return

        if in_penalty_kick() and not we_are_kicking_team():
            self._current_sub_task = "Stand"
            return

        # Default Skill
        self._current_sub_task = "DiveLightGoalieAngle"

    def dive_heavy_transition(self):
        if in_goal_kick() and we_are_kicking_team():
            log.debug("Goal Kick")
            self._current_sub_task = "KickAway"
            return

        time_to_reach_pos = calculateTimeToReachPose(
                myPos=myPos(),
                myHeading=myHeading(),
                targetPos=Vector2D(-HALF_FIELD_LENGTH, YWhenReachOurGoalBaseLine())
            )
        ball_will_score = abs(YWhenReachOurGoalBaseLine()) < GOAL_WIDTH+GOAL_POST_DIAMETER/2
        time_to_base_line_is_valid = timeToReachOurGoalBaseLineNoFriction() > 0
        if time_to_base_line_is_valid:
            self.time_to_base_line_is_valid_count.up()
        else:
            self.time_to_base_line_is_valid_count.reset()
        will_reach_pos_in_time = time_to_reach_pos < abs(timeToReachOurGoalBaseLineNoFriction())
        is_in_penalty_box, _ = is_ball_in_box()
        is_ball_close = ego_ball_distance() < CENTER_CIRCLE_DIAMETER or is_in_penalty_box

        # If we can see the ball, we can determine if we need to dive or not
        # Also if ball close or within penalty - kick away
        if ego_see_ball(self.DIVE_EGO_BALL_SEEN["HEAVY"]):
            if ball_will_score and self.time_to_base_line_is_valid_count.is_max() and not will_reach_pos_in_time:
                log.debug("GoalieDive!!!")
                self._current_sub_task = "GoalieDive"
                return

            # If ball is in front of goalie, or in the penalty box, kick it away,
            # but don't kick it away if the other team is kicking in
            if is_ball_close and (we_are_kicking_team() or not (in_penalty_kick() or in_kick_in())):
                log.debug("KickAway")
                self._current_sub_task = "KickAway"
                return

        if in_penalty_kick() and not we_are_kicking_team():
            self._current_sub_task = "Stand"
        else:
            self._current_sub_task = "DiveHeavyGoalieAngle"

    def _transition(self):
        if in_corner_kick() and not we_are_kicking_team() and not get_goalie_stop_kick_in_check():
            # Transition for going to the corner of our field and kicking the ball away
            self._current_sub_task = "GoalieCornerKickIn"
            return
        elif not in_corner_kick() and get_goalie_stop_kick_in_check():
            # Reset the flag once the corner kick phase is over
            self._current_sub_task = "GoalieCornerKickIn"
            set_goalie_stop_kick_in_check(False)
            
        if self.world.blackboard.config['behaviour.goalie_strategy'] == "DiveLight":
            self.dive_light_transition()
        else:
            self.dive_heavy_transition()

    def _reset(self):
        self._current_sub_task = f"{self.world.blackboard.config['behaviour.goalie_strategy']}GoalieAngle"
        self.time_to_base_line_is_valid_count = Hysteresis(0, self.DIVE_BALL_VEL_HYSTERESIS)

    def _tick(self):
        if self._current_sub_task == "GoalieDive":
            final_y = YWhenReachOurGoalBaseLine()
            my_y = myPos().y

            robot_left_y = my_y + (GoalieDive.DIVE_CENTRE_WIDTH/2)
            robot_right_y = my_y - (GoalieDive.DIVE_CENTRE_WIDTH/2)

            if final_y < robot_left_y and final_y > robot_right_y:
                self._tick_sub_task(direction = "Center")
            elif final_y > my_y:
                self._tick_sub_task(direction = "Left")
            else:
                self._tick_sub_task(direction = "Right")
            return
        elif self._current_sub_task == "KickAway":
            self._tick_sub_task(range=FIELD_WIDTH/2)
            return
        elif self._current_sub_task == "AggressiveGoalieAngle":
            self._tick_sub_task(aggressive_goalie = True)

        self._tick_sub_task()
