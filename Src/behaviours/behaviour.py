"""
This is the main entry point to our Python application from PythonSkill.cpp,
where 'robot' is defined so 'import robot' does not fail with ImportError.

The `import robot` bridge from C++ to Python is defined by various Python
wrappers, run once at import time. Please see the `tick` function below for a
bridge from C++ to Python and back to C++ which is run many times at run time.

For example, for the implementation of `robot.TOP_IMAGE_ROWS`,
see the file `VisionDefinitions_wrap.cpp` in the following directory:
@see
Src/robot/src/perception/behaviour/python/wrappers/

For info, the following contains the C++ definition of `TOP_IMAGE_ROWS`:
@see
Src/robot/include/perception/vision/VisionDefinitions.hpp
"""

from pkgutil import iter_modules
from traceback import format_exc
from importlib import import_module
from os.path import join
from util import log
from robot import BehaviourRequest, All
from util import led_override
from util.world import World
from util.Global import update_global, ball_lost_time, believe_more_in_team_ball
from util.TeamStatus import update_team_status, robot_can_score
from util.FieldGeometry import update_field_geometry
from util.Timer import update_timer
from util.Sonar import update_sonar
from util.GameStatus import update_game_status, whistle_detected
from util.NetworkEar import update_network_ear, get_ear_colour
from util.RemoteStiffener import update_remote_stiffener, get_stiffen_command
from util.ObstacleAvoidance import update_obstacle_avoidance
from util.Constants import LEDColour
from util.EventComms import update_event_comms

# from vision.visual_ref import estimate_ref_pose

attempted_debug_connect = False

skill_instance = None
headskill_instance = None

# Initialise global logger configs
log.init_logger()

def catch_all(tick):
    def catcher(blackboard):
        try:
            return tick(blackboard)
        except Exception:
            log.error("Behaviour exception:", say=False, exc_info=True)
            log.sound("deep_bip.wav")
            # TODO: Add a say for where the error occurred
            led_override.override_eye_segment(
                led_override.RIGHT_EYE, led_override.allSegments, LEDColour.purple)
            led_override.override_eye_segment(
                led_override.LEFT_EYE, led_override.allSegments, LEDColour.purple)
            raise

    return catcher


def skill_instance_factory(
        blackboard, skill, behaviour_packages):
    # Load the module and the class we're going to use.
    found_skill = False
    SkillClass = None
    behaviour_root = '/home/nao/data/behaviours'
    for package in behaviour_packages:
        look_in = join(behaviour_root, '/'.join(package))
        seen_modules = [name for _, name, _ in iter_modules([look_in])]
        if skill not in seen_modules:
            log.info(f"{skill}.py was not in {look_in}, skipping import attempt.")
            continue
        skill_path = '%s.%s' % ('.'.join(package), skill)
        try:
            skill_module = import_module(skill_path)
            # Access the class so we can do some reflection.
            SkillClass = getattr(skill_module, skill)
            found_skill = True
            log.info(f"Successfully imported {skill} from {skill_path}")
            break
        except ImportError as e:
            log.error(f"{package} {e}")
            log.error(format_exc())

    if not found_skill:
        raise ImportError('Skill: %s not found in behaviour packages %s' %
                          (skill, behaviour_packages))

    new_world = World(blackboard)  # It's a whole new world.
    _skill_instance = SkillClass(world=new_world)

    return _skill_instance


@catch_all
def tick(blackboard):
    """
    This is the main entry point from C++ into our Python behaviours and back.

    More specifically it is the bridge from which C++ calls Python inside
    the redbackbots executable, and receives a BehaviourRequest back.

    Currently called in `robot/perception/behaviour/python/PythonSkill.cpp`,
    the `PythonSkill::execute()` C++ function, and explicitly the line
    `behaviour_tick(bb)`.

    :param blackboard: The redbackbots Blackboard, a bunch of things
        stored in global memory.
    :return: A `robot.BehaviourRequest()` instance, defined in C++ inside
        `robot/types/BehaviourRequest.hpp`.
    """
    # connect to debugger if needed
    global attempted_debug_connect
    if not attempted_debug_connect and blackboard.config['debug.connect_pydevd']:
        try:
            import pydevd_pycharm

            # this file is written to by nao_sync
            with open('/tmp/last_sync_ip') as f:
                remote_ip = f.readline().strip().replace('\n', '')
            pydevd_pycharm.settrace(remote_ip, port=7070, stdoutToServer=True, stderrToServer=True)
        # print exception if debugger is not running
        except:
            print("Could not attach debugger. Is it running on IP " + remote_ip + "?")
    attempted_debug_connect = True


    # Update all blackboard dependent helper modules.
    update_event_comms(blackboard)
    update_timer(blackboard)
    update_global(blackboard)
    update_team_status(blackboard)
    update_field_geometry(blackboard)
    update_sonar(blackboard)
    update_network_ear(blackboard)
    update_remote_stiffener(blackboard)
    update_game_status(blackboard)
    update_obstacle_avoidance(blackboard)

    #estimate_ref_pose(blackboard)

    led_override.reset_led_override()

    global skill_instance
    if not skill_instance:
        behaviour_packages = [
            ['body', 'roles'],
            ['body', 'skills'],
            ['body', 'test']]
        skill_name = blackboard.behaviour.skill
        skill_instance = skill_instance_factory(
            blackboard, skill_name, behaviour_packages)

    global headskill_instance
    if not headskill_instance:
        head_packages = [
            ['head']
        ]
        headskill_name = blackboard.behaviour.headskill
        headskill_instance = skill_instance_factory(
          blackboard, headskill_name, head_packages)

    # On every tick of the perception thread, we update the blackboard,
    # tick the skill, and then return the resulting behaviour request.
    skill_instance.world.update(blackboard)
    skill_instance.world.b_request = BehaviourRequest()
    skill_instance.world.b_request.actions = All()
    skill_instance.tick()
    request = skill_instance.world.b_request

    headskill_instance.world.update(blackboard)
    headskill_instance.world.b_request = request
    headskill_instance.tick()
    request = headskill_instance.world.b_request


    # So we know which ball is being used by the robot. Team == green, Robots == Red
    if believe_more_in_team_ball():
        led_override.override_eye_segment(led_override.LEFT_EYE, led_override.TEAM_BALL_SEGMENTS, LEDColour.green)
    else:
        led_override.override_eye_segment(led_override.LEFT_EYE, led_override.TEAM_BALL_SEGMENTS, LEDColour.dim_white)

    # LED colouring for ball detection (moved to lower half)
    if len(blackboard.vision.balls) <= 0:
        led_override.override_eye_segment(led_override.LEFT_EYE, led_override.BALL_SEEN_SEGMENTS, LEDColour.dim_white)
        # request.actions.leds.leftEye = LEDSegments.off
    else:
        led_override.override_eye_segment(led_override.LEFT_EYE, led_override.BALL_SEEN_SEGMENTS, LEDColour.red)
        # request.actions.leds.leftEye = LEDSegments.ballSeen    
    
    # Use foot LEDs to indicate if we have done enough passes for us to kick a goal
    led_override.override(led_override.RIGHT_FOOT, LEDColour.green if robot_can_score() else LEDColour.blue)

    # Foot LED indicating if a whistle has been heard
    if whistle_detected():
        led_override.override(led_override.RIGHT_FOOT, LEDColour.red)
        led_override.override(led_override.LEFT_FOOT, LEDColour.red)

    # Override leds as requested by skills
    led_override.override_request(request)

    # Get right ear colour depending on network activity
    request.actions.leds.rightEar = get_ear_colour()

    # Remotely stiffen
    request.actions.stiffen = get_stiffen_command()

    request.actions.ballAge = ball_lost_time()

    # Obtain behaviour hierarchies of current skill
    request.behaviourDebugInfo.bodyBehaviourHierarchy = \
        skill_instance.world.behaviour_hierarchy
    skill_instance.world.behaviour_hierarchy = ""
    request.behaviourDebugInfo.headBehaviourHierarchy = \
        headskill_instance.world.behaviour_hierarchy
    headskill_instance.world.behaviour_hierarchy = ""

    return request
