import os
from util.actioncommand import standStraight
import util.actioncommand as ac
from datetime import datetime
from BehaviourTask import BehaviourTask
from robot import Sensors
from audio import whistle_controller
# from vision.visual_ref import estimate_ref_pose
# from audio import pose_audio
NAO_WHISTLE_LOCATION = os.path.join(os.environ['HOME'],
                                    'whistle/heard_whistles')
WHISTLE_FILE_FORMAT = 'whistle_%Y_%m_%d_%H%M%S.wav'
def whistle_heard(num_seconds):
    """
    :return: True if a whistle file was created in the last num_seconds.
    """
    now = datetime.now()
    # Ensure folder exists
    if not os.path.exists(NAO_WHISTLE_LOCATION):
        os.makedirs(NAO_WHISTLE_LOCATION)
    file_names = sorted(os.listdir(NAO_WHISTLE_LOCATION))
    try:
        deltas = [
            now - datetime.strptime(file_name, WHISTLE_FILE_FORMAT)
            for file_name in file_names
        ]
        # Note: Check both abs() and non-abs() so we ignore future whistles
        return any(
            abs(delta.total_seconds()) < num_seconds and
            delta.total_seconds() < num_seconds
            for delta in deltas
        )
    except:
        return []
class VisionReferee(BehaviourTask):
    global rhead
    global mhead
    global fhead
    # def _initialise_sub_tasks(self):
    #     whistle_controller.kill_all_python_processes()
    #     whistle_controller.start_listening_for_whistles()
    #     self.heard_whistle = False
    def _reset(self):
        print('Method Reset')
        whistle_controller.kill_all_python_processes()
        whistle_controller.start_listening_for_whistles()
        self.heard_whistle = False
        self.previous_pose = 'stand'
        self.tick_counter = 0
    def show_pose(self, pose_name):
        if pose_name == "goal-blue":
            self.world.b_request.actions.body = ac.goalblue()
        elif pose_name == "goal-red":
            self.world.b_request.actions.body = ac.goalred()
        elif pose_name == "goal-kick-blue":
            self.world.b_request.actions.body = ac.goalkickblue()
        elif pose_name == "goal-kick-red":
            self.world.b_request.actions.body = ac.goalkickred()
        elif pose_name == "corner-kick-blue":
            self.world.b_request.actions.body = ac.cornerkickblue()
        elif pose_name == "corner-kick-red":
            self.world.b_request.actions.body = ac.cornerkickred()
        elif pose_name == "kick-in-blue":
            self.world.b_request.actions.body = ac.kickinblue()
        elif pose_name == "kick-in-red":
            self.world.b_request.actions.body = ac.kickinred()
        elif pose_name == "pushing-freekick-blue":
            self.world.b_request.actions.body = ac.pushingfreeblue()
        elif pose_name == "pushing-freekick-red":
            self.world.b_request.actions.body = ac.pushingfreered()
        elif pose_name == "full-time":
            self.world.b_request.actions.body = ac.fulltime()
        elif pose_name == "stand":
            self.world.b_request.actions.body = ac.standStraight()
    def _tick(self):
        
        if not self.heard_whistle:
            if whistle_heard(1):
                print("I heard the whistle!!")
                self.tick_counter = 0
                self.heard_whistle = True
                self.pose_dict = {
                    "full-time": 0,
                    "goal-kick-blue": 0,
                    "goal-blue": 0,
                    "goal-kick-red": 0,
                    "goal-red": 0,
                    "corner-kick-blue": 0,
                    "corner-kick-red": 0,
                    "kick-in-blue": 0,
                    "kick-in-red": 0,
                    "pushing-freekick-blue": 0,
                    "pushing-freekick-red": 0
                }
                self.num_poses_seen = 0
                self.start_time = datetime.now()
        if self.heard_whistle:
            print("WHISTLES HEARD")
            detection_delay = (datetime.now() - self.start_time).total_seconds()
            print("detection_delay", detection_delay)
            
            if detection_delay > 1.0:
                rhead = 0
                mhead = 0
                fhead = 0
                bboard = self.world.blackboard
                sensorValues = bboard.motion.sensors.sensors
                fhead = sensorValues[Sensors.Head_Touch_Front]
                mhead = sensorValues[Sensors.Head_Touch_Middle]
                rhead = sensorValues[Sensors.Head_Touch_Rear]
                if(fhead == 1.0 or mhead == 1.0 or rhead == 1.0):
                    print('Touched')
                    self.heard_whistle = False
                    self.show_pose('stand')
                # else:
                    # self.pose = estimate_ref_pose(bboard)
                    # if self.pose is not None and self.pose != '':
                    #     self.pose = self.pose.replace(' ', '-')
                    #     self.pose_dict[self.pose] += 1
                    #     self.num_poses_seen = sum(self.pose_dict.values())
                    # print('pose dict', self.pose_dict)
                    # print("num_poses_seen", self.num_poses_seen)
                    # if self.num_poses_seen >= 3 or detection_delay > 5.0:
                    #     # if there is a pose detected, show pose and set it as previous pose
                    #     self.pose = max(self.pose_dict, key=self.pose_dict.get)
                    #     print('Selected pose from dict', self.pose)
                    #     # if self.pose is not None and self.pose != '':
                    #     # self.pose = self.pose.replace(' ', '-')
                    #     print("Pose in Tick: ", self.pose)
                    #     self.show_pose(self.pose)
                        # pose_audio.play_sound(self.pose)
                        ## END OF INITIAL CODE
                        # self.previous_pose = self.pose
                    #     if(fhead == 1.0 or mhead == 1.0 or rhead == 1.0 or self.headtouch_activated == True):
                    #         self.headtouch_activated = True
                    #         print('Touched')
                    #         self.heard_whistle = False
                    #         self.show_pose('stand')
                    #     else:
                    #         print('Not Touched!!')
                    # else:
                    #     self.pose = self.previous_pose
                    #     if(fhead == 1.0 or mhead == 1.0 or rhead == 1.0 or self.headtouch_activated== True):
                    #         self.headtouch_activated = True
                    #         print('Touched')
                    #         self.heard_whistle = False
                    #         self.show_pose('stand')
                    #     else:
                    #         self.show_pose(self.pose)
                    #         if self.pose != 'stand':
                    #             pose_audio.play_sound(self.pose)
                    #         print('Not Touched!!')
        else:
            # print('Waiting for Whistle!!')
            self.show_pose('stand')
        # self.tick_counter += 1
        # if self.tick_counter > 5:
        #     self.headtouch_activated = False
