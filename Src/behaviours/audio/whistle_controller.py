"""
NOTHING SHOULD CALL OR USE THESE FUNCTIONS
CAN BE USED TO TEST WHISTLE DETECTOR
"""


### ---- TO BE DELETED ---- ###

import os


def kill_all_python_processes():
    """
    kill all whistle_detector
    """
    print("Killing all whistle detectors AAA")
    # this readable version does not work on 2.1
    # os.system("pkill --signal 9 --full whistle_detector")
    # this short version does not work on 2.8
    # os.system("pkill -9f whistle_detector")
    # this version works on both 2.1 and 2.8
    os.system("pkill -9 -f whistle_detector")


def start_listening_for_whistles():
    """
    Start new background Python process to listen for whistles
    and save detected whistle files in NAO_WHISTLE_LOCATION
    """
    print("Running whistle controller")
    # os.system('/usr/bin/amixer -qs < /home/nao/data/volumeinfo.dat')
    os.system('python3 $HOME/whistle/whistle_detector.py &')

