#! /usr/bin/env python3

'''
Util methods for capturing or running shell commands
'''

import subprocess

def capture(command):
    data = subprocess.check_output(command, shell=True)
    return data.decode('utf-8').rstrip()

''' 
Execute the command
Return True/False if the command was successful
'''
def exec(command, hideOutput=True):
    stdout = None
    stderr = None
    if hideOutput:
        stdout = subprocess.DEVNULL
        stderr = subprocess.DEVNULL
    returncode = subprocess.call(command, stdout=stdout, stderr=stderr, shell=True)
    return returncode == 0