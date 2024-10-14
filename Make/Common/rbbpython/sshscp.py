#! /usr/bin/env python3

'''
Wrappers for SSH and SCP usage
- Work with SSH config files
- Run SSH and SCP commands
'''

import os

import rbbpython.config as cfg
import rbbpython.echo as echo
import rbbpython.grep as grep
import rbbpython.shellEscape as shell
try:
    # used by nao_sync to sync ip of machine
    import paramiko
except ImportError:
    # This is ok, because build_setup needs to use this file without paramiko, to install paramiko
    pass

'''
Static names/variables
'''
pubicKeyFile = os.path.expanduser('~/.ssh/id_rsa.pub')
sshDirectory = os.path.expanduser("~/.ssh")
sshConfigDirectory = sshDirectory + "/config"

'''
Long form commands
'''
DELETE_COMMAND = "rm -rf "
RSYNC_COMMAND = "rsync --archive --compress --partial "
RSYNC_CHMOD = " --chmod "
RSYNC_SUDO = " --rsync-path=\"sudo rsync\" "
SUDO = "sudo "

'''
Authorisation key file
'''
def authKeyFile():
    return cfg.rbbCheckoutDirectory() + "/Install/NaoHome/.ssh/authorized_keys"


'''
Check is the users public key in in the authorized key file
'''
def checkAuthKey():
    check_command = 'ssh-keygen -l -f ' + pubicKeyFile
    capture = shell.capture(check_command)
    keyid = capture.split(' ')[2]
    return grep.grep(authKeyFile(), keyid)

'''
Class for managing SSH and SCP commands
'''
class SshScp:
    def __init__(self, debug=False):
        self.debug = debug
        self.hostname = ''
        self.hideOutput = False
        self.use_ip = False
        self.robot_name = ''

    def setHostname(self, robot):
        self.hostname = "nao@" + robot
        self.robot_name = robot
        self.use_ip = False

    # For the purposes of this interface, hostname & ip are interchangeable
    # Thus the hostname parameter is directly set instead
    def setIp(self, ip):
        self.hostname = "nao@" + ip
        self.use_ip = True

    def setHideOutput(self, hide):
        self.hideOutput = hide
    
    def deleteFiles(self, dest, sudo=False):
        fullCommand = DELETE_COMMAND
        prepend = ""
        if sudo:
            prepend += SUDO
        fullCommand = prepend + fullCommand + dest
        echo.print_subitem(fullCommand, self.robot_name)
        
        return self.sshCommand(fullCommand)

    def rsyncFiles(self, src, dest, sudo=False, chmod=None):
        fullCommand = RSYNC_COMMAND
        append = " "
        if sudo:
            append += RSYNC_SUDO
        if chmod is not None:
            append += RSYNC_CHMOD + chmod + " "
        fullCommand = fullCommand + append + src + " " + self.hostname + ":" + dest
        echo.print_subitem(fullCommand, self.robot_name)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success

    def scpFile(self, src, dest):
        fullCommand = "scp " + src + " " + self.hostname + ":" + dest
        echo.print_subitem(fullCommand, self.robot_name)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success

    def sshCommand(self, command):
        fullCommand = "ssh -o ConnectTimeout=2 " + self.hostname + " \"" + command + "\""
        echo.print_subitem(fullCommand, self.robot_name)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success
    
    def sshCommandWithSession(self, command):
        """
        This is useful compared to sshCommand as it adds extra variables to the environment such as $SSH_CLIENT
        """
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.hostname.split('@')[1], username='nao', password='nao')
        ssh.exec_command(command)
        echo.print_subitem(command, self.robot_name)

    def makePath(self, path):
        command = "mkdir -p " + path
        return self.sshCommand(command)