#! /usr/bin/env python3

'''
Quick wrappers around loading of RBB configuration including
- Environment settings
- INI Config files
'''

import configparser
import json
import os

from rbbpython.echo import (
    print_error,
    print_warning
)

'''
Static names/variables
'''
cppExecutable="redbackbots"
initdWirelessFile = "rbbwireless"

'''
Location of configuration files for RBB
'''
def configDirectory():
    return rbbCheckoutDirectory() + "/Config"

'''
Location of build folder for given release
'''
def getBuildDirectory(platform, release):
    return str('{0}/Build/{1}/CMake/{2}'.format(rbbCheckoutDirectory(), platform, release))

'''
Location C++ executable file for given release
'''
def getCppExecutablePath(release):
    return str('{0}/Build/Linux/Nao/{1}/redbackbots'.format(rbbCheckoutDirectory(), release))

'''
Retrieve Environment parameter from executing shell
'''
def getEnvParameter(param, check=False):
    value = ""
    if param in os.environ:
        value = os.environ[param]
    else :
        # If checking mode, return the empty string, otherwise error.
        if check:
            print_warning("Cannot load env param (" + param + ")")
        else :
            print_error("Cannot load env param (" + param + 
                        "). RedBackBots Bash Config probably not loaded correctly."+
                        " Source <checkout_dir>/.bashrc")
            exit()
    return value

'''
Get the keys from the config block excluding the default block
'''
def getKeys(config):
    keys = [key for key in config.keys() if key != configparser.DEFAULTSECT]
    return keys

'''
On robot image root directory
'''
def imageRootDirectory():
    return "/home/nao"

'''
On robot image data sub-directory
'''
def imageDataDirectory():
    return imageRootDirectory() + "/data"

'''
On robot image sub-directory for robot-specific config files (such as vision/kinematics)
'''
def imageRobotConfigsDirectory():
    print_error("rbbpython.config.imageRobotConfigsDirectory() not updated for new build")
    return imageRootDirectory() + "/configs"

'''
Load file by name from root of config directory
'''
def loadConfig(file):
    fullPath = configDirectory() + "/" + file
    if not os.path.exists(fullPath):
        print_error("Cannot locate config file: " + fullPath)
        return None
    
    config = loadConfigFile(fullPath)
    return config

'''
Load file by name from root of config directory
'''
def loadRawConfig(file):
    fullPath = configDirectory() + "/" + file
    if not os.path.exists(fullPath):
        print_error("Cannot locate config file: " + fullPath)
        return None
    
    config = loadRawConfigFile(fullPath)
    return config

'''
Load a JSON formatted file by name from root of config directory
'''
def loadConfigJSON(file):
    fullPath = configDirectory() + "/" + file
    if not os.path.exists(fullPath):
        print_error("Cannot locate config file: " + fullPath)
        return None
    
    config = loadConfigJSONFile(fullPath)
    return config


'''
Load file by full path
'''
def loadConfigFile(file):
    config = configparser.ConfigParser()
    config.read(file)
    return config

'''
Load raw file by full path
'''
def loadRawConfigFile(file):
    config = configparser.ConfigParser(strict=False)
    config.optionxform=str
    config.read(file)
    return config

'''
Load a JSON formatting file by full path
'''
def loadConfigJSONFile(file):
    fh = open(file)
    config = json.load(fh)
    fh.close()
    return config

'''
Location of sub-directory for ML model files to copy/sync with the robot image
'''
def mlModelsDirectory():
    print_error("rbbpython.config.mlModelsDirectory() not updated for new build")
    return rbbCheckoutDirectory() + "/ml-models"

'''
Location of robot sync image
'''
def naoImageDirectory():
    return rbbCheckoutDirectory() + "/image"

'''
Location of robot sync image (nao user home folder)
'''
def naoImageHomeDirectory():
    return naoImageDirectory() + imageRootDirectory()

'''
Output a JSON formatted file by name from root of config directory
'''
def outputConfigJSON(file, config):
    fullPath = configDirectory() + "/" + file
    outputConfigJSONFile(fullPath, config)


'''
Output a JSON formatting file by full path
'''
def outputConfigJSONFile(file, config):
    with open(file, 'w') as outfile:
        json.dump(config, outfile, indent=4)

'''
Location of root directory for config files for all robots
'''
def robotConfigRootDirectory():
    return configDirectory() + "/Robots"

'''
Location of sub-directory for config files for given robot
'''
def robotConfigDirectory(robot):
    return robotConfigRootDirectory() + "/" + robot

'''
Location of sub-directory for network specific config files for given robot
'''
def robotNetworkConfigDirectory(robot):
    print_error("rbbpython.config.robotNetworkConfigDirectory() not updated for new build")
    return robotConfigDirectory(robot) + "/network"

'''
Location of sub-directory for source files to copy/sync with the robot image
'''
def robotSourceDirectory():
    print_error("rbbpython.config.robotSourceDirectory() not updated for new build")
    return rbbCheckoutDirectory() + "/robot-src"

'''
Root directory of RBB codebase
'''
def rbbCheckoutDirectory():
    return getEnvParameter('REDBACKBOTS_CHECKOUT_DIR')

'''
Location of sub-directory for config templates
'''
def templateDirectory():
    return configDirectory() + "/Templates"


def writeConfig(file, section, new_values):
    # Load the existing config file
    config = loadRawConfig(file)
    fullPath = configDirectory() + "/" + file

    # Check if the file is valid
    if config is None:
        print_error("Cannot write to an invalid config file.")
        return

    # Check if the section exists in the config
    if section not in config:
        print_error(f"Section '{section}' not found in the config file.")
        return

    # Update the existing values with the new values
    for key, value in new_values.items():
        config[section][key] = str(value)

    # Save the updated config file
    with open(fullPath, 'w') as config_file:
        config_file.write('\n')
        config.write(config_file, space_around_delimiters=False)
