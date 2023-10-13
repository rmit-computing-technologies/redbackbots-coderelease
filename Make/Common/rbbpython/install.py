#! /usr/bin/env python3

'''
This modules list the files to install/copy.
Used by both nao_sync and createArchive
'''

CHMOD_GO_NONE = 'go= '

'''
Lists, in order, files to install.
Array order determines the order of copying.

Element descriptions
description: Description of the element
src: File in Git repo to copy relative to RBB_CHECKOUT_DIR
dest: Destination on robot/archive to copy to relative to /home/nao
directory: True if the element to install is a directory of contents
naosync: True if should be synched by nao_sync
archive: True if should be included by createArchive
chmod: chmod rule for nao_sync
'''
installFiles = [
    {
        "description": "Authorized Keys",
        "src": "Install/NaoHome/.ssh/authorized_keys",
        "dest": ".ssh/",
        "directory": False,
        "naosync": "AllOnly",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "Nao Image home folder",
        "src": "Install/NaoHome",
        "dest": "",
        "directory": True,
        "naosync": "Always",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "Robot Configuration Files",
        "src": "Config/Robots",
        "dest": "config/Robots",
        "directory": True,
        "naosync": "AllOnly",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "Robot Sound Files",
        "src": "Config/Sounds",
        "dest": "config/Sounds",
        "directory": True,
        "naosync": "AllOnly",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "Robot source files (behaviours/config/etc.)",
        "src": "Src/behaviours",
        "dest": "data/behaviours",
        "directory": True,
        "naosync": "Always",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "Wireless Profiles",
        "src": "Install/Profiles",
        "dest": "Profiles",
        "directory": True,
        "naosync": "AllOnly",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
    {
        "description": "ML Model Files",
        "src": "Install/MLModels",
        "dest": "data",
        "directory": True,
        "naosync": "AllOnly",
        "archive": True,
        "chmod": CHMOD_GO_NONE
    },
]

'''
Lists all of the services to install (and link)
'''
services = [
    'alsa-kludge',
    'hal',
    'lola',
    'rbb'
]
