#! /usr/bin/env python3

'''
Use python bindings around apt-install to configure software
'''

import apt
import sys
import os

import rbbpython.echo as echo
import rbbpython.shellEscape as shell

cache = apt.cache.Cache()
#cache.update()
cache.open()

'''
Check apt cache for package installation status
'''
def aptcheck(pkg_name):
    try:
        pkg = cache[pkg_name]
        installed = pkg.is_installed
        return installed
    except KeyError:
        echo.print_warning(f"The cache has no package named {pkg_name}, attempting to update cache")
        new_cache = apt.cache.Cache()
        new_cache.open()
        pkg = new_cache[pkg_name]
        installed = pkg.is_installed
        return installed

'''
Install using Python-apt package. This requires the script to be run as "sudo" or with a user account with sudo privillages. Without, this will fail.
'''
def aptinstall(pkg_name):
    pkg = cache[pkg_name]
    installed = False
    if pkg.is_installed:
        echo.print_subitem("{pkg_name} already installed".format(pkg_name=pkg_name))
        installed = True
    else:
        pkg.mark_install()

        try:
            cache.commit()
            installed = True
        except Exception as arg:
            echo.print_error("Installation failed [{err}]".format(err=str(arg)))
    
    return installed

'''
Install using commandline apt with sudo permissions
'''
def aptinstallcmdline(pkgs):
    sep = ' '
    fullList = sep.join(pkgs)
    # noninteractive prevents apt from asking for user input
    command = "sudo DEBIAN_FRONTEND=noninteractive apt install -y " + fullList
    shell.exec(command, hideOutput=False)

'''
Check and add additional apt repositories
'''
def addAptRepository(repo):
    additional_sources_path = "/etc/apt/sources.list.d/"

    # Check in the main sources.list file
    sources_path = "/etc/apt/sources.list"
    with open(sources_path, "r") as sources_file:
        sources = sources_file.read()
        if repo in sources:
            echo.print_status(f"{repo} apt repository already configured")
            return True

    # Check in the additional sources.list.d/ directory
    additional_sources_files = os.listdir(additional_sources_path)
    for file_name in additional_sources_files:
        if repo in file_name:
            echo.print_status(f"{repo} apt repository already configured")
            return True

    echo.print_status(f"Adding {repo} to apt repositories")
    shell.exec(f"sudo add-apt-repository ppa:{repo}/ppa -y", hideOutput=False)
    shell.exec(f"sudo apt update -y", hideOutput=False)