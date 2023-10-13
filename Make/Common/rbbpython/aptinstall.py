#! /usr/bin/env python3

'''
Use python bindings around apt-install to configure software
'''

import apt
import sys

import rbbpython.echo as echo
import rbbpython.shellEscape as shell

cache = apt.cache.Cache()
#cache.update()
cache.open()

'''
Check apt cache for package installation status
'''
def aptcheck(pkg_name):

    pkg = cache[pkg_name]
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

