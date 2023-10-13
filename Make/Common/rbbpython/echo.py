#! /usr/bin/env python3

'''
Custom colour-based reporting to terminal
'''

import sys

class unix_colours:
    RED    = "\033[31;1m"
    GREEN  = "\033[32;1m"
    ORANGE = "\033[33;1m" 
    BLUE   = "\033[1;34m"
    CYAN   = "\033[1;36m"
    RESET  = "\033[0;0m"

def print_error(msg):
    sys.stdout.write(unix_colours.RED)
    print(" [!] ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_status(msg):
    sys.stdout.write(unix_colours.GREEN)
    print(" [+] ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_warning(msg):
    sys.stdout.write(unix_colours.ORANGE)
    print(" [-] ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_progress(msg, progress):
    print(" [{prog}%] {msg}".format(prog=progress, msg=msg))

def print_subitem(msg):
    print("  -  ", msg)


def query_yes_no(question, default="no"):
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        default = "no"

    if default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "

    sys.stdout.write(unix_colours.BLUE)
    sys.stdout.write(question + prompt)
    sys.stdout.write(unix_colours.RESET)
    choice = input().lower()
    if default is not None and choice == '':
        return valid[default]
    elif choice in valid:
        return valid[choice]

def query_generic(question, options=["yes", "no"], default="no"):
    prompt = question + " [" + \
             ",".join(options) + \
             "]"
    
    sys.stdout.write(unix_colours.BLUE)
    sys.stdout.write(prompt)
    sys.stdout.write(unix_colours.RESET)
    choice = input()
    if default is not None and choice == '':
        choice = default
    elif choice not in options:
        choice = ''
    return choice
    
