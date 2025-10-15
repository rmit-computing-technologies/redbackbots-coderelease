#! /usr/bin/env python3

'''
Custom colour-based reporting to terminal
'''

import sys
import os
# import art

BANNER_SPACING = 5
class unix_colours:
    RED     = "\033[31;1m"
    GREEN   = "\033[32;1m"
    ORANGE  = "\033[33;1m" 
    BLUE    = "\033[1;34m"
    MAGENTA = "\033[1;35m"
    CYAN    = "\033[1;36m"
    RESET   = "\033[3;0m"

def print_error(msg, robot=None):
    sys.stdout.write(unix_colours.RED)
    if robot:
        print(f"\t{'['+robot+']':<12} [!]    {msg}")
    else:
        print(" [!]   ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_status(msg, robot=None):
    sys.stdout.write(unix_colours.CYAN)
    if robot:
        print(f"\t{'['+robot+']':<12} [+]    {msg}")
    else:
        print(" [+]   ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_warning(msg, robot=None):
    sys.stdout.write(unix_colours.ORANGE)
    if robot: 
        print(f"\t{'['+robot+']':<12} [-]    {msg}")
    else:    
        print(" [-]   ", msg)
    sys.stdout.write(unix_colours.RESET)

def print_progress(msg, progress, robot=None):
    if robot:
        print(f"\t{'['+robot+']':<12} {'['+str(progress)+'%]':<6} {msg}")
    else:    
        print(f" {'['+str(progress)+'%]':<6} {msg}")

def print_subitem(msg, robot=None):
    if robot:
        print(f"\t{'['+robot+']':<12}  -     {msg}")
    else:    
        print("  -    ", msg)

def print_success(msg, robot=None):
    sys.stdout.write(unix_colours.GREEN)
    if robot:
        print(f"\t{'['+robot+']':<12} [*]    {msg}")
    else:    
        print(" [*]   ", msg)
    sys.stdout.write(unix_colours.RESET)


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

def print_banner(message, colour=unix_colours.RESET, spacing=BANNER_SPACING):
    sys.stdout.write(colour)
    # art.tprint(message)
    frame = "#"*(len(message))
    inside_spacing = " "*len(message)
    print()
    print(f"\t{'#'*spacing}{frame}{'#'*spacing}")
    for _ in range((spacing - 2)//2):
        print(f"\t{'#':<{spacing}}{inside_spacing}{'#':>{spacing}}")
    print(f"\t{'#':<{spacing}}{message}{'#':>{spacing}}")
    for _ in range((spacing - 2)//2):
        print(f"\t{'#':<{spacing}}{inside_spacing}{'#':>{spacing}}")
    print(f"\t{'#'*spacing}{frame}{'#'*spacing}")
    print()
    sys.stdout.write(unix_colours.RESET)


if __name__ == "__main__":
    print_error("This is an error")
    print_progress("This is progress", 1)
    print_progress("This is progress", 17)
    print_progress("This is progress", 100)
    print_status("This is a status")
    print_subitem("This is a subitem")
    print_warning("This is a warning")
    print_success("This is a success")

    print_error("This is an error", "jupiter")
    print_progress("This is progress", 1, "jarlaxle")
    print_progress("This is progress", 12, "jarlaxle")
    print_progress("This is progress", 100, "jarlaxle")
    print_status("This is a status", "joy")
    print_subitem("This is a subitem", "journey")
    print_warning("This is a warning", "Joule")
    print_success("This is a success", "Joule")

    print_banner("This is a banner!")
    print_banner("This is a big banner!", colour=unix_colours.CYAN, spacing=10)

    sys.stdout.write(unix_colours.MAGENTA)
    print("This is magenta")
