#! /usr/bin/env python3

'''
Simple grep for string in a given file
'''

import os.path
import re

def grep(file, string):
    found = False
    if os.path.exists(file):
        for line in open(file).readlines():
            if re.search(string,line):
                found = True
    return found