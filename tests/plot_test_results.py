#! /usr/bin/env python
import os
import sys

if __name__ == '__main__':
    os.chdir(os.path.dirname(sys.argv[0]))
    execfile('testQPOases_VelocityAllocation.py')
