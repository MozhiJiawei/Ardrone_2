#!/usr/bin/python
# -*- coding:utf-8 -*-

#Put irobot on (0.6, 1.2) to get (x1, y1)
#Then put it on (1.2, 1.2) to get (x2, y2)
'Run: python Ardrone_Ex.py x1 y1 x2 y2'

import sys

if(len(sys.argv) != 5):
    print('Invalid input number.')
    print('Run: python Ardrone_Ex.py x1(0.6) y1(1.2) x2(1.2) y2(1.2)')
    sys.exit(1)
else:
    a = 1 - 0.6 / (float(sys.argv[3]) - float(sys.argv[1]))
    x0 = (0.6 - (1 - a) * float(sys.argv[1])) / a
    b = 1.2 / float(sys.argv[2])
    print('xr = x - {0}(x - {1})'.format(a, x0))
    print('yr = y * {0}'.format(b))
