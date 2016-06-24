#!/usr/bin/python

import argparse
import math
import matplotlib.pyplot as plt
import numpy as np
import re

def gen_func( coeffs, min, max, steps ):
    step = (max-min) / steps
    xvals = []
    yvals = []
    func = np.poly1d(coeffs)
    for x in np.arange(min, max+step, step):
        y = func(x)
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals

argparser = argparse.ArgumentParser(description='polyfit.py')
argparser.add_argument('--degree', type=int, help='polynomial degree for fit')
argparser.add_argument('files', metavar='file', type=str, nargs='+', help='calibration input files')

args = argparser.parse_args()

for file in args.files:
    try:
        f = open(file, 'r')
        print 'opened:', file
    except:
        print 'cannot open file for reading:', file
    data = []
    title = ''
    for line in f:
        if re.match('\s*#', line):
            # comment line, first comment text becomes plot title
            if title == '':
                title = line.lstrip(' \t#').rstrip()
                print 'title:', title
        else:
            # support any whitespace or comma delimeter
            tokens = re.split('[,\s]+', line.strip())
            if len(tokens) == 2:
                print tokens
                data.append(tokens)
            elif len(tokens) == 1 and tokens[0] == '':
                # empty lines ok, just skip
                pass
            else:
                print 'Invalid line != 2 tokens:', line
    data_array = np.array(data, dtype=np.float64)
    data_array[:,0] *= 100
    data_array[:,1] *= math.pi / 180.0

    fit, res, _, _, _ = np.polyfit( data_array[:,1], data_array[:,0], args.degree, full=True )

    # magic to convert np array to list and reverse the order for display
    for i, x in enumerate(list(reversed(fit.tolist()))):
        print 'x%d: %.0f' % (i, x)

    xvals, yvals = gen_func(fit, data_array[:,1].min(), data_array[:,1].max(), 100)
    #cal_surf[0].plot(data_array[:,1],data_array[:,0],'r.',xvals,yvals,label='fit')
    plt.plot(data_array[:,1],data_array[:,0],'r.',xvals,yvals,label='fit')
    plt.xlabel('Desired surface angle (rad)')
    plt.ylabel('PWM Command (usec*100)')
    if title:
        plt.title(title)
    else:
        plt.title('Surface Calibration')
    plt.show()
