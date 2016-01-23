# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 21:48:07 2016

@author: jordi
"""

import csv
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def printCSV(filename):
    with open(filename, 'rb') as csvfile:
        CSVreader = csv.reader(csvfile, delimiter=';')
        for row in CSVreader:
            print ', '.join(row)
    return
            
def loadCSV(filename):
    reader=csv.reader(open(filename,"rb"),delimiter=';')
    x=list(reader)
    return np.array(x).astype('float')
    
def plotTraj3d(filename):
    data = loadCSV(filename)
    mpl.rcParams['legend.fontsize'] = 10
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(data[:,1], data[:,2], data[:,3], label='position')
    ax.legend()
    for direction in (-1, 1):
        for point in np.diag(direction * 2 * np.array([1,1,1])):
            ax.plot([point[0]], [point[1]], [point[2]], 'w')
    plt.show()
    
def plotTrajSubplots(filename):
    data = loadCSV(filename)
    x = data[:,0]
    # Three subplots sharing both x/y axes
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)
    ax1.plot(x, data[:,1])
    ax1.set_title('Sharing both axes')
    ax2.plot(x, data[:,2])
    ax3.plot(x, data[:,3], color='r')
    # Fine-tune figure; make subplots close to each other and hide x ticks for
    # all but bottom plot.
    f.subplots_adjust(hspace=0)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.show()
    
def plotTrajSingle(filename):
    data = loadCSV(filename)
    x = data[:,0]
    #line, = plt.plot(x, data[:,1], 'b', x, data[:,2], 'r', x, data[:,3], 'g')
    line1, = plt.plot(x, data[:,1], label='X')
    line2, = plt.plot(x, data[:,2], label='Y')
    line3, = plt.plot(x, data[:,3], label='Z')
    
    plt.legend([line1, line2, line3], ['X', 'Y', 'Z'])
    plt.show()
    
def plotYZ(filename):
    data = loadCSV(filename)
    y = data[:,1]
    z = data[:,3]
    plt.plot(y, z)
    
    
    