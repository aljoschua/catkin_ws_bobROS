#!/usr/bin/python2
# coding=utf-8

import numpy as np
import matplotlib.pyplot as pp

folder = "/home/aljoschua/Dropbox/Robotik/ü10/"
array = np.load(folder + "lane1.npy")


array = array.T[1:].T
x = array.T[0].T
y = array.T[1].T
print x

pp.plot(x, y)
pp.show()