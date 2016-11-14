import cv2
from Simulator_main import Differential_wheel
from random import randint
import os
import timeit
from copy import deepcopy

'''
    Initializing the Differential_wheel class
'''


var = Differential_wheel(960,1280, 100, 900, 0.0, 13.0, 0.45)      # (No. of rows, No. of cols, initial x, initial y, orientation(radians), Distance bw wheel, time period)

var.addNoise(1,1)                                           # Add noise to the two wheels

start = timeit.default_timer()

while True:
    try:
        print "Pos.", var.pos,
        v1 = randint(-20,20)                                  # Giving random velocity to left wheel bw. 0..50
        v2 = randint(-20,20)                                  # Giving random velocity to right wheel bw. 0..50

        v1 = 28.1818
        v2 = 28.1662

        print "Vel.", v1,v2
        

        
        value = var.request_kinectics(v1,v2)
        
        if cv2.waitKey(30) == 27:                           # Press escape to break the loop
            break
    except:
        break
    
stop = timeit.default_timer()
print "Time :: " , stop - start

cv2.waitKey(0)
var.endProg()                                               # Clean up the Program
