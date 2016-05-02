import cv2
from Simulator_main import Differential_wheel
from random import randint


'''
    Initializing the Differential_wheel class
'''


var = Differential_wheel(600,800,50,300, 0.0, 10.0, 0.5)      # (No. of rows, No. of cols, initial x, initial y, orientation(radians), Distance bw wheel, time period)

var.addNoise(1,1)                                           # Add noise to the two wheels

while True:
    try:
        v1 = randint(0,50)                                  # Giving random velocity to left wheel bw. 0..50
        v2 = randint(0,50)                                  # Giving random velocity to right wheel bw. 0..50
        
        #v1 = 50
        #v2 = 50
        #v1 = int(raw_input())
        #v2 = int(raw_input())
        
        print "Requesting_kinetics :",var.request_kinectics(v1,v2)

        if cv2.waitKey(10) == 27:                           # Press escape to break the loop
            break        
    except:
        break

var.saveImage('output/')                                    # Save the image to the location
cv2.waitKey(10)
var.endProg()                                               # Clean up the Program
