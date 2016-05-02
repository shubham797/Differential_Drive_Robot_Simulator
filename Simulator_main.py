import sys
import numpy as np
import math
import time
import cv2
from random import randint


class Differential_wheel:
    global omega
    global theta
    global left
    global right
    global cen
    global v1
    global v2
    global R
    global t
    global image
        
    def endProg(self):              # for testing
        cv2.destroyAllWindows()
        sys.exit(0)
        
    def create_matrix(self,x,y=0):
        if y != 0:
            array = np.zeros((x,y),dtype=np.float)
        else:
            array = np.zeros((x),dtype=np.float)
        return array

    def mult_matrix(self,x,y):
        x = np.matrix(x)
        y = np.matrix(y)
        z = x*y
        return z

    def add_matrix(self,x,y):
        x = np.matrix(x)
        y = np.matrix(y)
        z = x+y
        return z

    def mod_omega(self,vel1,vel2):
        w = vel2 - vel1
        w = w / Differential_wheel.R
        return w

    def get_r(self,vel1,vel2):
        rad = Differential_wheel.R*0.5*(vel1+vel2)
        if vel1 == vel2:
            vel2 = 0.00001+vel1
        rad = rad / (vel2-vel1)
        return rad

    def ICC(self,rx,ry,vel1,vel2,theta):
        rx = rx - self.get_r(vel1,vel2)*(math.sin(theta))
        ry = ry + self.get_r(vel1,vel2)*(math.cos(theta))
        return rx,ry

    def mod_left(self,matrix,omega):
        t = Differential_wheel.t
        matrix[0][0] = math.cos(omega*t)
        matrix[0][1] = -1 * math.sin(omega*t)
        matrix[1][0] = math.sin(omega*t)
        matrix[1][1] = math.cos(omega*t)
        matrix[2][2] = 1.0
        return matrix

    def mod_cen(self,center,position,ic,theta):
        center[0][0] = position[0]-ic[0]
        center[1][0] = position[1]-ic[1]
        center[2][0] = theta
        return center

    def mod_right(self,right,ic,omega):
        t = Differential_wheel.t
        right[0][0] = ic[0]
        right[1][0] = ic[1]
        right[2][0] = omega*t
        return right

    def rad_deg(self,rad):
        return ((rad/math.pi)*180)%360

    def plot_line(self,img,pos,mat):
        image = img
        cv2.line(image,(int(pos[0]),Differential_wheel.sx-int(pos[1])),(int(mat[0]),Differential_wheel.sx-int(mat[1])),(255,255,255),2)
        cv2.imshow("image",image)
        cv2.waitKey(20)
        return image

    def plot_direction(self,img,pos,degree):
        image = img
        cv2.circle(image,(int(pos[0]),Differential_wheel.sx-int(pos[1])),8,(0,255,0),1)
        xpos = int(int(pos[0]) + 8*math.cos(degree))
        ypos = int(Differential_wheel.sx - int(pos[1]) - 8*math.sin(degree))
        cv2.circle(image,(xpos,ypos),2,(0,0,255),-1)    
        cv2.imshow("image",image)
        cv2.waitKey(20)
        return image

    def set_param(self,x,y,theta,radi,ti):
        global pos
        Differential_wheel.pos[0] = x
        Differential_wheel.pos[1] = y
        Differential_wheel.v1 = 0.0
        Differential_wheel.v2 = 0.0
        Differential_wheel.R = radi
        Differential_wheel.t = ti
        Differential_wheel.theta = theta

    def saveImage(self,var):
        var += "Image.jpg"
        cv2.imwrite(var,Differential_wheel.image)
        print "Saved image : ",var

    def addNoise(self,l_nois,r_nois):
        Differential_wheel.noise = True
        Differential_wheel.ln = int(l_nois)
        Differential_wheel.rn = int(r_nois)

    def request_kinectics(self,vl,vr):
        
        while True:
            
            time.sleep(Differential_wheel.t)

            if Differential_wheel.noise == True:
                v2 = vr + (Differential_wheel.rn - randint(0,Differential_wheel.rn))
                v1 = vl + (Differential_wheel.ln - randint(0,Differential_wheel.ln))
            else:
                v2 = vr
                v1 = vl
            #print "v1_v2 : ",v1,v2

            if v1 == v2:
                Differential_wheel.mat[0] = Differential_wheel.pos[0] + v1*math.cos(Differential_wheel.theta)*Differential_wheel.t
                Differential_wheel.mat[1] = Differential_wheel.pos[1] + v1*math.sin(Differential_wheel.theta)*Differential_wheel.t

                if Differential_wheel.show == True:
                    Differential_wheel.image = self.plot_line(Differential_wheel.image,Differential_wheel.pos,Differential_wheel.mat)
                    self.plot_direction(Differential_wheel.image,Differential_wheel.mat,Differential_wheel.theta)
                
                Differential_wheel.pos[0] = Differential_wheel.mat[0]
                Differential_wheel.pos[1] = Differential_wheel.mat[1]
                print Differential_wheel.pos,self.rad_deg(Differential_wheel.theta)
                    
            elif v1 == -1*v2:
                Differential_wheel.theta = Differential_wheel.theta + (2*v2*Differential_wheel.t)/Differential_wheel.R
                print Differential_wheel.pos,self.rad_deg(Differential_wheel.theta)

            else:
                Differential_wheel.v1 = v1
                Differential_wheel.v2 = v2
                    
                Differential_wheel.omega = self.mod_omega(v1,v2)
                Differential_wheel.icc[0],Differential_wheel.icc[1] = self.ICC(Differential_wheel.pos[0],Differential_wheel.pos[1],Differential_wheel.v1,Differential_wheel.v2,Differential_wheel.theta)

                
                Differential_wheel.left = self.mod_left(Differential_wheel.left,Differential_wheel.omega)
                Differential_wheel.cen = self.mod_cen(Differential_wheel.cen,Differential_wheel.pos,Differential_wheel.icc,Differential_wheel.theta)
                Differential_wheel.right = self.mod_right(Differential_wheel.right,Differential_wheel.icc,Differential_wheel.omega)

                Differential_wheel.mat = self.add_matrix(self.mult_matrix(Differential_wheel.left,Differential_wheel.cen),Differential_wheel.right)

                Differential_wheel.theta = Differential_wheel.mat[2]
                
                if Differential_wheel.show == True:
                    Differential_wheel.image = self.plot_line(Differential_wheel.image,Differential_wheel.pos,Differential_wheel.mat)
                    self.plot_direction(Differential_wheel.image,Differential_wheel.mat,Differential_wheel.theta)
                
                Differential_wheel.pos[0] = Differential_wheel.mat[0]
                Differential_wheel.pos[1] = Differential_wheel.mat[1]

                print Differential_wheel.pos,self.rad_deg(Differential_wheel.theta)
            
            
            break
        return Differential_wheel.pos[0],Differential_wheel.pos[1],Differential_wheel.theta
    
    def __del__(self):
        class_name = self.__class__.__name__
        self.endProg()
        print class_name, "instance destroyed"
    
    def __init__(self,wx,wy,x,y,theta,radi,ti):
        
        Differential_wheel.show = True
        Differential_wheel.noise = False
        Differential_wheel.pos = self.create_matrix(2)
        Differential_wheel.mat = self.create_matrix(2)
        Differential_wheel.icc = self.create_matrix(2)

        Differential_wheel.sx = wx
        Differential_wheel.sy = wy
        Differential_wheel.left = self.create_matrix(3,3)
        Differential_wheel.cen = self.create_matrix(3,1)
        Differential_wheel.right = self.create_matrix(3,1)
    
        self.set_param(x,y,theta,radi,ti)
        Differential_wheel.omega = self.mod_omega(0,0)
        Differential_wheel.image = np.zeros((Differential_wheel.sx,Differential_wheel.sy,3),dtype=np.uint8)

        Differential_wheel.icc[0],Differential_wheel.icc[1] = self.ICC(Differential_wheel.pos[0],Differential_wheel.pos[1],Differential_wheel.v1,Differential_wheel.v2,Differential_wheel.theta)        


