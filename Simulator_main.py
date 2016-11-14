import sys
import numpy as np
import math
import time
import cv2
from random import randint
from copy import deepcopy


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
    global pos
        
    def endProg(self):              # for testing
        cv2.destroyAllWindows()
        #sys.exit(0)
        
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
        w = w / self.R
        return w

    def get_r(self,vel1,vel2):
        rad = self.R*0.5*(vel1+vel2)
        if vel1 == vel2:
            vel2 = 0.00001+vel1
        rad = rad / (vel2-vel1)
        return rad

    def ICC(self,rx,ry,vel1,vel2,theta):
        rx = rx - self.get_r(vel1,vel2)*(math.sin(theta))
        ry = ry + self.get_r(vel1,vel2)*(math.cos(theta))
        return rx,ry

    def mod_left(self,matrix,omega):
        t = self.t
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
        t = self.t
        right[0][0] = ic[0]
        right[1][0] = ic[1]
        right[2][0] = omega*t
        return right

    def rad_deg(self,rad):
        return ((rad/math.pi)*180)%360

    def plot_line(self,img,pos,mat):
        image = img
        cv2.line(image,(int(pos[0]),self.sx-int(pos[1])),(int(mat[0]),self.sx-int(mat[1])),(255,255,255),2)
        cv2.imshow("image",image)
        cv2.waitKey(20)
        return image
    
    def plot_direction(self,img,pos,degree):
        image = img
        cv2.circle(image,(int(pos[0]),self.sx-int(pos[1])),8,(0,255,0),1)
        xpos = int(int(pos[0]) + 8*math.cos(degree))
        ypos = int(self.sx - int(pos[1]) - 8*math.sin(degree))
        cv2.circle(image,(xpos,ypos),2,(0,0,255),-1)    
        cv2.imshow("image",image)
        cv2.waitKey(20)
        return image

    def set_param(self,x,y,theta,radi,ti):
        global pos
        self.pos[0] = x
        self.pos[1] = y
        self.v1 = 0.0
        self.v2 = 0.0
        self.R = radi
        self.t = ti
        self.theta = theta

    def saveImage(self,var):
        var += "Image.jpg"
        print cv2.imwrite(var,self.image)
        print "Saved image : ",var

    def addNoise(self,l_nois,r_nois):
        self.noise = True
        self.ln = int(l_nois)
        self.rn = int(r_nois)
		
    def hideImage(self):
        self.show = False

    def draw_circle(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.temp_image = cv2.circle(self.temp_image,(x,y),1,(255,0,0),-1)
            height = self.temp_image.shape[0]
            self.points_obstacles.append([x,y])
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_move_x = x
            self.mouse_move_y = y

    def callback(self,number = 2):
        cv2.setMouseCallback(self.name,self.draw_circle)
        while True:
            cv2.imshow(self.name, self.img)
            if len(self.point) >= number:
                break
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cv2.destroyWindow(self.name)
        return self.point

    def addObstacle(self):
        self.temp_image = deepcopy(self.image)
        new_obstacle = True
        self.points_obstacles = []
        

        cv2.namedWindow("Add Obstacles")
        cv2.setMouseCallback("Add Obstacles",self.draw_circle)
        while True:
            cv2.imshow("Add Obstacles", self.temp_image)
            
            if new_obstacle:
                self.points_obstacles = []
                new_obstacle = False
            
            plot_pts = np.array(self.points_obstacles, np.int32)
            cv2.polylines(self.temp_image,[plot_pts.reshape((-1,1,2))],True,(255,0,0),2)

            if len(self.points_obstacles) > 1:
                cv2.fillPoly(self.temp_image,[plot_pts.reshape((-1,1,2))],(255,0,0))
                
            ch = cv2.waitKey(30)
            if ch == ord('n') or ch == ord('N'):
                self.image = deepcopy(self.temp_image)
                new_obstacle = True

            if ch == ord('r') or ch == ord('R'):
                self.temp_image = deepcopy(self.image)
                new_obstacle = True
                
            if ch & 0xFF == 27:
                break
        cv2.destroyWindow("Add Obstacles")
        cv2.imshow("Final Map", self.image)
        cv2.waitKey(0)
        print "Done"
    	
    def request_kinectics(self,vl,vr):
        
        while True:
            
            time.sleep(0)#self.t)

            if self.noise == True:
                v2 = vr + (self.rn - randint(0,self.rn))
                v1 = vl + (self.ln - randint(0,self.ln))
            else:
                v2 = vr
                v1 = vl
            #print "v1_v2 : ",v1,v2

            if v1 == v2:
                self.mat[0] = self.pos[0] + v1*math.cos(self.theta)*self.t
                self.mat[1] = self.pos[1] + v1*math.sin(self.theta)*self.t

                if self.show == True:
                    self.image = self.plot_line(self.image,self.pos,self.mat)
                    self.plot_direction(self.image,self.mat,self.theta)
                
                self.pos[0] = self.mat[0]
                self.pos[1] = self.mat[1]
                #print self.pos,self.rad_deg(self.theta)
                    
            elif v1 == -1*v2:
                self.theta = self.theta + (2*v2*self.t)/self.R
                #print self.pos,self.rad_deg(self.theta)

            else:
                self.v1 = v1
                self.v2 = v2
                    
                self.omega = self.mod_omega(v1,v2)
                self.icc[0],self.icc[1] = self.ICC(self.pos[0],self.pos[1],self.v1,self.v2,self.theta)

                
                self.left = self.mod_left(self.left,self.omega)
                self.cen = self.mod_cen(self.cen,self.pos,self.icc,self.theta)
                self.right = self.mod_right(self.right,self.icc,self.omega)

                self.mat = self.add_matrix(self.mult_matrix(self.left,self.cen),self.right)

                self.theta = self.mat[2]
                
                if self.show == True:
                    self.image = self.plot_line(self.image,self.pos,self.mat)
                    self.plot_direction(self.image,self.mat,self.theta)
                
                self.pos[0] = self.mat[0]
                self.pos[1] = self.mat[1]

                #print self.pos,self.rad_deg(self.theta)
            
            
            break
        return self.pos[0],self.pos[1],self.theta

    
    def __del__(self):
        class_name = self.__class__.__name__
        self.endProg()
        print class_name, "instance destroyed"
    
    def __init__(self,wx,wy,x,y,theta,radi,ti):
        
        self.show = True
        self.noise = False
        self.pos = self.create_matrix(2)
        self.mat = self.create_matrix(2)
        self.icc = self.create_matrix(2)

        self.sx = wx
        self.sy = wy
        self.left = self.create_matrix(3,3)
        self.cen = self.create_matrix(3,1)
        self.right = self.create_matrix(3,1)
    
        self.set_param(x,y,theta,radi,ti)
        self.omega = self.mod_omega(0,0)
        self.image = np.zeros((self.sx,self.sy,3),dtype=np.uint8)

        self.icc[0],self.icc[1] = self.ICC(self.pos[0],self.pos[1],self.v1,self.v2,self.theta)        


