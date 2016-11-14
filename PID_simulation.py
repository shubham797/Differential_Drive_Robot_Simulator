import cv2
import sys
import numpy as np
import os
from math import atan2, degrees, pi, sqrt
import math
from datetime import datetime
sys.path.append('C:\Users\hp pc\Desktop\Project\Map\Differential_Drive_Robot_Simulator')
from Simulator_main import Differential_wheel
from copy import deepcopy

class RoundBot:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.i = 0
        self.integral = 0.
        self.orientation = 0.0
        self.length = 11.0

    def setParam(self, new_x, new_y, new_orientation):                                      # This orient is the change

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation += float(new_orientation)
        self.orientation %= (2.0 * pi)

    def correction_to_actuator(self,correction,pid_param):

        self.kp = pid_param[0]
        self.kd = pid_param[1]
        self.ki = pid_param[2]
        
        if self.i == 0:
            self.cost_t = correction
            self.differ = 0
            self.i += 1
        else:
            self.differ = correction - self.cost_t
            self.cost_t = correction
        self.integral += correction
        

        #print "Three Params : ",correction,self.differ,self.integral
        new_turn = correction * self.kp + self.differ * self.kd + self.integral * self.ki
        
        print "Correction : ",correction, new_turn

        #new_turn *= 0.56
        
        v_l,v_r, vel_r_l, vel_r_r = self.getVelocity(new_turn)#, fixed_val = velocity)

        if vel_r_l > 99.5:
            vel_r_l = 99.5
        if vel_r_r > 99.5:
            vel_r_r = 99.5
            
        result1 = str(int(vel_r_l + 0.5))  + "_" + str(int(vel_r_r + 0.5)) + "_0"
        result = str(v_l)  + "_" + str(v_r) + "_0"

        send_result1 = "{:20}".format(result1)
        send_result = "{:20}".format(result)
        
        return send_result, send_result1

    def getVelocity(self,omega, fixed_val = 30):                        # Get individual velocity of two wheels  For my robot
        
        #fixed_val = self.vel_2_pwm(fixed_val)                          # create different function for different robot
        #print "Fixed _val :", fixed_val
        if abs(omega) < 0.1:
            vel_l = fixed_val
            vel_r = fixed_val
        else:
            if omega <= 0.785 and omega >= -0.785:
                length = 13.0
                vel_l = fixed_val - 0.5*(omega)*length
                vel_r = fixed_val + 0.5*(omega)*length
            else:
                if omega > 0:
                    omega = 0.785
                else:
                    omega = -0.785
                length = 13.0
                vel_l = fixed_val - 0.5*(omega)*length
                vel_r = fixed_val + 0.5*(omega)*length

        #print "Velocity :: ", vel_l, vel_r
        
        vel_r_l = vel_l
        vel_r_r = vel_r

        #print "Velocity :: ", vel_r_l, vel_r_r
        
        return vel_l, vel_r, vel_r_l, vel_r_r

    '''
        Connection Part
    '''
    def connect_to_server(self,host,port):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host,port))
            return sock
        except:
            print "Unexpected error:", sys.exc_info()[0]
            print "Error in Connecting"
            sys.exit(0)


    def send_data(self,sock,dat):
        try:
            sock.send(dat)
            #print dat
        except :
            print "Error in Sending"

    def close_conn(self,sock):
        sock.close()
    '''
        Connection Part over
    '''

class CallBack:

    def __init__(self,n,src):
        self.img = src
        self.name = n
        self.point = []
        cv2.namedWindow(self.name)

    def draw_circle(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.img = cv2.circle(self.img,(x,y),5,(255,0,0),-1)
            height = self.img.shape[0]
            self.point.append([x,height-y])

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


    
class Controller:
   
    def __init__(self,img,p):
        #cb = CallBack("Choose src and dest",img)
        #p = cb.callback()
        
        self.s = p[0]
        self.d = p[1]
        self.orient = 0.0
        
        self.i = 0

    def setNewPosition(self,newx,newy,orient):
        self.x = newx
        self.y = newy
        self.x_val = self.x
        self.y_val = self.y

        if self.i == 0:
            self.i += 1
            self.prevx = self.x
            self.prevy = self.y
            

        omg,initial = self.findAngle(orient, self.prevx, self.prevy, self.x, self.y)

        self.prevx = self.x
        self.prevy = self.y
        
        return omg,initial

    def findAngle(self,deg,x1,y1,x2,y2):         # getting angles from 0 - 360 using two points
        dx = x2 - x1
        dy = y2 - y1
        rads = atan2(dy,dx)
        rads %= 2*pi
        degs = degrees(rads)
        #print "Previous :: ", deg,degs
        d = (degs - deg)# % 360
        '''
        if d >= 180:
            d = -1 * (d-180)
        elif d <= -180:
            d = -180 - d
        '''
        self.orient = degs
        return d,degs

    def getPathFollowerError(self):
        rx,ry = self.dist_from_sincurve(self.s,self.d,[self.x,self.y])
        omega,_ = self.findAngle(self.orient,self.x,self.y,rx,ry)
        #print "PAth track error : " ,omega
        return math.radians(omega)

    def getCrossTrackError(self):                # Return the cost track error :: data type -- Float
        # every logic to calculate cost track error
        # distance from desired curve
        #cte = self.dist_from_line(self.s,self.d,[self.x,self.y])
        cte = self.dist_from_circle(self.s,self.d,[self.x,self.y])
        #if self.dist_from_target(self.s,self.d,[self.x,self.y]) < 20.0:
        #    cte = -10000
        #if self.x > self.d[0]:
        #    cte = -10000
        cte *= 0.1
        return cte

    def dist_from_target(self,src,dest,position):
        dist = math.sqrt((dest[1]-position[1])**2 + (dest[0]-position[0])**2)
        return dist
    
    def dist_from_line(self,src,dest,position):         # Movement along line
        base_val = dest[0]*src[1] - dest[1]*src[0]
        base_dist = math.sqrt(((dest[0]-src[0])**2) + ((dest[1]-src[1])**2))
        dist = ((dest[1]-src[1])*position[0])-((dest[0]-src[0])*position[1])
        dist = dist + base_val
        value = float(float(dist)/float(base_dist))
        return value

    def dist_from_circle(self,src,dest,position):           # Circle movement
        
        base_val = dest[0]*src[1] - dest[1]*src[0]
        
        cen_x = (src[0]+dest[0])/2.
        cen_y = (src[1]+dest[1])/2.
        #print cen_x,cen_y
        #print position
        radius = math.sqrt(((dest[0]-cen_x)**2) + ((dest[1]-cen_y)**2))
        dist = math.sqrt(((position[0]-cen_x)**2) + ((position[1]-cen_y)**2))
        #dist = ((dest[1]-src[1])*position[0])-((dest[0]-src[0])*position[1])
        value = radius - dist
        print "Value : ",value
        return value

    def dist_from_eclipse(self,src,dest,position):           # eclipse movement
        
        base_val = dest[0]*src[1] - dest[1]*src[0]
        
        cen_x = (src[0]+dest[0])/2.
        cen_y = (src[1]+dest[1])/2.
        print cen_x,cen_y
        print position
        radius = math.sqrt(((dest[0]-cen_x)**2) + ((dest[1]-cen_y)**2))
        dist = math.sqrt(((position[0]-cen_x)**2) + ((position[1]-cen_y)**2))
        #dist = ((dest[1]-src[1])*position[0])-((dest[0]-src[0])*position[1])
        value = radius - dist
        print "Value : ",value
        return value

    def dist_from_sincurve(self,src,dest,position):                 
        slope = (dest[1]-src[1])/(dest[0]-src[0])
        self.x_val = src[0] + 4.0 * self.i
        self.y_val = 500. + 30. * math.sin(self.x_val*0.025)
        #print "Distance from Sin curve : ", self.x_val,self.y_val,position
        #dist = math.sqrt(((position[0]-self.x_val)**2) + ((position[1]-self.y_val)**2))
        #if position[1] > self.y_val :
        #    dist *= -1
        self.i += 1
        #print "Distance from Sin curve : ", dist
        return self.x_val, self.y_val

    def dist_from_square(self,src,dest,position):
        pass

    def dist_from_sincurve_local(self,src,dest,position):       # Sine and cosine curve movement 
        slope = (dest[1]-src[1])/(dest[0]-src[0])
        #self.x_val = src[0] + 3.0 * self.i
        self.y_val = src[1] + 30. * math.sin(self.x_val*0.0095)
        #print "Distance from Sin curve : ", self.x_val,self.y_val,position
        dist = math.sqrt(((position[0]-self.x_val)**2) + ((position[1]-self.y_val)**2))
        if position[1] > self.y_val :
            dist *= -1

        '''    
        if self.i == 1:
            self.i += 1
            self.tra_prevx = self.x
            self.tra_prevy = self.y
            

        omg,initial = self.findAngle(orient, self.prevx, self.prevy, self.x, self.y)

        self.prevx = self.x
        self.prevy = self.y
        '''
        self.i += 1
        
        #print "Angle of bot : ",degrees(atan2(math.cos(self.x_val*0.025),1))
        #print "Distance from Sin curve : ", dist
        return dist

    def dist_from_trajectory_points(self,src,dest,position):
        if self.i == 0:
            path = np.load("Points.npy")
            path = deepcopy(path.astype(float))
            self.i += 1
        
       
       
def plotPointOnImage(img,vec,size,color):     # print set of points on image
    font = cv2.FONT_HERSHEY_SIMPLEX
    max_height = img.shape[0]
    for x in range(0, size):
        cx = int(vec[x][0])
        cy = int(max_height-vec[x][1])
        cv2.circle(img,(cx,cy), 2, color, -1)
        #cv2.putText(img,str(cx) + ' ' + str(cy),(cx-20,cy-20),font,0.3,(0,0,255),1,cv2.LINE_AA)
    return img

def create_graph():
    font = cv2.FONT_HERSHEY_SIMPLEX
    graph = np.zeros((600,800,3),dtype=np.uint8)
    graph = np.invert(graph)
    cv2.line(graph,(0,300),(800,300),(0,0,0),2)
    cv2.line(graph,(100,0),(100,600),(0,0,0),2)
    cv2.putText(graph,"Time ",(400,350),font,1.,(100,100,100),2,cv2.LINE_AA)
    return graph
        
def simulate():
    global pos_x, pos_y
    global image
    global x, P, pid_param
    global orient

    points = []

    bot = RoundBot()

    intial_error = 23
    pid_param = [1.000111,2.108,0.000001]     # P,D,I param
    
    var = Differential_wheel(960,1280, 94, 427, 0, 13.0, 0.22)      # (No. of rows, No. of cols, initial x, initial y, orientation(radians), Distance bw wheel, time period)
    var.addNoise(0,0)
    
    img_src = var.image

    # for the Line
    #control = Controller(img_src,[[90,450],[1000,450]])                                                   # CHG
    #cv2.line(img_src,(control.s[0],960-control.s[1]),(control.d[0],960-control.d[1]),(255,0,0),2)

    # for the circle
    control = Controller(img_src,[[90,450],[600,450]])                                                 # CHG
    center_circle = (345,450)
    center_radius = 510/2
    cv2.circle(img_src,(center_circle[0],960-center_circle[1]), center_radius, (255,0,0), 2)


    cv2.circle(img_src,(control.s[0],960-control.s[1] + intial_error), 2, (255,255,255), -1)
    cv2.imshow("Window",img_src)
    cv2.waitKey(0)
    cv2.destroyWindow("Window")
    
    i = 0
    orient = 0

    #Graph
    graph = create_graph()
    it = 0
    
    while True:
        try:
            pos_x,pos_y = var.pos
            print "Orient : " ,orient

            gol, orient = control.setNewPosition(pos_x,pos_y,orient)
            bot.setParam(pos_x,pos_y,orient)
            feedback = control.getCrossTrackError()                                  # Get the cost track error
            if feedback == -1000:
                print "Reached Goal"
                break

            #print "Orient : ", gol, orient
            data, data1 = bot.correction_to_actuator(feedback,pid_param)
            
            #data, data1 = bot.correction_to_actuator_omega(control.getPathFollowerError())
            
            #bot.send_data(bot_client,data1)
            
            #___ Converting data to vel
            #print "Data : ", data1
            v1,v2,e = data.split('_')
            #___ Conversion finished


            print "Velocity :: ", v1,v2
            
            var.request_kinectics(float(v1),float(v2))
            img_src  = plotPointOnImage(img_src,[[control.x_val,control.y_val]],1,(255,255,255))      # CHG
            graph  = plotPointOnImage(graph,[[100+it,300+10*feedback]],1,(0,0,255))      # CHG
            
            #image2 = cv2.resize(image,(1600,1200))
            #cv2.imshow("Window",img_src)

            it += 1
            points.append([10*feedback])
            cv2.imshow("Error vs time graph",graph)
            ch = cv2.waitKey(30) & 0xFF
            if ch == 27 :
                break
        except:
          print sys.exc_info()
          break


    #img_src = plotPointOnImage(img_src,points,len(points),(255,255,255))
    #cv2.imwrite("SaveImageThis.jpg",ret)
    np.save("savePoints",points)

    
    back = str(datetime.now()).split(' ')[1]
    file_name = "output/" + str(str(datetime.now()).split(' ')[0]+ "_" + back.split('.')[1] + "_image_PID_kalman.jpg")
    print file_name
    print cv2.imwrite(file_name ,img_src)                                                   # CHG
    #np.save(file_name + str("savePoints"),points)

    cv2.waitKey(0) 
    cv2.destroyAllWindows()
    sys.exit(0)
    

simulate()

