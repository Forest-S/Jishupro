# -*-coding:utf-8 -*-

import math
import serial
import time
from ctypes import *
import sys
import numpy
import cv2
import threading

#constant variable
#arm length
l1 = 85.01
l2 = 116.15
#portnum of arduino
ard_port = 8
ser_ard = False
ser_ard = serial.Serial(ard_port-1, 9600, timeout=1)
use_cam = True
servo = False

def demo():
    while True:
        go_linear(80,170,500,theta=0)
        go_linear(80,40,200,theta=0)
        go_linear(180,40,200,theta=0)
        go_linear(80,40,200,theta=0)
def remove_karashis():
    while True:
        time.sleep(2)
        remove_karashi()
    

def main():
    init()
    #wait
    raw_input()
    
    open_futa()
    tigiru()
    remove_shoyus()
    add_shoyu()
    stir()
def init():
    print "initializing..."
    ser_ard.write("init,100.")
    go_linear(70,180,200,theta = 0, timeout = 10)
    servo.SetAngle(4,0)
    servo.SetAngle(5,-math.pi/2)
    valve(0)
    servo.SetSpeed(3,0)
    return
def open_futa():
    go_linear(75,60,600,theta = -0.2)
    go_linear(75,30,200,theta = -0.2)
    valve(1)
    go_linear(75,0,200,theta = -0.2)
    go_linear(75,60,50,theta = -0.2, timeout = 1.5)
    centpos = numpy.array([140,-15])
    draw_circle(centpos, 60, 100,start = 3.2, end = 0 , step = -0.2)
    return
def tigiru():
    go_linear(195,-30,100,wait = False)
    ser_ard.write("mot,-255.")
    time.sleep(0.5)
    ser_ard.write("init,100.")
    go_linear(200,0,200,timeout = 1)
    go_linear(180,60,200,timeout = 1)
    go_linear(180,60,200,theta = 0,timeout = 1)
    go_linear(200,20,300,theta = 0, timeout = 1)
    time.sleep(0.5)
    valve(0)
    time.sleep(1.5)
def remove_shoyus():
    #remove_shoyu
    #go_linear(85,70,200,theta = 0)
    #go_linear(85,10,200,theta = 0)
    #valve(1)
    #go_linear(85,-20,50,theta = 0)
    #go_linear(85,20,100,theta = 0)
    #go_linear(100,120,300,theta = 0)
    #go_linear(190,60,200,theta = 0)
    #valve(0)
    remove_shoyu()
    #remove_karashi
    remove_karashi()
    
    #remove_sheet
    go_linear(80,60,150,theta = -0.2)
    go_linear(80,0,100,theta = -0.2)
    valve(1)
    go_linear(80,-30,50,theta = -0.2,timeout = 1)
    go_linear(70,60,50,theta = -0.2)
    ser_ard.write("mot,255.")
    for i in range(4):
        go_linear(80,180,600,theta = 0,error = 50)
        go_linear(80,80,600,theta = 0,error = 50)
    ser_ard.write("init,100.")
    go_linear(80,180,200,theta=0)

def remove_shoyu():
    x_min = 320-30
    x_max = 320+30
    y_disp_max = 400
    y_disp_min = 150
    arm_max_y = 105
    arm_min_y = 60
    mot_duty = 80
    while True:
        karashi_x, karashi_y = k_detect.reload_window_shoyu()
        print karashi_x,karashi_y
        if ((karashi_x > x_min) and (karashi_x < x_max)):
            ser_ard.write("mot,0.")
            cv2.waitKey(100)
            for i in range(10):
                #repeating to get latest camera data
                karashi_x, karashi_y = k_detect.reload_window_shoyu()
            print karashi_x,karashi_y
            if((karashi_x > x_min) and (karashi_x < x_max)):
                break
            else:
                mot_duty = -mot_duty
        else :
            ser_ard.write("mot,"+str(mot_duty)+".")
    karashi_y_arm = arm_min_y + ((1.0*y_disp_max - karashi_y) / (y_disp_max - y_disp_min)) * (arm_max_y - arm_min_y)
    if(karashi_y_arm > arm_max_y): karashi_y_arm = arm_max_y
    if(karashi_y_arm < arm_min_y): karashi_y_arm = arm_min_y
    print karashi_y_arm
    go_linear(karashi_y_arm,60,300,theta = -0.2)
    go_linear(karashi_y_arm,10,200,theta = -0.2)
    valve(1)
    go_linear(karashi_y_arm,-30,50,theta = -0.2,timeout=1)
    go_linear(karashi_y_arm,40,50,theta = 0)
    go_linear(80,120,300,theta = 0)
    go_linear(190,60,200,theta = 0)
    valve(0)
    time.sleep(0.5)
    
def remove_karashi():
    x_min = 320-30
    x_max = 320+30
    y_disp_max = 400
    y_disp_min = 150
    arm_max_y = 105
    arm_min_y = 60
    mot_duty = 80
    while True:
        karashi_x, karashi_y = k_detect.reload_window()
        print karashi_x,karashi_y
        if ((karashi_x > x_min) and (karashi_x < x_max)):
            ser_ard.write("mot,0.")
            cv2.waitKey(100)
            for i in range(10):
                #repeating to get latest camera data
                karashi_x, karashi_y = k_detect.reload_window()
            print karashi_x,karashi_y
            if((karashi_x > x_min) and (karashi_x < x_max)):
                break
            else:
                mot_duty = -mot_duty
        else :
            ser_ard.write("mot,"+str(mot_duty)+".")
    karashi_y_arm = arm_min_y + ((1.0*y_disp_max - karashi_y) / (y_disp_max - y_disp_min)) * (arm_max_y - arm_min_y)
    if(karashi_y_arm > arm_max_y): karashi_y_arm = arm_max_y
    if(karashi_y_arm < arm_min_y): karashi_y_arm = arm_min_y
    print karashi_y_arm
    go_linear(karashi_y_arm,60,300,theta = -0.2)
    go_linear(karashi_y_arm,10,200,theta = -0.2)
    valve(1)
    go_linear(karashi_y_arm,-30,50,theta = -0.2,timeout=1)
    go_linear(karashi_y_arm,40,50,theta = 0)
    go_linear(80,120,300,theta = 0)
    go_linear(190,60,200,theta = 0)
    valve(0)
    time.sleep(0.5)
def add_shoyu():
    servo.SetSpeed(4,1)
    servo.SetSpeed(5,5)
    servo.SetAngle(4,1.7)
    while True:
        now_angle = servo.GetAngle(4)
        servo.SetSpeed(4,1+2.5*math.sin(math.pi*(1.7-now_angle)/1.7))
        if(1.7 - now_angle < 0.05 ) : break
    servo.SetAngle(5,1)
    time.sleep(0.6)
    servo.SetAngle(5,-math.pi/2)
    time.sleep(0.5)
    servo.SetSpeed(4,5)
    servo.SetAngle(4,0)
    return
def stir():
    go_linear(80,180,50,theta = -1.7)
    valve(0)
    go_linear(80,135,50,theta = -1.7)
    ser_ard.write("mot,100.")
    servo.SetSpeed(3,1)
    for i in range(10):
        go_linear(80,135,50,theta = -1.75)
        time.sleep(1)
        go_linear(80,135,50,theta = -1.55)
        time.sleep(1)
    servo.SetSpeed(3,0)
    ser_ard.write("init,120.")
    go_linear(70,185,50,theta = -1.7)
def valve(value):
    if(value == 0):
        print "valve close"
        return servo.SetAngle(6,-1)
    else :
        print "valve open"
        return servo.SetAngle(6,1)
def draw_circle(centpos, r,speed, start = 0, end = 3.14, step = -0.1):
    now_theta = start
    while True:
        dest_pos = centpos + r*numpy.array([math.cos(now_theta), math.sin(now_theta)])
        print dest_pos
        go_linear(dest_pos[0],dest_pos[1],speed,theta = -now_theta+math.pi,wait = False)
        now_theta = now_theta + step
        if now_theta < end :
            break
    return
    
def go_linear(xd,yd,speed, theta = "None", wait = True, error = 10, timeout = 5 ):
    print "go_linear" , xd,yd
#calculate Jacobian
    dest = numpy.array([xd,yd])
    dest_theta = ik_pos(dest)
    now_theta = numpy.array([servo.GetAngle(1),servo.GetAngle(2)])
    now_pos = fk_pos(now_theta)
    theta1 = now_theta[0]
    theta2 = now_theta[1]
    v_dest = speed * (dest - now_pos) / numpy.linalg.norm(dest-now_pos)
    J = [[l1*math.cos(theta1)+l2*math.cos(theta1-theta2), -l2 * math.cos(theta1 - theta2)] ,
         [-l1*math.sin(theta1)-l2*math.sin(theta1-theta2),l2*math.sin(theta1-theta2)]]
    omega = numpy.linalg.solve(J,v_dest)
    servo.SetSpeed(1,omega[0])
    servo.SetSpeed(2,omega[1])
    servo.SetAngle(1,dest_theta[0])
    servo.SetAngle(2,dest_theta[1])
    start_time = time.time()
    while True:
        now_theta = numpy.array([servo.GetAngle(1),servo.GetAngle(2)])
        now_pos = fk_pos(now_theta)
        v_dest = speed * (dest - now_pos)/numpy.linalg.norm(dest-now_pos)
        theta1 = now_theta[0]
        theta2 = now_theta[1]
        J = [[l1*math.cos(theta1)+l2*math.cos(theta1-theta2), -l2 * math.cos(theta1 - theta2)] ,
             [-l1*math.sin(theta1)-l2*math.sin(theta1-theta2),l2*math.sin(theta1-theta2)]]
        omega = numpy.linalg.solve(J,v_dest)
        if(max(omega[0],omega[1]) > 6.5):
            if(max(omega[0],omega[1] == omega[0])):
                omega[0] = 6.5
                omega[1] = 6.5*omega[1]/omega[0]
            else:
                omega[0] = 6.5*omega[0]/omega[1]
                omega[1] = 6.5
        if abs(omega[0]) < 0.1:
            omega[0] = 0.1
        if abs(omega[1]) < 0.1:
            omega[1] = 0.1
        servo.SetSpeed(1,omega[0])
        servo.SetSpeed(2,omega[1])
        if(theta is not  "None"):
            servo.SetAngle(3,theta - theta1 + theta2)
        if (wait == False) : break
        if((numpy.linalg.norm(now_pos - dest) < error)) :
            if(theta is not "None"):
                   servo.SetAngle(3,theta - theta1 + theta2)
            break
        if(time.time() - start_time > timeout) : break
    return

def ik_pos(dest):
    xd = dest[0]
    yd = dest[1]
    if((xd**2+yd**2 > (l1+l2)**2) or ((xd**2 + yd**2) < abs(l1-l2)**2)):
        print "ik cannot be solved"
        return False
    d1 = (xd**2+yd**2+l1**2-l2**2)/(2*l1)
    d2 = (xd**2+yd**2-l1**2+l2**2)/(2*l2)
    theta1_1 = math.atan2(yd,xd) + math.atan2(math.sqrt(xd**2+yd**2-d1**2),d1)
    theta1_2 = math.atan2(yd,xd) - math.atan2(math.sqrt(xd**2+yd**2-d1**2),d1)
    theta2_1 = -math.atan2(math.sqrt(xd**2+yd**2-d1**2),d1) - math.atan2(math.sqrt(xd**2+yd**2-d2**2),d2)
    theta2_2 = +math.atan2(math.sqrt(xd**2+yd**2-d1**2),d1) + math.atan2(math.sqrt(xd**2+yd**2-d2**2),d2)
    if(math.sin(theta1_1) > math.sin(theta1_2)):
        theta1 = theta1_1
        theta2 = theta2_1
    else:
        theta1 = theta1_2
        theta2 = theta2_2
    theta1 = math.pi/2 - theta1
    if(theta1 > math.pi):
        theta1 = 2*math.pi-theta1
    return numpy.array([theta1,theta2])

def fk_pos(theta):
    theta1 = theta[0]
    theta2 = theta[1]
    xd = l1*math.sin(theta1) + l2*math.sin(theta1-theta2)
    yd = l1*math.cos(theta1) + l2*math.cos(theta1-theta2)
    return numpy.array([xd,yd])
    
def radtobit(theta):
    return int(theta*1024/(300*math.pi/180)+512)
def bittorad(theta):
    return (theta-512)*300*math.pi/(1024*180)

class cam():
    def __init__(self) :
        self.cap = cv2.VideoCapture(0)
        if (self.cap.isOpened() == True):
            print "camera is successfully opened"
        else :
            print "camera initializing is failed"
        self.stop_flag = False
        cv2.namedWindow('image_karashi')
        cv2.namedWindow('image')
        cv2.createTrackbar('Hmin','image_karashi',20,360,self.changeparam)
        cv2.createTrackbar('Hmax','image_karashi',30,360,self.changeparam)
        cv2.createTrackbar('Smin','image_karashi',27,255,self.changeparam)
        cv2.createTrackbar('Smax','image_karashi',255,255,self.changeparam)
        cv2.createTrackbar('Vmin','image_karashi',96,255,self.changeparam)
        cv2.createTrackbar('Vmax','image_karashi',255,255,self.changeparam)
        self.Hmax = cv2.getTrackbarPos('Hmax','image_karashi')
        self.Hmin = cv2.getTrackbarPos('Hmin','image_karashi')
        self.Smax = cv2.getTrackbarPos('Smax','image_karashi')
        self.Smin = cv2.getTrackbarPos('Smin','image_karashi')
        self.Vmax = cv2.getTrackbarPos('Vmax','image_karashi')
        self.Vmin = cv2.getTrackbarPos('Vmin','image_karashi')
    def __del__(self):
        self.cap.release()
        self.stop_flag = True
        print "finish"
    def changeparam(self,x):
        self.Hmax = cv2.getTrackbarPos('Hmax','image_karashi')
        self.Hmin = cv2.getTrackbarPos('Hmin','image_karashi')
        self.Smax = cv2.getTrackbarPos('Smax','image_karashi')
        self.Smin = cv2.getTrackbarPos('Smin','image_karashi')
        self.Vmax = cv2.getTrackbarPos('Vmax','image_karashi')
        self.Vmin = cv2.getTrackbarPos('Vmin','image_karashi')
    def get_karashi_pos(self):
        cv2.waitKey(10)
        ret, frame = self.cap.read()
        if (frame == None):
            self.cap.release()
            self.__init__()
            ret, frame = self.cap.read()
        frame_extract = self.extract_color(frame,self.Hmin,self.Hmax,self.Smin,self.Smax,self.Vmin,self.Vmax)
        #print self.Hmin,self.Hmax
        kernel = numpy.ones((5,5),numpy.uint8)
        frame_extract = cv2.erode(frame_extract,kernel,iterations = 1)
        frame_extract = cv2.dilate(frame_extract,kernel,iterations = 1)
        moments = cv2.moments(frame_extract)
        x_c = 0
        y_c = 0
        if (moments["m00"] != 0):
            x_c = int(moments["m10"]/moments["m00"])
            y_c = int(moments["m01"]/moments["m00"])
        cv2.circle(frame_extract, (x_c,y_c), 3,(0,0,0))
        cv2.circle(frame, (x_c,y_c), 20,(0,0,255))
        return  x_c,y_c,frame,frame_extract
    def get_shoyu_pos(self):
        cv2.waitKey(10)
        ret, frame = self.cap.read()
        if (frame == None):
            self.cap.release()
            self.__init__()
            ret, frame = self.cap.read()
        frame_extract = self.extract_color(frame,75,183,0,255,0,136)
        #print self.Hmin,self.Hmax
        kernel = numpy.ones((5,5),numpy.uint8)
        frame_extract = cv2.erode(frame_extract,kernel,iterations = 1)
        frame_extract = cv2.dilate(frame_extract,kernel,iterations = 1)
        moments = cv2.moments(frame_extract)
        x_c = 0
        y_c = 0
        if (moments["m00"] != 0):
            x_c = int(moments["m10"]/moments["m00"])
            y_c = int(moments["m01"]/moments["m00"])
        cv2.circle(frame_extract, (x_c,y_c), 3,(0,0,0))
        cv2.circle(frame, (x_c,y_c), 20,(0,0,255))
        return  x_c,y_c,frame,frame_extract
    def extract_color(self, src, Hmin,Hmax,Smin,Smax,Vmin,Vmax ):
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        lower= numpy.array([Hmin, Smin,Vmin])
        upper= numpy.array([Hmax,Smax,Vmax])
        img_mask = cv2.inRange(hsv,lower,upper)
        return img_mask
    def reload_window(self):
        tmp = self.get_karashi_pos()
        frame = tmp[2]
        frame_extract = tmp[3]
        cv2.imshow('image', frame)
        cv2.imshow('image_karashi',frame_extract)
        cv2.waitKey(10)
        return tmp[0],tmp[1]
    def reload_window_shoyu(self):
        tmp = self.get_shoyu_pos()
        frame = tmp[2]
        frame_extract = tmp[3]
        cv2.imshow('image', frame)
        cv2.imshow('image_karashi',frame_extract)
        cv2.waitKey(10)
        return tmp[0],tmp[1]
if use_cam == True:
    k_detect = cam()

def config():
    while True:
        print k_detect.reload_window()
        if(cv2.waitKey(10) == 27):break
        
#Servo Control Class
class Servo:
    #dynamixel library settings
    dxlib2 = windll.LoadLibrary("dxlib_x32.dll")
    dxopen = dxlib2.DX_OpenPort
    dxclose = dxlib2.DX_ClosePort
    dxping = dxlib2.DX_Ping
    dxwriteword = dxlib2.DX_WriteWordData
    dxreadword = dxlib2.DX_ReadWordData
    dxwritebyte = dxlib2.DX_WriteByteData

    dxopen.argtypes = [c_char_p, c_long]
    dxopen.restype = c_void_p
    dxclose.argtypes = [c_void_p]
    dxping.argtypes = [c_void_p, c_ubyte, c_int, c_void_p]
    dxwriteword.argtypes = [c_void_p, c_ubyte, c_ushort, c_ushort, c_void_p]
    dxreadword.argtypes  = [c_void_p, c_ubyte, c_ushort, c_void_p, c_void_p]
    dxwritebyte.argtypes = [c_void_p, c_ubyte, c_ushort, c_void_p, c_void_p]
    TORQUE_ENABLE_ADDR = 24
    GOAL_POS_ADDR = 30
    SPEED_ADDR = 32
    TORQUE_LIMIT_ADDR = 34
    PRESENT_POS_ADDR = 36
    OK = 1
    NG = 0
    terr = c_ushort()
    dat  = c_ushort()
    dxport = "\\.\COM9"
    def __init__(self):
        self.Open()
    def Open(self):
        self.devid = self.dxopen(self.dxport, 1000000)
        if (self.devid == None) :
            print "dxopen failed"
        else :
            print "dxopen succeeded"
        return self.devid
    def GetAngle(self, ID):
        self.dxreadword(self.devid, c_ubyte(ID), self.PRESENT_POS_ADDR, byref(self.dat), byref(self.terr))
        return bittorad(self.dat.value)
    def SetAngle(self, ID, Angle) :
        if(ID == 3):
            Angle = Angle + 1
        angle_bit = radtobit(Angle)
        return self.dxwriteword(self.devid, c_ubyte(ID), self.GOAL_POS_ADDR, c_ushort(angle_bit), byref(self.terr))
    def SetSpeed(self, ID, speed) :
        speed_bit = int(math.ceil(abs(speed*60/(0.111*2*math.pi))))
        return self.dxwriteword(self.devid, c_ubyte(ID), self.SPEED_ADDR, c_ushort(speed_bit), byref(self.terr))

servo = Servo()
