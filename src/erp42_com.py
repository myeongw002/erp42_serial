#!/usr/bin/env python3
# -*- coding: utf-8 -*-한글 주석 선언
'''
ROS KINETIC, python2.7, Ubuntu 16.04

ERP42 UPPER to PCU 프로토콜

Specification : RS232
DATA Ordering :  Big Endian (UPPER to PCU)
Cycle Time : 20 msec
Baud:115200, parity : None, Stop : 1
언맨드 제어기 (이하 PCU)
USER PC or 제어기 (이하 UPPER)

Packet : 14Bytes
byte name : [S, T, X, AorM, E-STOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

STX -> Start bytes// [0x53, 0x54, 0x58]
AorM -> Auto or Manual// manual mode : 0x00 , auto mode : 0x01
ESTOP -> Emergency STOP// E-STOP Off : 0x00, E-STOP On : 0x01
GEAR -> F,N,B gear// forward drive : 0x00, neutral : 0x01, backward drive : 0x02
SPEED -> actual speed (KPH) * 10// 0 ~ 200
STEER -> actual steering dgree (dgree) * 71, 오차율 : 4%, negative is left steer// Right : 0 ~ 2000, Left : -2000 ~ 0 
BRAKE -> 1 : no braking, 150 : full braking// 1 ~ 150
ALIVE -> increasing each one step//1,2,3,4,5......253,254,255,1,2,3.....
ETX -> end bytes//[0x0D, 0x0A]

Packet Default
byte name : [[0x53, 0x54, 0x58], 0x01, 0x00, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, 0, (0x0D, 0x0A)]
'''
import rospy
from erp42_serial.msg import ESerial
from erp42_serial.msg import Evelocity
from std_msgs.msg import Float64
import os
import struct
import time
import serial

print('serial begin')

# Set a PORT Number & baud rate
PORT = '/dev/ttyUSB0'
BaudRate = 115200
ERPS = serial.Serial(PORT,BaudRate)


class erp42Serial :
    STX = (0x53, 0x54, 0x58)
    AorM = 0x01  #defolat 0x01 A
    ESTOP = 0x00 #defloat 0x00 off
    GEAR = 0  #동적 파라미터
    SPEED = 0 #동적 파라미터
    STEER = 0 #동적 파라미터
    BRAKE = 0 #동적 파라미터
    ALIVE = 0
    ETX = (0x0D, 0x0A)
    
    '''
    def __init__(self, G, SP, ST, B):
        self.GEAR = G  #동적 파라미터
        self.SPEED = SP #동적 파라미터
        self.STEER = ST #동적 파라미터
        self.BRAKE = B #동적 파라미터
    '''
    def setParams(self, G=0, SP=0, ST=0, B=0): #__init__과 비슷하지만, class를 초기화 할필요없다.
        self.GEAR = G  #동적 파라미터 forward drive : 0x00, neutral : 0x01, backward drive : 0x02
        self.SPEED = SP #동적 파라미터 actual speed (KPH) * 10// 0 ~ 200
        self.STEER = ST #동적 파라미터  Right : 0 ~ 2000, Left : -2000 ~ 0 
        self.BRAKE = B #동적 파라미터 1 : no braking, 150 : full braking// 1 ~ 150

    def Alive(self): 
        if self.ALIVE <= 254 :
            self.ALIVE += 1
        else :
            self.ALIVE = 0
        
        
    def Struct(self):
        s = struct.pack('!BBBBBBHhBBBB',self.STX[0], self.STX[1], self.STX[2], self.AorM, self.ESTOP, self.GEAR, self.SPEED, self.STEER, self.BRAKE, self.ALIVE, self.ETX[0], self.ETX[1])
        tu = struct.unpack('!BBBBBBBBBBBBBB', s)
        return tu

    
##-------------------------------------------------##

              #GEAR, SPEED, STEER, BRAKE
Sclass = erp42Serial()

'''
g = input("기어 전진 = 0, 중립 = 1, 후진 = 2 : ")
spd = input("속도 0 ~ 200 : ")
ster = input("스티어링 -2000 ~ 2000 : ")
brk = input("브레이크 0 ~ 150 : ")
'''
g = 0
spd = 0
ster = 0
brk = 1
##spd = input("속도 0 ~ 200 : ")
##ster = input("스티어링 -2000 ~ 2000 : ")

def serialwrite( g, spd, ster, brk):
    Sclass.Alive()
    Sclass.setParams(g, spd, ster, brk)
    a = Sclass.Struct()
    #print(a)
    ERPS.write(a)

LISEN = []
List = []
turn = 0
def serialread():
    global LISEN
    global List
    global turn
    if ERPS.readable():
        LINE = ERPS.read()
        L = struct.unpack('!B', LINE)
        a = L[0]
        if a == 0x53 and turn == 0:
            turn = 1
        elif a == 0x54 and turn == 1 :
            turn = 2
        elif a == 0x58 and turn == 2 :
            turn = 3
        elif turn >= 3 and turn <= 15:
            turn += 1
            LISEN.append(a)
        elif turn > 15 :
            List = LISEN
            turn = 0
            LISEN = []
            #print(List)
    return List


#encorder list num : 8 9 10 11 
time_turn = 0
time0 = 0
L1 = 0
L2 = 0
msg2 = Evelocity()
msg3 = Float64() #for pid control input


def encoder_speed(L):
    global time_turn
    global time0
    global L1
    global L2
    D = 0.01665 # mitter  [1+ = 16.65mm]
    time_set = 0.1
    if len(L) == 13 :
        #print(L[8],L[9],L[10],L[11])
        L = (L[8]+L[9]*256+L[10]*256**2+L[11]*256**3)*D
        if time_turn == 0:
            time0 = time.time()
            time_turn = 1
            L1 = L
        elif time_turn == 1 and time.time() - time0 >= time_set:
            L2 = L - L1
            V = L2/time_set #m/s
            msg2.velocity = V; msg3.data = V
            pub1.publish(msg2); pub2.publish(msg3)
            print(V)
            time_turn = 0




rospy.init_node('erp42_com')

'''
msg = ESerial()
msg.gear = 1
msg.speed = 0
msg.steer = 0
msg.brake = 0
msg.shutdown = 0
'''
g = 1
spd = 0
ster = 0
brk = 0
#serialwrite(msg.gear, msg.speed, msg.steer, msg.brake)
#serialwrite(g, spd, ster, brk)
def callback(msg):
    global g
    global spd
    global ster
    global brk
    g = msg.gear
    spd = msg.speed
    ster = msg.steer
    brk = msg.brake


sub = rospy.Subscriber('erp42_serial', ESerial, callback)
pub1 = rospy.Publisher('erp42_velocity', Evelocity, queue_size=100) #erp42_velocity
pub2 = rospy.Publisher('state', Float64, queue_size=100) #erp42_velocity
rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    serialwrite(g, spd, ster, brk)
    L = serialread()
    encoder_speed(L)
    rate.sleep()
    
    
    
    
    #print("gear : %d / speed : %d / steer : %d / brake : %d / velocity : %f" %(g, spd, ster, brk, msg2.velocity))
    
    
    
    
    
    

