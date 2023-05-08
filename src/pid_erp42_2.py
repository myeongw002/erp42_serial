#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64 #setpoint
from erp42_serial.msg import ESerial
from erp42_serial.msg import setState


rospy.init_node('pid_erp42')


msg2 = Float64() #setpoint
msg3 = ESerial()


set_velocity = 0 #m/s
set_degree = 0   #degree
set_brake = 0
set_gear = 0

def set_state_callback(msg): #pid_input
    global set_velocity
    global set_degree
    global set_brake
    global set_gear
    set_velocity = msg.set_velocity
    set_degree = msg.set_degree
    set_brake = msg.set_brake
    set_gear = msg.set_gear
    msg2.data = set_velocity #input pid setpoint (velocity)
    ################################
    #degree pid code expected later#
    #msg2.data = set_degree        #
    ################################
    print(set_velocity)


pid_speed = 0    #m/s
def control_effort_callback(msg): #pid_output
    global pid_speed
    pid_speed = msg.data

def serial_msg(speed=0,brake=0,steer=0,gear=1):

    if gear == 1:
        msg3.speed = 0
        

def velocity_controller():
    if set_gear == 0 and set_brake <= 0: #forward
        msg3.gear = 0
        if pid_speed >= 0 :
            msg3.speed = int(pid_speed)
            msg3.brake = 0
        else :
            msg3.speed = 0
            msg3.brake = int(pid_speed)*(-1)
    elif set_gear == 2 and set_brake <= 0: #backward
        msg3.gear = 2
        if pid_speed <= 0 :
            msg3.speed = int(pid_speed)*(-1)
            msg3.brake = 0
        else :
            msg3.speed = 0
            msg3.brake = int(pid_speed)
    elif set_gear == 1 :
        msg3.speed = 0
        msg3.gear = 1



def degree_controller():
    if set_degree <= 28 and set_degree >= -28 :
        msg3.steer = int(set_degree*71)
    elif set_degree > 28 :
        msg3.steer = 2000
    elif set_degree < -28 :
        msg3.steer = -2000


def brake_controller():
    if set_brake >= 1 :
        msg3.brake = int(set_brake)
    elif set_brake <= 0 :
        msg3.brake = 0


def serial_msg():
    brake_controller()
    velocity_controller()
    degree_controller()


sub1 = rospy.Subscriber('set_state', setState, set_state_callback)
sub2 = rospy.Subscriber('control_effort', Float64, control_effort_callback)
pub1 = rospy.Publisher('setpoint', Float64, queue_size=1)
pub2 = rospy.Publisher('erp42_serial', ESerial, queue_size=100)


#rospy.spin()
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    print("gear : %d / speed : %d / steer : %d / brake : %d" %(msg3.gear, msg3.speed, msg3.steer, msg3.brake))
    print("set_velocity : %f / pid_speed : %f " %(set_velocity, pid_speed))
    pub1.publish(msg2)
    pub2.publish(msg3)
    serial_msg()
    rate.sleep()
    
    
