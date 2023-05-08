#!/usr/bin/env python3

#sensor_msgs/Joy

import rospy
from sensor_msgs.msg import Joy
from erp42_serial.msg import ESerial


'''

msg.axes[1] : accel, brack
msg.axes[3] : steer
msg.buttons[4] : foward
msg.buttons[5] : backward


g = input("기어 전진 = 0, 중립 = 1, 후진 = 2 : ")
spd = input("속도 0 ~ 200 : ")
ster = input("스티어링 -2000 ~ 2000 : ")
brk = input("브레이크 0 ~ 150 : ")

'''

rospy.init_node('joy_direct_erp42')

msg2 = ESerial()

def callback(msg):
    msg2 = ESerial()
    if msg.buttons[4] == 1 and msg.buttons[5] == 0: #safe butten
        if msg.axes[1] > 0 :
            msg2.speed = int(msg.axes[1]*50)
        else :
            msg2.brake = int(msg.axes[1]*(-120))
        msg2.steer = int(msg.axes[3]*(-2000))
        msg2.gear = 0
    elif msg.buttons[4] == 0 and msg.buttons[5] == 1:
        if msg.axes[1] > 0 :
            msg2.speed = int(msg.axes[1]*50)
        else :
            msg2.brake = int(msg.axes[1]*(-120))
        msg2.steer = int(msg.axes[3]*(-2000))
        msg2.gear = 2
    elif msg.axes[1] < 0 :
        msg2.brake = int(msg.axes[1]*(-120))
    else :
        msg2.gear = 1; msg2.speed = 0; msg2.steer = 0; #msg2.brake = 0;
    print("gear : %d / speed : %d / steer : %d / brake : %d" %(msg2.gear, msg2.speed, msg2.steer, msg2.brake))
    pub.publish(msg2)

sub = rospy.Subscriber('joy', Joy, callback)
pub = rospy.Publisher('erp42_serial', ESerial, queue_size=100)

rospy.spin()

