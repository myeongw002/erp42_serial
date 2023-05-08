#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from erp42_serial.msg import setState


rospy.init_node('joy_pid_erp42')

print('start! joy_pid_erp42')

msg2 = setState()

setpoint = 0
def callback(msg):
    global setpoint
    global pid_speed
    set_velocity = 2 # m/s
    set_degree = 28  # degree
    set_brake = 150
    msg2.set_degree = msg.axes[3]*(-set_degree)
    if msg.axes[1] < -0.1 :
        msg2.set_brake = msg.axes[1]*set_brake*(-1)
    else :
        msg2.set_brake = 0

    if msg.buttons[4] == 1 and msg.buttons[5] == 0: #safe butten
        msg2.set_gear = 0 #forward
        if msg.axes[1] >= 0 :
            msg2.set_velocity = msg.axes[1]*set_velocity
    elif msg.buttons[4] == 0 and msg.buttons[5] == 1: #safe butten
        msg2.set_gear = 2 #backward
        if msg.axes[1] >= 0 :
            msg2.set_velocity = msg.axes[1]*set_velocity*(-1)
    else :                #N
        msg2.set_gear = 1
        msg2.set_velocity = 0


sub = rospy.Subscriber('joy', Joy, callback)
pub = rospy.Publisher('set_state', setState, queue_size=100)


#rospy.spin()
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    pub.publish(msg2)
    rate.sleep()
    
    
