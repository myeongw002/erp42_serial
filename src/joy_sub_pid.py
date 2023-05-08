#!/usr/bin/env python3

#rosrun joy joy_node first
import rospy
from sensor_msgs.msg import Joy 
from erp42_serial.msg import setState
import time 

max_speed = 15 #actual speed(kph) *10
max_degree = 28
max_brake = -150  

axes = [0,0,0,0,0,0,0,0,0,0]
buttons = []

velocity = 0
steer = 0
gear1 = 0
gear2 = 0

def callback(msg):

    global velocity,steer,gear1,gear2
    
    axes = list(msg.axes)
    buttons = list(msg.buttons) 
    #print(f"axes:{axes}")
    #print(f"buttons:{buttons}")
    velocity = axes[1] 
    steer = int(axes[3] * -max_degree)
    gear1 = int(axes[2])
    gear2 = int(axes[5])
    
 
msg = setState()
pub = rospy.Publisher('set_state', setState, queue_size=100)
   
#gear 0: forward 1: neutral 2: backward

def joy_publisher(velocity=0,steer=0,gear1=1,gear2=1):

    if velocity < 0 :
        msg.set_brake = int(velocity * max_brake)
        msg.set_velocity = 0
    
    else:
        msg.set_brake = 0    
        
        if gear1 ==-1 and gear2 != -1:
            msg.set_gear = 0
            msg.set_velocity = int(velocity * max_speed)

        elif gear2 == -1 and gear1 != -1:
            msg.set_gear = 2
            msg.set_velocity = int(velocity * max_speed)
    
        else:
            msg.set_gear = 1
            msg.set_velocity = 0
        
    msg.set_degree = steer
        

    #msg.shutdown= 0
    
    print(f'speed:{msg.set_velocity}, brake:{msg.set_brake}, steer:{msg.set_degree}, gear:{msg.set_gear}')  
    pub.publish(msg) 

    
    
if __name__ == '__main__':
    rospy.init_node('joy_sub_pid')
    rate = rospy.Rate(1000)
    print('Joy stick node start')
    rospy.Subscriber("joy", Joy, callback)
    
    while not rospy.is_shutdown():
        joy_publisher(velocity,steer,gear1,gear2)
        rate.sleep()
        

