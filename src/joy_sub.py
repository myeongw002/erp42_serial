#!/usr/bin/env python3

#rosrun joy joy_node first
import rospy
from sensor_msgs.msg import Joy 
from erp42_serial.msg import ESerial
import time 

max_speed = 250
max_steer = 2000
max_brake = -150  

axes = [0,0,0,0,0,0,0,0,0,0]
buttons = []

def callback(msg):

    global axes
    global buttons
    axes = list(msg.axes)
    buttons = list(msg.buttons) 
    #print(f"axes:{axes}")
    #print(f"buttons:{buttons}")
    joy_publisher()
    
    
def joy_subscriber():
    rospy.init_node('joy_subscriber')
    print('Joy stick node start')
    rospy.Subscriber("joy", Joy, callback)
    #rospy.spin()
    
msg = ESerial()
pub = rospy.Publisher('erp42_serial', ESerial, queue_size=100)
   
#gear 0: forward 1: neutral 2: backward

def joy_publisher():
    if axes[1] >= 0:
        msg.speed = int(axes[1] * max_speed)
        msg.brake = 1
    elif axes[1] < 0:
        msg.brake = int(axes[1] * max_brake)
        msg.speed = 0
        
    if int(axes[2]) == 1 and int(axes[5]) == 1:
        msg.gear = 1
    elif int(axes[2]) == -1 and int(axes[5]) == 1:
        msg.gear = 0
    elif int(axes[2]) == 1 and int(axes[5]) == -1:
        msg.gear = 2
    else:
        msg.gear = 1
            
    msg.steer = int(axes[3] * -max_steer)
    msg.shutdown= 0
    
    print(f'speed:{msg.speed}, brake:{msg.brake}, steer:{msg.steer}, gear:{msg.gear}, shutdown:{msg.shutdown}')  
    pub.publish(msg) 

    
    
if __name__ == '__main__':

    joy_subscriber()

    rospy.spin()
        

