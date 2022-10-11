#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64MultiArray
import pygame


i=5
yaw     = 0
pitch   = 0
roll    = 0
def callbackIMU(attitude):
    global yaw, pitch, roll
    yaw     = attitude.data[0]
    pitch   = attitude.data[1]
    roll    = attitude.data[2]


def main():    
    pygame.init()
    num_joysticks = pygame.joystick.get_count()
    if num_joysticks == 0:
        raise ValueError("No joysticks attached!")
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    pub     = rospy.Publisher('RC', Int64MultiArray, queue_size=10 )
    sub     = rospy.Subscriber('IMU', Int64MultiArray, callbackIMU )
    rospy.init_node('OrnibiBotPC', anonymous=False)
    r       = rospy.Rate(50)

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            pass
        [throttle, elevator, rudder] = int(joystick.get_axis(1)*-100), int(joystick.get_axis(3)*-100), int(joystick.get_axis(2)*100)
        #print("{},{},{}".format(throttle,rudder,elevator))        
        dataRC  = Int64MultiArray(data=[throttle, rudder, elevator])
        pub.publish(dataRC)
        print("{},{},{}".format(yaw,pitch,roll))        

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass

