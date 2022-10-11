from enum import auto
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int16
import numpy as np


class OrnibiBot:
    force = np.zeros(3, dtype=np.float64)
    moment = np.zeros(3, dtype=np.float64)
    wing_position={
        "left":np.zeros(1),
        "right":np.zeros(1)
    }

    def __init__(self) -> None:
        rospy.Subscriber('wing_left', Int16, self.wing_left_callback)
        rospy.Subscriber('wing_right', Int16, self.wing_right_callback)
        rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.force_callback)

    def wing_left_callback(self, position):
        r = self.wing_position["left"] = position.data

        # print(r)
        
    
    def wing_right_callback(self, position):
        self.wing_position["right"] = position.data

    def force_callback(self, magnitude):
        self.force = [magnitude.wrench.force.x, magnitude.wrench.force.y, magnitude.wrench.force.z]
        self.moment = [magnitude.wrench.torque.x, magnitude.wrench.torque.y, magnitude.wrench.torque.z]

        print(self.force)

if __name__ == '__main__':
    rospy.init_node('OrnibiBot', anonymous=False)
    robot = OrnibiBot()

    while True:
        rospy.Rate(100).sleep()
