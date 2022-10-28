from enum import auto
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int16
from ornibibot.msg import OrnibiBotData
import numpy as np


class OrnibiBot:
    force = np.zeros(3, dtype=np.float64)
    moment = np.zeros(3, dtype=np.float64)
    wing_position={
        "left": 0,
        "right": 0
    }

    def __init__(self, init):
        if init == 1:
            rospy.Subscriber('wing_left', Int16, self.wing_left_callback)
            rospy.Subscriber('wing_right', Int16, self.wing_right_callback)
            rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.force_callback)
            self.stream_data = rospy.Publisher('ornibibot_restream', OrnibiBotData, queue_size=100)

    def wing_left_callback(self, position):
        rospy.loginfo_once("Wing left data is received.")
        self.wing_position["left"] = position.data

        # print(r)
        
    
    def wing_right_callback(self, position):
        rospy.loginfo_once("Wing right data is received.")
        self.wing_position["right"] = position.data

    def force_callback(self, magnitude):
        rospy.loginfo_once("Force data is received.")
        self.force = [magnitude.wrench.force.x, magnitude.wrench.force.y, magnitude.wrench.force.z]
        self.moment = [magnitude.wrench.torque.x, magnitude.wrench.torque.y, magnitude.wrench.torque.z]

        # print(self.fosssrce)


