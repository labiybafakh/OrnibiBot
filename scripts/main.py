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

    def __init__(self) -> None:
        rospy.Subscriber('wing_left', Int16, self.wing_left_callback)
        rospy.Subscriber('wing_right', Int16, self.wing_right_callback)
        rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.force_callback)
        self.stream_data = rospy.Publisher('ornibibot_restream', OrnibiBotData, queue_size=100)

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

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        

        ornibibot_data = OrnibiBotData()
        ornibibot_data.header.stamp = rospy.Time.now()
        ornibibot_data.header.frame_id = 'ornibibot'
        ornibibot_data.force.x = robot.force[0]
        ornibibot_data.force.y = robot.force[1]
        ornibibot_data.force.z = robot.force[2]
        ornibibot_data.moment.x = robot.moment[0]
        ornibibot_data.moment.y = robot.moment[1]
        ornibibot_data.moment.z = robot.moment[2]
        ornibibot_data.wing_left.data = int(robot.wing_position["left"])
        ornibibot_data.wing_right.data = int(robot.wing_position["right"])

        robot.stream_data.publish(ornibibot_data)
        rospy.Rate(100).sleep()
