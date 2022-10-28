import rospy
import math
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int16
from ornibibot.msg import OrnibiBotData
from ornibibot_core import OrnibiBot

if __name__ == '__main__':
    rospy.init_node('OrnibiBot', anonymous=False)
    robot = OrnibiBot(1)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        rospy.loginfo_once("Data are streamed.")
        
        ornibibot_data = OrnibiBotData()
        ornibibot_data.header.stamp = rospy.Time.now()
        ornibibot_data.header.frame_id = 'ornibibot'
        ornibibot_data.force.x = robot.force[0]
        ornibibot_data.force.y = robot.force[1]
        ornibibot_data.force.z = robot.force[2]
        ornibibot_data.moment.x = robot.moment[0]
        ornibibot_data.moment.y = robot.moment[1]
        ornibibot_data.moment.z = robot.moment[2]
        ornibibot_data.wing_left.data = math.radians(robot.wing_position["left"])
        ornibibot_data.wing_right.data = math.radians(robot.wing_position["right"])

        robot.stream_data.publish(ornibibot_data)
        rospy.Rate(1000).sleep()