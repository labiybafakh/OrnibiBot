import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int16
from ornibibot.msg import OrnibiBotData
from ornibibot_core import OrnibiBot


class DataPlot():
    def __init__(self):
        self.robot = OrnibiBot(0)
        self.timestamp = rospy.Time.now()
        rospy.Subscriber("ornibibot_restream", OrnibiBotData, self.data_callback)
        self.force={
            "thrust": [],
            "lateral": [],
            "lift": []
        }
        self.moment={
            "mx": [],
            "my": [],
            "mz": []
        }
        self.wing_position={
            "left": [],
            "right": []
        }
        self.iteration = 0

    def collect_data(self):
        #Force
        self.force["thrust"].append(self.robot.force[0])
        self.force["lateral"].append(self.robot.force[1])
        self.force["lift"].append(self.robot.force[2])

        #Moment
        self.moment["mx"].append(self.robot.moment[0])
        self.moment["my"].append(self.robot.moment[1])
        self.moment["mz"].append(self.robot.force[2])

        #Wing Position
        self.wing_position["left"].append(self.robot.wing_position["left"])
        self.wing_position["right"].append(self.robot.wing_position["right"])

        print(self.wing_position["right"])

    def data_callback(self, data):
        #Gather all OrnibiBot data streamed
        rospy.loginfo_once("Data are gathered.")
        self.robot.timestamp = data.header.stamp
        self.robot.force = [data.force.x, data.force.y, data.force.z]
        self.robot.moment = [data.moment.x, data.moment.y, data.moment.z]
        self.robot.wing_position["left"] = data.wing_left.data
        self.robot.wing_position["right"] = data.wing_right.data

        self.collect_data()



if __name__ == '__main__':
    rospy.init_node('OrnibiBot_plotter', anonymous=False)
    plotter = DataPlot()
    iteration = 0
    while not rospy.is_shutdown():
        # iteration = iteration + 1
        # print(iteration)
        # rospy.spin()
        rospy.Rate(1000).sleep()