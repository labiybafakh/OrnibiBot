import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from ornibibot_gui import uiDialog
from random import Random, random
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import WrenchStamped
import numpy as np


class GUI(QMainWindow):
    def __init__(self, parent=None):
        super(GUI, self).__init__(parent)
        self.ui = uiDialog()

        self.force = np.zeros(3)

        rospy.Rate(50)

        rospy.Subscriber("leptrino_force_torque/force_torque", WrenchStamped, self.force_sensor_callback)
        rospy.Subscriber("wing_left", Int16, self.wing_left_callback)
        rospy.Subscriber("wing_right", Int16, self.wing_right_callback)
        
    
    def force_sensor_callback(self, msg):
        self.force_x.append(msg.wrench.force.x)
        self.force_y.append(msg.wrench.force.y)
        self.force_z.append(msg.wrench.force.z)
        np.append(self.force, [msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z])
        self.time.append(rospy.Time.now().to_nsec()/1000)
        self.counter_force_data +=1
        # self.index = self.counter_force_data
        # pass
    
    def wing_left_callback(self, msg):
        pass

    def wing_right_callback(self, msg):
        
        pass


    def update_data(self):
        self.xdata.append(self.time)
        self.ydata.append(self.force_x)
        self.canvas.axes.cla()
        self.canvas.axes.plot(self.xdata, self.ydata, 'r')

        self.canvas.draw()
        self.canvas.flush_events()

    def connect_callback(self):
        
        if  self.flag_connect == 0:
            self.connect.setText("Disconnect")
            self.flag_connect = 1
        else:
            self.connect.setText("Connect")
            self.flag_connect = 0

    def measure_force_callback(self):
        pass

    def initial_callback(self):
        
        pass
        # self.tail_rolling
        # message = QMessageBox()
        # message.setWindowTitle("Warn")
        # message.setText("ROSCORE is not running")

        # return message.exec_()


    def update(self):
        self.label.adjustSize()


if __name__ == '__main__':
    rospy.init_node("OrnibiBot GUI")
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())
