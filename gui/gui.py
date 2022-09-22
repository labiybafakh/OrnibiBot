from random import Random, random
import rospy
from geometry_msgs.msg import WrenchStamped
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QWidget
import sys
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import numpy as np


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=400, height=400, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.setObjectName("MainWindow")
        # self.resize(740, 543)
        self.setFixedSize(840, 840)
        self.setMouseTracking(False)
        self.setDocumentMode(True)
        self.setGeometry(200, 200, 300, 300)
        self.setWindowTitle("TESTING")
        rospy.init_node('GUI', anonymous=False)

        self.initUI()

        #Timer
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_data)
        self.timer.start()

        #initilize
        self.flag_connect=0
        self.counter_force_data=0

    def initUI(self):
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(510, 200, 170, 271))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        
        self.init_navigation_section()
        self.init_button_section()
        self.init_plotter_section()

        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 740, 20))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi(self)
        QtCore.QMetaObject.connectSlotsByName(self)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "OrnibiBot Controller"))
        self.frequency_title.setPlainText(_translate("MainWindow", "Frequency"))
        self.rolling_title.setPlainText(_translate("MainWindow", "Rolling"))
        self.pitching_title.setPlainText(_translate("MainWindow", "Pitching"))

        self.connect.setText(_translate("MainWindow", "Connect"))
        self.measure_force.setText(_translate("MainWindow", "Measure Force"))
        self.initial.setText(_translate("MainWindow", "Initial"))

    def init_navigation_section(self):
        #layout navigation    
        self.navigation_layout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.navigation_layout.setContentsMargins(0, 0, 0, 0)
        self.navigation_layout.setObjectName("navigation_layout")

        #frequency slider
        self.frequency = QtWidgets.QSlider(self.horizontalLayoutWidget_2)
        self.frequency.setAutoFillBackground(False)
        self.frequency.setMaximum(100)
        self.frequency.setPageStep(5)
        self.frequency.setProperty("value", 0)
        self.frequency.setSliderPosition(0)
        self.frequency.setOrientation(QtCore.Qt.Vertical)
        self.frequency.setObjectName("frequency")
        self.navigation_layout.addWidget(self.frequency)

        #rolling dial
        self.tail_rolling = QtWidgets.QDial(self.horizontalLayoutWidget_2)
        self.tail_rolling.setMinimum(-60)
        self.tail_rolling.setMaximum(60)
        self.tail_rolling.setObjectName("tail_rolling")
        self.navigation_layout.addWidget(self.tail_rolling)

        #pitching slider
        self.tail_pitching = QtWidgets.QSlider(self.horizontalLayoutWidget_2)
        self.tail_pitching.setMinimum(-60)
        self.tail_pitching.setMaximum(60)
        self.tail_pitching.setPageStep(5)
        self.tail_pitching.setOrientation(QtCore.Qt.Vertical)
        self.tail_pitching.setObjectName("tail_pitching")
        self.navigation_layout.addWidget(self.tail_pitching)

        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(460, 140, 261, 31))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        
        self.navigation_title_layout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.navigation_title_layout.setContentsMargins(0, 0, 0, 0)
        self.navigation_title_layout.setObjectName("navigation_title_layout")
        
        self.frequency_title = QtWidgets.QPlainTextEdit(self.horizontalLayoutWidget)
        self.frequency_title.setObjectName("frequency_title")
        self.navigation_title_layout.addWidget(self.frequency_title)
        
        self.rolling_title = QtWidgets.QPlainTextEdit(self.horizontalLayoutWidget)
        self.rolling_title.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.rolling_title.setObjectName("rolling_title")
        self.navigation_title_layout.addWidget(self.rolling_title)
        
        self.pitching_title = QtWidgets.QPlainTextEdit(self.horizontalLayoutWidget)
        self.pitching_title.setObjectName("pitching_title")
        self.navigation_title_layout.addWidget(self.pitching_title)

    def init_button_section(self):
        self.connect = QtWidgets.QPushButton(self.centralwidget)
        self.connect.setGeometry(QtCore.QRect(20, 30, 100, 23))
        self.connect.setObjectName("connect")
        self.connect.clicked.connect(self.connect_callback)

        self.measure_force = QtWidgets.QPushButton(self.centralwidget)
        self.measure_force.setGeometry(QtCore.QRect(170, 30, 100, 23))
        self.measure_force.setObjectName("measure_force")
        self.measure_force.clicked.connect(self.measure_force_callback)

        self.initial = QtWidgets.QPushButton(self.centralwidget)
        self.initial.setGeometry(QtCore.QRect(320, 30, 100, 23))
        self.initial.setObjectName("initial")
        self.initial.clicked.connect(self.initial_callback)

    def init_plotter_section(self):
        #Init ros subscriber
        self.sub_force = rospy.Subscriber("leptrino_force_torque/force_torque", WrenchStamped, self.callback_force_sensor)
        self.force_x = []
        self.force_y = []
        self.force_z = []
        self.time = []
        self.force = np.array([0,0,0], dtype=np.float32)


        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 100, 421, 401))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        
        self.graph_layout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.graph_layout.setContentsMargins(0, 0, 0, 0)
        self.graph_layout.setObjectName("graph_layout")


        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        # self.canvas.axes.plot([0,1,2,3,4], [10,1,20,3,40])
        self.xdata = []
        self.ydata = []
        self.update_data()
        # self.setCentralWidget(sc)

        self.toolbar = NavigationToolbar(self.canvas, self)

        self.graph_layout.addWidget(self.toolbar)
        self.graph_layout.addWidget(self.canvas)

        self.plotter_force = QtWidgets.QWidget(self.verticalLayoutWidget)
        self.plotter_force.setObjectName("plotter_force")
        self.graph_layout.addWidget(self.plotter_force)

        self.plotter_moment = QtWidgets.QWidget(self.verticalLayoutWidget)
        self.plotter_moment.setObjectName("plotter_moment")
        self.graph_layout.addWidget(self.plotter_moment)

        self.show()
    
    def callback_force_sensor(self, msg):
        self.force_x.append(msg.wrench.force.x)
        self.force_y.append(msg.wrench.force.y)
        self.force_z.append(msg.wrench.force.z)
        np.append(self.force, [msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z])
        self.time.append(rospy.Time.now().to_nsec()/1000)
        self.counter_force_data +=1
        # self.index = self.counter_force_data
        # pass


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



def window():
    app = QApplication(sys.argv)
    win = MyWindow()

    win.show()
    sys.exit(app.exec_())

window()