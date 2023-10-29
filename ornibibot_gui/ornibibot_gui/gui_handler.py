import rclpy
from rclpy.node import Node
import random

# from std_msgs.msg import UInt8, Float32
from ornibibot_msgs.msg import OrnibiBotGUI
import tkinter as tk
from tkinter import Scale, Button, Radiobutton
import matplotlib
from itertools import count

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


flapping_frequency = 0.0
flapping_mode = 0

class CommROS(Node):

    def __init__(self):
        super().__init__('CommROS')
        self.publisher_frequency = self.create_publisher(OrnibiBotGUI, 'flapping_frequency_mode', 5)
        timer_period = 0.05  # seconds
        self.timer_publish = self.create_timer(timer_period, self.timer_publisher)
        # self.timer_mode = self.create_timer(timer_period, self.timer_callback_mode)

    def timer_publisher(self):
        msg = OrnibiBotGUI()
        msg.time = self.get_clock().now().to_msg()
        msg.flapping_frequency = flapping_frequency
        msg.flapping_mode = flapping_mode
        self.publisher_frequency.publish(msg)

class GUI():
    def __init__(self, master, node):
        super().__init__()
        self.master = master
        self.node = node
        self.master.title("OrnibiBot GUI")
        self.master.after(50, self.ros_spin)
        
        self.interfacing()

        # values for first graph
        self.x_vals = []
        self.y_vals = []
        # values for second graph
        self.y_vals2 = []

        self.index = count()
        self.index2 = count()

    def interfacing(self):
        # Create the first trackbar (slider) for Frequency
        self.slider_frequency = Scale(self.master, from_=0.0, to=10.0, length= 300 ,orient=tk.VERTICAL, label="Flappign Frequency")
        self.slider_frequency.pack(pady=50)
        self.slider_frequency.bind("<Motion>", self.on_slider_change)

        self.button = Button(self.master, text="Zero Frequency", command=self.button_zero_frequency)
        self.button.pack(anchor="center")

        self.label_flapping_mode = tk.Label(self.master, text="flapping mode")
        # Dictionary to create multiple buttons
        self.values = {"Sine" : 1,
                "Triangle" : 5,
                "Square" : 3,
                "Saw" : 7,
                "Rev-Saw" : 9}
        
        self.clicked_variable = tk.IntVar()
        
        for (text, value) in self.values.items():
            Radiobutton(self.master, text = text, variable = self.clicked_variable, 
            value = value, command=self.flapping_mode_callback).pack(ipady = 5)

        # self.canvas = FigureCanvasTkAgg(plt.gcf(), master= self.master)
        # self.canvas.get_tk_widget().grid(column=0, row=1)

        # plt.gcf().subplots(1,2)

        # self.ani = FuncAnimation(plt.gcf(), self.animate_plot, interval= 1000, blit= False)

    def on_slider_change(self, event):
        global flapping_frequency, flapping_mode
        flapping_frequency = float(self.slider_frequency.get())

    def close_window(self):
        root.destroy()
        root.quit()
        rclpy.shutdown()

    def flapping_mode_callback(self):
        global flapping_mode
        flapping_mode = self.clicked_variable.get()

    def button_zero_frequency(self):
        global flapping_frequency
        self.slider_frequency.set(0.0)
        flapping_frequency = 0.0
        
    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.master.after(50, self.ros_spin)

    def animate_plot(self):
        self.x_vals.append(next(self.index))
        self.y_vals.append(random.randint(0, 5))
        self.y_vals2.append(random.randint(0, 5))
        # Get all axes of figure
        ax1, ax2 = plt.gcf().get_axes()
        # Clear current data
        ax1.cla()
        ax2.cla()
        # Plot new data
        ax1.plot(self.x_vals, self.y_vals)
        ax2.plot(self.x_vals, self.y_vals2)

if __name__ == '__main__':
    root = tk.Tk()
    rclpy.init()

    node = CommROS()
    app = GUI(root, node)
    root.bind('q', app.close_window)
    # rclpy.spin(node)
    root.mainloop()
