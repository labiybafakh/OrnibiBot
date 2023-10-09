import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8, Float32
import tkinter as tk
from tkinter import Scale


flapping_frequency = 0
flapping_mode = 0

class CommROS(Node):

    def __init__(self):
        super().__init__('CommROS')
        self.publisher_ = self.create_publisher(UInt8, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        pass

    def timer_callback(self):
        msg = UInt8()
        msg.data = flapping_mode
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1
        pass

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Tkinter with 2 Trackbars in Class")
        
        # Create the first trackbar (slider)
        self.slider1 = Scale(self.master, from_=0, to=100, orient=tk.HORIZONTAL, label="Slider 1")
        self.slider1.pack(pady=20)
        self.slider1.bind("<Motion>", self.on_slider_change)
        flapping_frequency = self.slider1

        # Create the second trackbar (slider)
        self.slider2 = Scale(self.master, from_=0, to=100, orient=tk.HORIZONTAL, label="Slider 2")
        self.slider2.pack(pady=20)
        self.slider2.bind("<Motion>", self.on_slider_change)
        flapping_mode = self.slider2

    def on_slider_change(self, event):
        # This function is called whenever a slider value changes
        print(f"Slider 1 value: {self.slider1.get()}")
        print(f"Slider 2 value: {self.slider2.get()}")



if __name__ == '__main__':
    root = tk.Tk()
    app = GUI(root)
    rclpy.init()
    node = CommROS()
    # rclpy.spin(node)
    root.mainloop()