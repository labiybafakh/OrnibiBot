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
        self.publisher_frequency = self.create_publisher(Float32, 'flapping_frequency', 5)
        self.publisher_mode = self.create_publisher(UInt8, 'flapping_mode', 5)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = flapping_mode
        self.publisher_frequency.publish(msg)
        
class GUI():
    def __init__(self, master, node):
        self.master = master
        self.node = node
        self.master.title("Tkinter with 2 Trackbars in Class")
        # self.master.after(50, self.ros_spin)
        
        # Create the first trackbar (slider)
        self.slider1 = Scale(self.master, from_=0, to=100, orient=tk.HORIZONTAL, label="Slider 1")
        self.slider1.pack(pady=20)
        self.slider1.bind("<Motion>", self.on_slider_change)

        # Create the second trackbar (slider)
        self.slider2 = Scale(self.master, from_=0, to=100, orient=tk.HORIZONTAL, label="Slider 2")
        self.slider2.pack(pady=20)
        self.slider2.bind("<Motion>", self.on_slider_change)

    def on_slider_change(self, event):
        # This function is called whenever a slider value changes
        # print(f"Slider 1 value: {self.slider1.get()}")
        # print(f"Slider 2 value: {self.slider2.get()}")
        global flapping_frequency, flapping_mode
        flapping_frequency = self.slider1.get()
        flapping_mode = self.slider2.get()

    def close_window(self):
        root.destroy()
        root.quit()
        rclpy.shutdown()



    def ros_spin(self):
        # rclpy.spin_once(self.node, timeout_sec=0.05)
        # self.master.after(50, self.ros_spin)
        pass


if __name__ == '__main__':
    root = tk.Tk()
    rclpy.init()

    node = CommROS()
    app = GUI(root, node)
    root.bind('q', app.close_window)
    # rclpy.spin(node)
    root.mainloop()
