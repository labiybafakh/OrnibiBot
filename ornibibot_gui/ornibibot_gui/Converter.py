import rclpy
from rclpy.node import Node
# from ornibibot_msgs.msg import OrnibiBotData
import pandas as pd
import numpy as np



header_list = [
                "OrnibiBotData.time", "OrnibiBotData.actual_left", "OrnibiBotData.actual_right", "OrnibiBotData.desired_left", "OrnibiBotData.desired_right",
                "OrnibiBotData.power_left", "OrnibiBotData.power_right", "OrnibiBotData.F_x", "OrnibiBotData.F_y", "OrnibiBotData.F_z",
                "OrnibiBotData.M_x", "OrnibiBotData.M_y", "OrnibiBotData.M_z"   
            ]

data_example = [0,0.1,0.5,0.1,0.3,0,0.1,0.5,0.1,0.3,0,0.1,0.5]

data_example = np.array(data_example)


# print(len(data_example))

data_frame = pd.DataFrame(data_example, columns= header_list)

print(data_frame)

