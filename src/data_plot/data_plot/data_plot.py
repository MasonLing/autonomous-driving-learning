import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        # print("data_plot_node creat!!!") #不是特别专业用print
        # 自带的打印函数(自带信息，节点/时间等会打印到终端)
        self.get_logger().info("!!! data_plot_node was cread!!")

def main():
    rclpy.init()
    plot_node = PlotData()
    # print("1111")
    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()#防止按下CTRL + C无法关闭


if __name__ == '__main__':
    main()
