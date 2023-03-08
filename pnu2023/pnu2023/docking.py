import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray, Heading
from mechaship_interfaces.srv import Key, ThrottlePercentage, RGBColor

class docking(Node):
    def __init__():
        super().__init__(
            "docking_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
    
    def image(self): # 도킹 부분 표식 구분 함수
        pass
    
    def led_on(self): # 구분한 그림에 따른 led 제어 함수
        pass
    
    def avoid_wall(self): # 벽과의 거리를 조절하여 운전하는 함수
        pass
    
    def parking(self): #주차하는 함수
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = docking()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()