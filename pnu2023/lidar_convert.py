import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray

'''
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
'''

"""
Notes:
    Lidar의 점을 Grouping하고 벽인지 부표인지 구분하는 함수로
    
"""

class Classify(Node):
    def __init__(self):
        super().__init__(
            "classify_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.grouping_threshold = (
            self.get_parameter_or(
                "grouping_threshold",
                Parameter("grouping_threshold", Parameter.Type.DOUBLE, 3.0),
            ).get_parameter_value().double_value
        )
        self.max_gap_in_group = (
            self.get_parameter_or(
                "max_gap_in_group",
                Parameter("max_gap_in_group", Parameter.Type.DOUBLE, 1.0),
            ).get_parameter_value().double_value
        )
        
        self.get_logger().info("max_gap_in_group: %s" % (self.max_gap_in_group))
        self.get_logger().info("grouping_threshold: %s" % (self.grouping_threshold))
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_profile
        )
        self.rhos = []
        self.thetas = []
        self.angle = 0

    def listener_callback(self, data):
        self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
        self.get_logger().info("rad min: %s" % (math.degrees(data.angle_min)))
        self.get_logger().info("rad max: %s" % (math.degrees(data.angle_max)))
        self.get_logger().info("range min: %s" % (data.range_min))
        self.get_logger().info("range max: %s" % (data.range_max))
        
        self.angle = data.angle_min
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)):
                self.rhos.append(scan_data)
                self.thetas.append(angle)
            angle += data.angle_increment
           
    def grouping(self):
        pass

        

def main(args=None):
    rclpy.init(args=args)
    node = Classify()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()