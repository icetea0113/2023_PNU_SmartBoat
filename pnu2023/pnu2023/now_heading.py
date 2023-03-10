import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from mechaship_interfaces.msg import Heading

import math


class MagnetometerNode(Node):

    def __init__(self):
        super().__init__('magnetometer_node')
        self.mag_sub = self.create_subscription(MagneticField, '/imu/data', self.mag_callback, 10)
        self.qua_sub = self.create_subscription(Imu, '/imu/data', self.qua_callback, 10)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
                
        self.now_heading = self.create_publisher(Heading, 'heading', qos_profile)
        
        self.get_logger().info('Magnetometer node initialized')
        self.roll_x = 10000  # 값을 정상적으로 계산하는지 확인하기 위해 초기값을 10000으로 설정
        self.pitch_y = 10000
        self.yaw_z = 10000

    def qua_callback(self, msg):
        qx = msg.orientation.x  #Quaternian vector element - x axis
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        (self.roll_x, self.pitch_y, self.yaw_z) = self.euler_from_quaternion(qx, qy, qz, qw)
        
        heading = Heading()
        [heading.roll, heading.pitch, heading.yaw] = [self.roll_x, self.pitch_y, self.yaw_z]
        
        self.get_logger().info("roll : {}, pitch : {}, yaw: {}".format(self.roll_x, self.pitch_y, self.yaw_z))

    def euler_from_quaternion(self, qx, qy, qz, qw):
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        roll_x = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.degrees(math.asin(t2))

        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_z = math.degrees(math.atan2(t3, t4))

        return (roll_x, pitch_y, yaw_z) # in radians
        


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()