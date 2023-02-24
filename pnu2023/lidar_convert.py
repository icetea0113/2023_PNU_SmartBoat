import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray
from mechaship_interfaces.srv import Key, ThrottlePercentage, RGBColor

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
                Parameter("grouping_threshold", Parameter.Type.DOUBLE, 9.0),
            ).get_parameter_value().double_value
        )
        self.max_gap_in_group = (
            self.get_parameter_or(
                "max_gap_in_group",
                Parameter("max_gap_in_group", Parameter.Type.DOUBLE, 0.1),
            ).get_parameter_value().double_value
        )
        self.safety_distance = (
            self.get_parameter_or(
                "safety_distance",
                Parameter("safety_distance", Parameter.Type.DOUBLE, 1.0),
            ).get_parameter_value().double_value
        )
        self.get_logger().info("max_gap_in_group: %sm" % (self.max_gap_in_group))
        self.get_logger().info("grouping_threshold: %s개" % (self.grouping_threshold))
        self.get_logger().info("safety_distance: %sm" % (self.safety_distance))
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_profile
        )

        self.create_timer(0.1, self.navigate)

        self.set_key_handler = self.create_client(Key, "/actuators/key/set")
        self.set_throttle_handler = self.create_client(
            ThrottlePercentage, "/actuators/throttle/set_percentage"
        )
        
        
    def listener_callback(self, data):
        self.rhos = []
        self.thetas = []
        self.angle = 0
        self.d_theta = 0

        # these list use for step a ~ step c and consist of (rho, angle)
        self.group_1 = []
        self.obstacle = []
        self.wall = []
        self.key_angle = [5 * angle for angle in range(0,37)]
        
        '''
        다음 변수들은 subscription을 만들어야함.
        1. 목표지점을 향한 각도 (GPS + IMU)
        2. 현재 선수 각도 (IMU)
        '''
        self.now_heading = 0
        self.target_heading = 0
        
        # 다음 변수는 최종 서보모터로 입력을 넣어줄 키 각도이다.
        self.final_key_angle = 90
        
        #---------------------------------------------------#
        self.angle = data.angle_min
        self.d_theta = data.angle_increment
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)) and (scan_data <= 4):
                self.rhos.append(scan_data)
                self.thetas.append(self.angle)
            self.angle += self.d_theta

        self.grouping()
        self.decision_group()
        self.expand_obstacle()
        self.expand_wall()
        self.detect_angle()
    
    def cosines(self,a,b,theta = -1):
        if theta == -1 :
            theta = self.d_theta
        # self.get_logger().info("starting 코사인법칙_2 process")
        return math.sqrt(math.pow(a,2)+math.pow(b,2)-2*a*b*math.cos(theta))

    
    def trans_polar_to_ortho(self, rho, theta):
        # self.get_logger().info("starting 좌표계 변환 process")
        x = rho * math.cos(theta)
        y = rho * math.sin(theta)
        return (x, y)
    
    def grouping(self): #step b
        sub_group = []
        for index in range(len(self.rhos) - 1):
            a = self.rhos[index]
            a_theta = self.thetas[index]
            b = self.rhos[index + 1]
            b_theta = self.thetas[index + 1]
            dist_p_to_p = self.cosines(a, b)
            if (a, a_theta) not in sub_group:
                sub_group.append((a, a_theta))
            if dist_p_to_p <= self.max_gap_in_group :
                sub_group.append((b, b_theta))
            elif len(sub_group) >= self.grouping_threshold :
                self.group_1.append(sub_group)
                sub_group.clear()
                index += 1
            else :
                sub_group.clear()
                index += 1
        for group in self.group_1:
            print(group[0])
        print("end - 1st group")
        # ----- 여기까지 검토 완료 (비정상) (검토날짜 : 2023_02_24(Fri) _ 15:50)
        # group_1 -> tuple 형태로 (거리, 각도)로 들어가 있음.
        # group_1 에 똑같은 sub_group들이 들어가는 현상 발생
    
    def decision_group(self): #step c ~ step d
        for group in self.group_1:
            temp_group = []
            for index in range(2,len(group)):
                (x1, y1) = self.trans_polar_to_ortho(group[index-2][0], group[index-2][1])
                (x2, y2) = self.trans_polar_to_ortho(group[index-1][0], group[index-1][1])
                (x3, y3) = self.trans_polar_to_ortho(group[index][0], group[index][1])
                
                if group[index-2] not in temp_group:
                    temp_group.append(group[index-2])
                    temp_group.append(group[index-1]) 
                    
                degree = math.degrees(abs(math.atan2((y1-y2),(x1-x2)) - math.atan2((y3-y2),(x3-x2))))
                if degree <= 20 or degree >= 160:
                    temp_group.append(group[index])
                else:
                    distance = self.cosines(temp_group[0][0],temp_group[-1][0],temp_group[0][1]-temp_group[-1][1])
                    if distance >= 1:
                        self.wall.append(temp_group)
                    else :
                        self.obstacle.append(temp_group)
                    index += 2
                
        #------ 위 함수 수정 요함 (재검토) (검토날짜 : 2022_02_24(Fri) _ 16:28)
        # 비정상적으로 obstacle을 쪼개는 현상 발생. 수정 요함
        self.get_logger().info("count of obstacle : %s. " % (len(self.obstacle)))
        self.get_logger().info("count of wall : %s." % (len(self.wall)))
                    
    def expand_obstacle(self): #step e
        if len(self.obstacle) > 0 :
            for group in self.obstacle:
                start_point_polar = group[0]
                end_point_polar = group[-1]
                
                start_point_otho = self.trans_polar_to_ortho(group[0][0], group[0][1])
                end_point_otho = self.trans_polar_to_ortho(group[-1][0], group[-1][1])
                middle_point = [(start_point_otho[0]+end_point_otho[0])/2 , (start_point_otho[1]+end_point_otho[1])/2]
                
                dist_p2p = self.cosines(start_point_polar[0], end_point_polar[0], (end_point_polar[1]-start_point_polar[1]))
                radius = dist_p2p/2.0 + self.safety_distance
                p2p_angle = math.atan2((end_point_otho[1]-start_point_otho[1]),(end_point_otho[0]-start_point_otho[0]))
                
                height = radius * math.sin(p2p_angle)
                width = radius * math.cos(p2p_angle)
                
                expand_1 = (middle_point[0]+width, middle_point[1]+height)
                expand_2 = (middle_point[0]-width, middle_point[1]-height)
                expand_1_angle = math.degrees(math.atan(expand_1[1]/expand_1[0]))
                expand_2_angle = math.degrees(math.atan(expand_2[1]/expand_2[0]))
                
                key_angle = self.key_angle
                
                for angle in self.key_angle:
                    if min(expand_1_angle, expand_2_angle) < angle < max(expand_1_angle, expand_2_angle):
                        key_angle.remove(angle)
                
                self.key_angle = key_angle
    
    def expand_wall(self): # step h
        if len(self.wall) > 0 :
            min_wall = self.wall[0]
            for wall in self.wall:
                if min_wall[0] > wall[0]:
                    min_wall = wall
                    
            for idx in range(len(self.wall)):
                expand_dist = self.safety_distance / math.cos(self.wall[idx][0][1] - min_wall[0][1])
                if self.wall[idx][0][0] < expand_dist :
                    print("주변에 벽이 있어요!!")

    def detect_angle(self): #step f~g
        '''
        heading 목표 방향을 수신한 뒤, key_angle과 가장 가까운 각도로 주행하도록 함.
        &&(And) 벽과의 최소거리가 0이 넘어야 주행이 가능함.
        '''
        self.get_logger().info("starting detect angle process")
        self.key_angle
    
    def navigate(self):
        key = Key.Request()
        key = self.final_key_angle

        self.set_key_handler.call_async(key)

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