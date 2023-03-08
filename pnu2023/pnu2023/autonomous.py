import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray, Heading, Target
from mechaship_interfaces.srv import Key, ThrottlePercentage, RGBColor

"""
Notes:
    주행하도록 해주는 파일.
    
    ** 현재 보완해야할 점:
    1. key를 제어하는 부분을 autonomous.py에 옮기도록 한다.
        - lidar로 계산한 heading 각도를 msg로 publish
        - 줄무늬 부표에 따라 제어를 할 수 있는 코드를 새로 만들도록 한다.
        - 각도 계산은 어떻게 하지?
            1) 화각을 사용해 부분적인 각도 계산?
    ㄴ
    순서도 변경)
    1. 줄무늬 부표가 보이는가?
        1) 두 개의 줄무늬 부표 사이로 이동하도록 하기.
        2) if문을 2개 작성하여 각 줄무늬 부표에 따라 각도 삭제하기.
    2-1) 줄무늬 부표가 보인다면 그 조건을 통해 만들어진 각도를 목표 각도로 설정
    2-2) 줄무늬 부표가 보이지 않는다면, 목표지점을 향한 각도를 목표 각도로 설정
    3. 설정한 목표각도를 최우선적으로 주행하도록 하나, 전방에 장애물이 있다면 그에 따라 주행 각도를 변경하도록 함.
    
"""

class Autonomous(Node):
    def __init__(self):
        super().__init__(
            "classify_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
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
        
        self.obstacle_subscription = self.create_subscription(
            ClassificationArray, "/Obstacle", self.obstacle_listener_callback, qos_profile
        )
        self.wall_subscription = self.create_subscription(
            ClassificationArray, "/Wall", self.wall_listener_callback, qos_profile
        )
        self.now_heading_subscription = self.create_subscription(
            Heading, "heading", self.heading_listener_callback, qos_profile
        )

        self.obstacle_subscription  # prevent unused variable warning
        self.wall_subscription  # prevent unused variable warning

        self.obstacle = ClassificationArray()
        self.wall = ClassificationArray()
        
        self.create_timer(0.1, self.navigate)

        self.set_key_handler = self.create_client(Key, "/actuators/key/set")
        self.set_throttle_handler = self.create_client(
            ThrottlePercentage, "/actuators/throttle/set_percentage"
        )
        

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
        
        self.target_heading = 0
        
        # 다음 변수는 최종 서보모터로 입력을 넣어줄 키 각도이다.
        self.final_key_angle = 90
        self.throttle_value = 50
        
        
    def listener_callback(self, data):
        
        self.rhos = []
        self.thetas = []
        self.angle = 0
        self.d_theta = 0
        
        #---------------------------------------------------#
        self.angle = data.angle_min
        self.d_theta = data.angle_increment
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)) and (scan_data <= 4):
                self.rhos.append(scan_data)
                self.thetas.append(self.angle)
            self.angle += self.d_theta

    
        # these list use for step a ~ step c and consist of (rho, angle)
        self.group_1 = []
        self.obstacle = []
        self.wall = []
        self.key_angle = [5 * angle for angle in range(0,37)]
        
        '''
        다음 변수들은 subscription을 만들어야함.
        1. 현재 선수 각도 (IMU)
        2. 목표지점을 향한 각도 (GPS + IMU)
        '''
        self.now_heading = 0
        self.target_heading = 0
        
        # 다음 변수는 최종 서보모터로 입력을 넣어줄 키 각도이다.
        self.final_key_angle = 90
        
        self.grouping()
        self.decision_group()
        self.expand_obstacle()
        self.expand_wall()
        self.detect_angle()
    
    def wall_listener_callback(self, data):
        rho = data.classifications[0]
        theta = data.classifications[1]
        for i in range(rho):
            self.wall.append((rho[i],theta[i]))
            
    def obstacle_listener_callback(self, data):
        rho = data.classifications[0]
        theta = data.classifications[1]
        for i in range(rho):
            self.obstacle.append((rho[i],theta[i]))
    
    def heading_listener_callback(self, data):
        # Heading -> [pitch, roll, yaw]
        self.now_heading = data.yaw
                    
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
                p2p_angle = math.atan((end_point_otho[1]-start_point_otho[1])/(end_point_otho[0]-start_point_otho[0]))
                
                height = radius * math.sin(p2p_angle)
                width = radius * math.cos(p2p_angle)
                
                expand_1 = (middle_point[0]+width, middle_point[1]+height)
                expand_2 = (middle_point[0]-width, middle_point[1]-height)
                expand_1_angle = math.degrees(math.atan(expand_1[1]/expand_1[0]))
                expand_2_angle = math.degrees(math.atan(expand_2[1]/expand_2[0]))
                
                key_angle = self.key_angle.copy()
                
                for angle in self.key_angle:
                    if min(expand_1_angle, expand_2_angle) < angle < max(expand_1_angle, expand_2_angle):
                        key_angle.remove(angle)
                
                self.key_angle = key_angle
    
    def expand_wall(self): # step h
        # self.wall -> 벽들의 좌표들의 집합 [(wall1),(wall2),(wall3)...]
        if len(self.wall) > 0 :
            # 벽까지의 최소거리 집합
            min_wall_dist = []
            min_wall_angle = []
            for wall in self.wall:
                rhos = [rho[0] for rho in wall]
                min_rho = min(rhos)
                min_wall_angle.append(wall[rhos.index(min_rho)][1])
                min_wall_dist.append(min_rho)
            #---------------------------------
            if min(min_wall_dist) < self.safety_distance:
                min_idx = min_wall_dist.index(min(min_wall_dist))
                if min_wall_angle[min_idx] < 70:
                    self.final_key_angle = 60
                    self.get_logger().info("왼쪽에 벽이 있음!")
                elif min_wall_angle[min_idx] > 110:
                    self.final_key_angle = 120
                    self.get_logger().info("오른쪽에 벽이 있음!")
                
        
        # if len(self.wall) > 0 :
        #     min_wall = self.wall[0].copy()
        #     for wall in self.wall:
        #         if min_wall[0] > wall[0]:
        #             min_wall = wall
                    
        #     for idx in range(len(self.wall)):
        #         expand_dist = self.safety_distance / math.cos(self.wall[idx][0][1] - min_wall[0][1])
        #         if self.wall[idx][0][0] < expand_dist :
        #             print("주변에 벽이 있어요!!")
        #             #추가적인 제어 코드 설정
                

    def detect_angle(self): #step f~g
        '''
        heading 목표 방향을 수신한 뒤, key_angle과 가장 가까운 각도로 주행하도록 함.
        &&(And) 벽과의 최소거리가 0이 넘어야 주행이 가능함.
        '''
        abs_key_angle = [5 * angle for angle in range(1,37)]
        angle = abs_key_angle.copy()
        for idx in range(len(abs_key_angle)):
            abs_key_angle[idx] = abs(abs_key_angle[idx] - self.target_heading)
        min_idx = abs_key_angle.index(min(abs_key_angle))
        target_heading = angle[min_idx]

        now_heading = self.now_heading.yaw ## <-- 체크할 것 / degree of yaw 
        
        if target_heading not in self.key_angle:
            second_key_angle = self.key_angle.copy()
            for idx in range(len(second_key_angle)):
                second_key_angle[idx] = abs(second_key_angle[idx] - self.target_heading)
            second_min_idx = [idx for idx, value in enumerate(second_key_angle) if value == min(second_key_angle)]
            # 최솟값을 가지는 index 배열
            if len(second_min_idx) == 1:
                self.final_key_angle = self.key_angle[second_min_idx[0]]
            # 최솟값을 가지는 index가 2개라면 아래의 if문 작동
            elif (abs(now_heading - self.key_angle[second_min_idx[0]]) 
                  - abs(now_heading - self.key_angle[second_min_idx[1]]) < 0):
                self.final_key_angle = self.key_angle[second_min_idx[0]]
            else:
                self.final_key_angle = self.key_angle[second_min_idx[1]]
        else:
            self.final_key_angle = target_heading
        
        self.lidar_target_publisher.publish(self.final_key_angle)

def main(args=None):
    rclpy.init(args=args)
    node = Autonomous()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()