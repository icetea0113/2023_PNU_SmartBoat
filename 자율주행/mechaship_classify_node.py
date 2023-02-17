import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray


class MechashipClassify(Node):
    def __init__(self):
        super().__init__(
            "mechaship_classify_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.classify_method = (
            self.get_parameter_or(
                "classify_method",
                Parameter("classify_method", Parameter.Type.STRING, "lsm"),
            )
            .get_parameter_value()
            .string_value
        )
        self.differential_threshold = (
            self.get_parameter_or(
                "differential_threshold",
                Parameter("differential_threshold", Parameter.Type.DOUBLE, 5.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.max_rho_index_range = (
            self.get_parameter_or(
                "max_rho_index_range",
                Parameter("max_rho_index_range", Parameter.Type.DOUBLE, 0.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.classify_min_length = (
            self.get_parameter_or(
                "classify_min_length",
                Parameter("classify_min_length", Parameter.Type.INTEGER, 10),
            )
            .get_parameter_value()
            .integer_value
        )
        self.classify_max_length = (
            self.get_parameter_or(
                "classify_max_length",
                Parameter("classify_max_length", Parameter.Type.INTEGER, 100),
            )
            .get_parameter_value()
            .integer_value
        )
        self.classify_index_range = (
            self.get_parameter_or(
                "classify_index_range",
                Parameter("classify_index_range", Parameter.Type.DOUBLE, 0.3),
            )
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info("classify_method: %s" % (self.classify_method))
        self.get_logger().info(
            "differential_threshold: %s" % (self.differential_threshold)
        )
        self.get_logger().info("max_rho_index_range: %s" % (self.max_rho_index_range))
        self.get_logger().info("classify_min_length: %s" % (self.classify_min_length))
        self.get_logger().info("classify_max_length: %s" % (self.classify_max_length))
        self.get_logger().info("classify_index_range: %s" % (self.classify_index_range))

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_profile
        )
        self.scan_subscription  # prevent unused variable warning

        self.buoy_publisher = self.create_publisher(
            ClassificationArray, "Buoy", qos_profile
        )
        self.wall_publisher = self.create_publisher(
            ClassificationArray, "Wall", qos_profile
        )

    def listener_callback(self, data):
        self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
        self.get_logger().info("rad min: %s" % (math.degrees(data.angle_min)))
        self.get_logger().info("rad max: %s" % (math.degrees(data.angle_max)))
        self.get_logger().info("range min: %s" % (data.range_min))
        self.get_logger().info("range max: %s" % (data.range_max))

        rhos = []  # 거리(M)
        thetas = []  # 각도(radian)
        angle = data.angle_min
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)):
                rhos.append(scan_data)
                thetas.append(angle)
            angle += data.angle_increment

        # 극좌표계 적분해서 기울기로 그룹핑
        polar_coordinates_group = self.grouping_with_differential(
            rhos, thetas, self.differential_threshold
        )

        if self.classify_method == "rho":
            # rho로 분리하는 방법
            polar_coordinates_group = self.regrouping_with_max_rho(
                polar_coordinates_group, self.max_rho_index_range
            )
            buoy, wall = self.classify_with_rho(
                polar_coordinates_group,
                self.classify_min_length,
                self.classify_index_range,
            )
        elif self.classify_method == "lsm":
            # 최소차승법으로 분리하는 방법
            buoy, wall = self.classify_with_LSM(
                polar_coordinates_group,
                self.classify_min_length,
                self.classify_max_length,
            )
        else:
            return

        self.buoy_publisher.publish(buoy)
        self.wall_publisher.publish(wall)

    def grouping_with_differential(self, rhos, thetas, threshold):
        rhos_df = np.r_[0, np.diff(rhos)] / 0.1

        polar_coordinates_group = []
        polar_coordinates = []
        for rho, theta, rho_df in zip(rhos, thetas, rhos_df):
            if abs(rho_df) > threshold:
                polar_coordinates_group.append(polar_coordinates)
                polar_coordinates = []

            polar_coordinates.append((rho, theta))

        return polar_coordinates_group

    def regrouping_with_max_rho(self, polar_coordinates_group, index_range=0):
        # index_range : 0~1
        polar_coordinates_regroup = []
        for polar_coordinates in polar_coordinates_group[2:]:
            rhos = list(map(lambda x: x[0], polar_coordinates))
            max_rho_index = rhos.index(max(rhos))

            if index_range == 0:
                if max_rho_index != 0 and max_rho_index != len(polar_coordinates) - 1:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * index_range
                max_index = (len(polar_coordinates) - 1) * (1 - index_range)
                if max_rho_index > min_index and max_rho_index < max_index:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

        return polar_coordinates_regroup

    def classify_with_rho(self, polar_coordinates_group, min_length=0, index_range=0):
        # index_range : 0~1
        buoy = Classification()
        wall = Classification()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length:
                continue

            rhos = list(map(lambda x: x[0], polar_coordinates))
            min_rho_index = rhos.index(min(rhos))
            if min_rho_index == 0 or min_rho_index == length - 1:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            left_max_rho = max(rhos[:min_rho_index])
            right_max_rho = max(rhos[min_rho_index:])
            if abs(left_max_rho - right_max_rho) > 0.07:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            if index_range == 0:
                if min_rho_index == int(len(polar_coordinates) * 0.5 - 1):
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * (0.5 - index_range)
                max_index = (len(polar_coordinates) - 1) * (0.5 + index_range)
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range >= 1:
                min_index = len(polar_coordinates) * 0.5 - 1 - index_range
                max_index = len(polar_coordinates) * 0.5 - 1 + index_range
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

        return buoy, wall

    def classify_with_LSM(self, polar_coordinates_group, min_length=0, max_length=100):
        buoy = ClassificationArray()
        wall = ClassificationArray()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length or length > max_length:
                continue

            coords = Classification()

            coords.ranges = list(map(lambda x: x[0], polar_coordinates))
            coords.thetas = list(map(lambda x: x[1], polar_coordinates))

            # 최소 차승법
            fx = Polynomial.fit(coords.thetas, coords.ranges, deg=2).convert()
            axis = fx.coef[1] / (-2.0 * fx.coef[2])
            self.get_logger().info("fx: %s" % (str(fx)))
            self.get_logger().info("axis: %s" % (str(axis)))

            center_theta = coords.thetas[int(len(coords.thetas) / 2.0)]
            if fx.coef[2] > 0 and center_theta * 0.9 < axis < center_theta * 1.1:
                buoy.classifications.append(coords)
            else:
                wall.classifications.append(coords)

        return buoy, wall


def main(args=None):
    rclpy.init(args=args)
    node = MechashipClassify()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()