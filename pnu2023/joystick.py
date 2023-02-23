import os, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from mechaship_interfaces.srv import ThrottlePercentage, Key, RGBColor


def map(x, input_min, input_max, output_min, output_max):
    res = (x - input_min) * (output_max - output_min) / (
        input_max - input_min
    ) + output_min
    return output_min if res < output_min else res


class MechashipTeleopJoystick(Node):

    # 서보모터 최소, 최대
    KEY_MIN = 45
    KEY_MAX = 135

    def __init__(self):
        super().__init__("mechaship_teleop_joystick")
        self.get_logger().info("mechaship_teleop_joystick Start")

        qos_profile = QoSProfile(depth=10)

        self._data_joy = Joy()
        self._is_data = False

        self.create_subscription(Joy, "joy", self.joy_pub_callback, qos_profile)
        self.emo = self.create_subscription(
            Bool, "/emo_event", self.joy_pub_callback, qos_profile
        )
        self.throttle_client = self.create_client(
            ThrottlePercentage, "actuators/throttle/set_percentage"
        )
        self.key_client = self.create_client(Key, "actuators/key/set")
        self.rgb_client = self.create_client(RGBColor, "rgbled/set")
        self.throttle_input = 0
        self.create_timer(0.1, self.set_mcu)

    def joy_pub_callback(self, data):
        self._data_joy = data
        self._is_data = True

            
    def set_mcu(self):

        if self._is_data == False:
            return
        # 위: 양수, 좌: 양수
        stick_left_vertical = self._data_joy.axes[1]
        stick_left_horizontal = self._data_joy.axes[0]

        stick_right_vertical = self._data_joy.axes[2]
        stick_right_horizontal = self._data_joy.axes[3]

        button_1 = self._data_joy.buttons[0]
        button_2 = self._data_joy.buttons[1]
        button_3 = self._data_joy.buttons[2]
        button_4 = self._data_joy.buttons[3]

        throttle_data = ThrottlePercentage.Request()
        key_data = Key.Request()
        if stick_left_vertical == 1:
            if self.throttle_input < 100 :
                self.throttle_input += 5
        elif stick_left_vertical == -1:
            if self.throttle_input > 0 :
                self.throttle_input -= 5
        
        if self._data_joy.buttons[6] == 1 and self._data_joy.buttons[7] == 1:
            self.throttle_input = 0
            
        throttle_data.percentage = self.throttle_input
        
        key_data.degree = int(
            map(stick_left_horizontal, -1, 1, self.KEY_MIN, self.KEY_MAX)
        )

        color_data = RGBColor.Request()
        change_color = button_1 or button_2 or button_3 or button_4
        if button_1 == 1:
            color_data.red = 0
            color_data.green = 254
            color_data.blue = 0
        elif button_2 == 1:
            color_data.red = 254
            color_data.green = 0
            color_data.blue = 0
        elif button_3 == 1:
            color_data.red = 0
            color_data.green = 0
            color_data.blue = 254
        elif button_4 == 1:
            color_data.red = 255
            color_data.green = 255
            color_data.blue = 255

        self.throttle_client.call_async(throttle_data)
        self.key_client.call_async(key_data)
        if change_color == True:
            self.rgb_client.call_async(color_data)


def main(args=None):
    rclpy.init(args=args)

    mechaship_teleop_joystick = MechashipTeleopJoystick()

    rclpy.spin(mechaship_teleop_joystick)
    mechaship_teleop_joystick.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



