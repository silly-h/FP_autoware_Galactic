# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from autoware_auto_control_msgs.msg import AckermannControlCommand

# 弧度转角度系数
radian2angle = 57.29577951308232

# 前轮中心与后轮中心之间
wheel_base = 0.451

class TopicSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self.subscriber = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.sub_callback, 100)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 100)

    def sub_callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.longitudinal.speed
        
        # 获取 AckermannControlCommand 的线速度和转向角度
        # angular_velocity_z = longitudinal_velocity * tan(steer_tire_angle) / wheel_base
        linear_speed = msg.longitudinal.speed
        steering_angle_rad = msg.lateral.steering_tire_angle
        
        # 计算角速度
        angular_velocity_z = math.tan(steering_angle_rad) * linear_speed / wheel_base
        twist_msg.angular.z = angular_velocity_z
        self.publisher.publish(twist_msg)

def main():
    rclpy.init()
    node = TopicSubscriberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()