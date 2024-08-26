import rclpy
import math
import time
from builtin_interfaces.msg import Time
from rclpy.node import Node
from scout_msgs.msg import ScoutStatus
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, VelocityReport
from geometry_msgs.msg import Twist

# 弧度转角度系数
radian2angle = 57.29577951308232

class ScoutStatusSubscriber(Node):

    def __init__(self):
        super().__init__('scout_status_subscriber')
        self.subscription = self.create_subscription(
            ScoutStatus,
            '/scout_status',
            self.listener_callback,
            10)
        self.subscription

        # 创建发布器
        self.publisher_data = self.create_publisher(Twist, 'twist_cmd_feedback', 10)
        self.publisher_control_mode = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.publisher_gear = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.publisher_steering = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.publisher_velocity = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)

        # 定义变量来存储消息中的数据
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.vehicle_state = 0
        self.control_mode = 0
        self.error_code = 0
        self.battery_voltage = 0.0
        self.actuator_states = []
        self.light_control_enabled = False
        self.front_light_state = 0
        self.rear_light_state = 0

    def listener_callback(self, msg):
        # 更新变量
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        self.vehicle_state = msg.vehicle_state
        self.control_mode = msg.control_mode
        self.error_code = msg.error_code
        self.battery_voltage = msg.battery_voltage
        self.actuator_states = msg.actuator_states
        self.light_control_enabled = msg.light_control_enabled
        self.front_light_state = msg.front_light_state
        self.rear_light_state = msg.rear_light_state

         # 计算角速度
        feedback_twist_angular_z = math.tan( self.angular_velocity / radian2angle) * self.linear_velocity / 0.451

        # 发布消息
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = feedback_twist_angular_z
        self.publisher_data.publish(twist)

        control_mode = ControlModeReport()
        control_mode.mode = 1  # 你需要根据实际情况设置这个值
        self.publisher_control_mode.publish(control_mode)

        gear = GearReport()
        gear.report = 2  # 你需要根据实际情况设置这个值
        self.publisher_gear.publish(gear)

        steering = SteeringReport()
        steering.steering_tire_angle = -(self.angular_velocity / radian2angle)  # 你需要根据实际情况计算这个值
        self.publisher_steering.publish(steering)

        velocity = VelocityReport()
        sec_ = int(time.time())
        # 纳秒
        nanosec_ = int((time.time()-sec_)*1e9)
        velocity.header.stamp = Time(sec = sec_,nanosec = nanosec_)
        velocity.header.frame_id = "base_link"
        velocity.longitudinal_velocity = self.linear_velocity
        velocity.lateral_velocity = 0.0
        velocity.heading_rate = 0.0
    
        self.publisher_velocity.publish(velocity)

def main(args=None):
    rclpy.init(args=args)

    scout_status_subscriber = ScoutStatusSubscriber()

    rclpy.spin(scout_status_subscriber)

    scout_status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
