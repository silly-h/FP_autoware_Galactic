import rclpy
from rclpy.node import Node
import pyproj
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatStatus

# 创建一个转换器，用于将ECEF坐标转换为WGS84坐标
transformer = pyproj.Transformer.from_crs(
    "EPSG:4978", 
    "EPSG:4326", 
    always_xy=True
)

class Converter(Node):
    def __init__(self):
        super().__init__('odometry_to_navsatfix')
        self.pub = self.create_publisher(NavSatFix, 'navsatfix', 10)
        self.create_subscription(Odometry, 'fixposition/odometry', self.callback, 10)

    def callback(self, odometry_msg):
        # 从Odometry消息中获取ECEF坐标
        x = odometry_msg.pose.pose.position.x
        y = odometry_msg.pose.pose.position.y
        z = odometry_msg.pose.pose.position.z

        # 将ECEF坐标转换为WGS84坐标
        lon, lat, alt = transformer.transform(x, y, z)

        # 创建一个新的NavSatFix消息
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = odometry_msg.header
        navsatfix_msg.latitude = lat
        navsatfix_msg.longitude = lon
        navsatfix_msg.altitude = alt

        # 设置状态信息
        navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS

        # 设置位置协方差
        navsatfix_msg.position_covariance = odometry_msg.pose.covariance
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # 发布NavSatFix消息
        self.pub.publish(navsatfix_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()