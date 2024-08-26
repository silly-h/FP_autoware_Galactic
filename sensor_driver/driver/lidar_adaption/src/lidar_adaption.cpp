#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


double RadiusOutlierFilter = 0.0;


class LidarTransformNode : public rclcpp::Node
{
public:
    // 构造函数
    LidarTransformNode() : Node("points_raw_transform_node")
    {
        // 初始化坐标转换参数
        // this->declare_parameter<double>("transform_x", 0.0);
        // this->get_parameter("transform_x", transform_x);
        transform_x = this->declare_parameter("transform_x", 0.156300);
        transform_y = this->declare_parameter("transform_y", 0.0);
        transform_z = this->declare_parameter("transform_z", 0.710000);
        transform_roll = this->declare_parameter("transform_roll", 0.0);
        transform_pitch = this->declare_parameter("transform_pitch", 0.0);
        transform_yaw = this->declare_parameter("transform_yaw", 0.0);

        std::cout << "velodyne to base_link:" << std::endl
                  << "  transform_x:    " << transform_x << std::endl
                  << "  transform_y:    " << transform_y << std::endl
                  << "  transform_z:    " << transform_z << std::endl
                  << "  transform_roll: " << transform_roll << std::endl
                  << "  transform_pitch:" << transform_pitch << std::endl
                  << "  transform_yaw:  " << transform_yaw << std::endl;

        // Initialize translation
        transform_stamp.transform.translation.x = transform_x;
        transform_stamp.transform.translation.y = transform_y;
        transform_stamp.transform.translation.z = transform_z;
        // Initialize rotation (quaternion)
        tf2::Quaternion quaternion;
        quaternion.setRPY(transform_roll, transform_pitch, transform_yaw);
        transform_stamp.transform.rotation.x = quaternion.x();
        transform_stamp.transform.rotation.y = quaternion.y();
        transform_stamp.transform.rotation.z = quaternion.z();
        transform_stamp.transform.rotation.w = quaternion.w();

        // 创建发布者
        publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/outlier_filtered/pointcloud",
            10);
        publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/concatenated/pointcloud",
            10);

        // 订阅原始点云消息
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points",
            10,
            std::bind(&LidarTransformNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 过滤掉距离传感器较近的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *xyz_cloud);
        for (size_t i = 0; i < xyz_cloud->points.size(); ++i)
        {
          if (sqrt(xyz_cloud->points[i].x * xyz_cloud->points[i].x + xyz_cloud->points[i].y * xyz_cloud->points[i].y +
                   xyz_cloud->points[i].z * xyz_cloud->points[i].z) >= RadiusOutlierFilter && !isnan(xyz_cloud->points[i].z))
          {
            pcl_output->points.push_back(xyz_cloud->points.at(i));
          }
        }
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*pcl_output, output);
        output.header = msg->header;
    
        // 执行坐标转换
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud("base_link", transform_stamp, output, transformed_cloud);
    
        // 发布转换后的点云消息
        publisher_1->publish(transformed_cloud);
        publisher_2->publish(transformed_cloud);
    }
  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1, publisher_2;

    double transform_x, transform_y, transform_z, transform_roll, transform_pitch, transform_yaw;
    geometry_msgs::msg::TransformStamped transform_stamp;
};

int main(int argc, char **argv)
{
    // 初始化节点
    rclcpp::init(argc, argv);

    // 创建实例
    auto node = std::make_shared<LidarTransformNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
