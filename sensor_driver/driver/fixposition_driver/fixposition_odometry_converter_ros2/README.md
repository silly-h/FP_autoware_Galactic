
# Fixposition Odometry Converter

## Description

An extra node is provided to help with the integration of the wheel odometry on your vehicle. This node is intended to be used as a middleware if you already have a topic with the wheel odometry values running on your system. At the moment, messages of the type `geometry_msgs/Twist`, `geometry_msgs/TwistWithCov` and `nav_msgs/Odometry` are accepted. To select one of these, modify the `topic_type` in the `odom_converter.yaml` file. You can choose to send just the x component of this velocity, both the x and y components, or the entire velocity. If sending more than one component (and intending for it to be used in the fusion engine), make sure that the wheelspeed sensor's configuration has the `Dimensions` field correctly configured.
The speed values from the selected topic are extracted, converted, and republished to  the `/fixposition/speed` topic, where they will be consumed by the main ROS driver node, and sent to the VRTK2.

_Please note that currently, the odometry converter only works for situations where the desired input odometry has just one value, the total vehicle speed or the total vehicle speed, configured to be RC. For situations where more sensor inputs are desired, a custom converter is necessary._

## Input Parameters

The `odom_converter.yaml` file exposes the necessary parameters for the correct operation of the node. The parameter that may cause the most doubt is the `multiplicative_factor`. This should be chosen such that the inputed float velocity value is transformed into milimeters per second, e.g. 1000 for an input that is expressed in meters per second.

## Launch

After the configuration is set, to launch the node simply run:

   `ros2 launch fixposition_odometry_converter_ros2 odom_converter.launch`

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
