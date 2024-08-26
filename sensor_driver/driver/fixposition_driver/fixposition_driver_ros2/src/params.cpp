/**
 *  @file
 *  @brief Implementation of Parameter Loading
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* ROS */
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/params.hpp>

namespace fixposition {

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, FpOutputParams& params) {
    const std::string RATE = ns + ".rate";
    const std::string RECONNECT_DELAY = ns + ".reconnect_delay";
    const std::string TYPE = ns + ".type";
    const std::string FORMATS = ns + ".formats";
    const std::string IP = ns + ".ip";
    const std::string PORT = ns + ".port";
    const std::string BAUDRATE = ns + ".baudrate";

    node->declare_parameter(RATE, 100);
    node->declare_parameter(RECONNECT_DELAY, 5.0);
    node->declare_parameter(TYPE, "tcp");
    node->declare_parameter(FORMATS, std::vector<std::string>());
    node->declare_parameter(PORT, "21000");
    node->declare_parameter(IP, "192.168.3.199");
    node->declare_parameter(BAUDRATE, 115200);
    // read parameters
    if (node->get_parameter(RATE, params.rate)) {
        RCLCPP_INFO(node->get_logger(), "%s : %d", RATE.c_str(), params.rate);
    } else {
        RCLCPP_WARN(node->get_logger(), "Using Default %s : %d", RATE.c_str(), params.rate);
    }
    if (node->get_parameter(RECONNECT_DELAY, params.reconnect_delay)) {
        RCLCPP_INFO(node->get_logger(), "%s : %f", RECONNECT_DELAY.c_str(), params.reconnect_delay);
    } else {
        RCLCPP_WARN(node->get_logger(), "%s : %f", RECONNECT_DELAY.c_str(), params.reconnect_delay);
    }

    std::string type_str;
    node->get_parameter(TYPE, type_str);
    if (type_str == "tcp") {
        params.type = INPUT_TYPE::TCP;
    } else if (type_str == "serial") {
        params.type = INPUT_TYPE::SERIAL;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Input type has to be tcp or serial!");
        return false;
    }

    node->get_parameter(FORMATS, params.formats);
    for (size_t i = 0; i < params.formats.size(); i++) {
        RCLCPP_INFO(node->get_logger(), "%s[%ld] : %s", FORMATS.c_str(), i, params.formats.at(i).c_str());
    }

    // Get parameters: port (required)
    if (node->get_parameter(PORT, params.port)) {
        RCLCPP_INFO(node->get_logger(), "%s : %s", PORT.c_str(), params.port.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "Using Default %s : %s", PORT.c_str(), params.port.c_str());
    }

    if (params.type == INPUT_TYPE::TCP) {
        if (node->get_parameter(IP, params.ip)) {
            RCLCPP_INFO(node->get_logger(), "%s : %s", IP.c_str(), params.ip.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Using Default %s : %s", IP.c_str(), params.ip.c_str());
        }
    } else if (params.type == INPUT_TYPE::SERIAL) {
        if (node->get_parameter(BAUDRATE, params.baudrate)) {
            RCLCPP_INFO(node->get_logger(), "%s : %d", BAUDRATE.c_str(), params.baudrate);
        } else {
            RCLCPP_WARN(node->get_logger(), "Using Default %s : %d", BAUDRATE.c_str(), params.baudrate);
        }
    }

    return true;
}

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, const std::string& ns, CustomerInputParams& params) {
    const std::string SPEED_TOPIC = ns + ".speed_topic";
    node->declare_parameter(SPEED_TOPIC, "/fixposition/speed");
    node->get_parameter(SPEED_TOPIC, params.speed_topic);
    RCLCPP_INFO(node->get_logger(), "%s : %s", SPEED_TOPIC.c_str(), params.speed_topic.c_str());
    return true;
}

bool LoadParamsFromRos2(std::shared_ptr<rclcpp::Node> node, FixpositionDriverParams& params) {
    bool ok = true;

    ok &= LoadParamsFromRos2(node, "fp_output", params.fp_output);
    ok &= LoadParamsFromRos2(node, "customer_input", params.customer_input);

    return ok;
}

}  // namespace fixposition
