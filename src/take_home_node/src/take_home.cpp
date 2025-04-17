#include "take_home_node/take_home.hpp"

#include <queue>
#include <rclcpp_components/register_node_macro.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto imu_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    rawIMU_subscriber = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", imu_qos_profile,
        std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);


}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
}

double_t convertToDouble(const rclcpp::Time& time) {
    return time.seconds() + pow(static_cast<double>(time.nanoseconds()), -9);
}

std::deque<double_t> timestampQueue;

void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
    // use the header, not gnss_seconds to get jitter
    rclcpp::Time timestamp = imu_msg->header.stamp;
    double_t time = convertToDouble(timestamp);

    timestampQueue.push_back(time);
    while (time - timestampQueue.front() > 1) {
        timestampQueue.pop_front();
    }

    std::list<double_t> delta_ts;
    for (long unsigned int i = 1; i < timestampQueue.size(); i++) {
        delta_ts.push_back(timestampQueue[i] - timestampQueue[i-1]);
    }
    // variance calculation
    double_t squareX = 0; // E[X^2]
    double_t squareExpected = 0; // (E[X])^2

    for (double_t delta : delta_ts) {
        squareX += pow(delta, 2);
        squareExpected += delta;
    }
    squareExpected /= delta_ts.size();
    squareX /= delta_ts.size();
    squareExpected = pow(squareExpected, 2);

    auto jitter = static_cast<float_t>(squareX - squareExpected);

    std_msgs::msg::Float32 time_msg;
    time_msg.data = jitter;
    jitter_publisher_->publish(time_msg);
}

// void TakeHome::lap_callback() {
//
// }


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
