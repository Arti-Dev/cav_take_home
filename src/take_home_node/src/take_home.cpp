#include "take_home_node/take_home.hpp"

#include <queue>
#include <rclcpp_components/register_node_macro.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto reliable_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    rawIMU_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", reliable_qos_profile,
        std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

    lap_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/curvilinear_distance", reliable_qos_profile,
        std::bind(&TakeHome::lap_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    lap_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);


}

double_t convertToDouble(const rclcpp::Time& time) {
    return time.seconds() + pow(static_cast<double>(time.nanoseconds()), -9);
}

double_t currentTime = 0;

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

  rclcpp::Time timestamp = odom_msg->header.stamp;
  currentTime = convertToDouble(timestamp);
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
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

int lap = 0;
double_t lastFinishLineTime = 0;
float_t lastDistance;
void TakeHome::lap_callback(std_msgs::msg::Float32::ConstSharedPtr lap_msg) {
    // assume that when lap_msg is 0 it means the car has crossed the finish line
    // in reality, not sure if this will always happen (due to inconsistencies, maybe it reports -1 or 1)
    // but the bag seems to be fine

    float_t dist = lap_msg->data;
    if (lap == 0 && dist == 0) {
        lastFinishLineTime = currentTime;
        lap = 1;
    } else if (lap == 1 && dist == 0 && lastDistance > 0) {
        lastFinishLineTime = currentTime;
    }

    double_t lapTime = currentTime - lastFinishLineTime;
    if (lap == 0) lapTime = 0;
    std_msgs::msg::Float32 time_msg;
    time_msg.data = lapTime;
    lap_publisher_->publish(time_msg);

    lastDistance = dist;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
