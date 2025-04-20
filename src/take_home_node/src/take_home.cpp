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

    wheelSpeedSubscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos_profile,
        std::bind(&TakeHome::speed_callback, this, std::placeholders::_1));

    wheelSteerSubscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "/raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steer_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    lap_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);

    rr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    rl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    fr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    fl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);


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
float v_x = 0;
float v_y = 0;
float omega = 0;

void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc

  v_x = odom_msg->twist.twist.linear.x;
  v_y = odom_msg->twist.twist.linear.y;

  omega = odom_msg->twist.twist.angular.z;

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

// In radians.
float front_wheel_angle = 0;
void TakeHome::steer_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg) {
    float raw_angle = steer_msg->primary_steering_angle_fbk;
    float front_wheel_angle_degrees = raw_angle / 15.0f;
    front_wheel_angle = front_wheel_angle_degrees * (M_PI / 180.0f);
}

const float_t w_f = 1.638;
const float_t w_r = 1.523;
const float_t l_f = 1.7238;

void TakeHome::speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr speed_msg) {

    float kmph_to_ms = 1000.0f/3600.0f;

    float speed_rr = speed_msg->rear_right * kmph_to_ms;
    float speed_rl = speed_msg->rear_left * kmph_to_ms;
    float speed_fr = speed_msg->front_right * kmph_to_ms;
    float speed_fl = speed_msg->front_left * kmph_to_ms;

    float v_xr = v_x - (0.5f * omega * w_r);
    float slip_rr = (speed_rr - v_xr) / v_xr;
    float slip_rl = (speed_rl - v_xr) / v_xr;

    float v_xf = v_x - 0.5f * omega * w_f;
    float v_yf = v_y + omega * l_f;
    float v_steer_x = cos(front_wheel_angle) * v_xf - sin(front_wheel_angle) * v_yf;
    float slip_fr = (speed_fr - v_steer_x) / v_steer_x;
    float slip_fl = (speed_fl - v_steer_x) / v_steer_x;

    std_msgs::msg::Float32 rr_msg, rl_msg, fr_msg, fl_msg;
    rr_msg.data = slip_rr;
    rl_msg.data = slip_rl;
    fr_msg.data = slip_fr;
    fl_msg.data = slip_fl;

    rr_publisher->publish(rr_msg);
    rl_publisher->publish(rl_msg);
    fr_publisher->publish(fr_msg);
    fl_publisher->publish(fl_msg);
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
