#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>


#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

  void lap_callback(std_msgs::msg::Float32::ConstSharedPtr lap_msg);

  void speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr speed_msg);

  void steer_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr rawIMU_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheelSpeedSubscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr wheelSteerSubscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fl_publisher;
};
