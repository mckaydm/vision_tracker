#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode();

private:
  void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr sub_;

  // State: x, y, vx, vy
  std::array<double, 4> xA_;
  std::array<double, 4> xA_hat_;
  std::array<double, 2> prev_pos_;

  // Parameters
  double Kd_, Kp_, mass_, gravity_, dt_;
  bool first_measurement_;
  rclcpp::Time last_measurement_time_;
};
