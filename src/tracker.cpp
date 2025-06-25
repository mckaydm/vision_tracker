#include <memory>            
#include "rclcpp/rclcpp.hpp"
#include "vision_tracker/tracker.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

TrackerNode::TrackerNode()
: Node("tracker_node"),
  xA_{0.0, 0.0, 0.0, 0.0},
  xA_hat_{0.0, 0.0, 0.0, 0.0},
  prev_pos_{0.0, 0.0},
  gravity_(9.81),
  dt_(1.0),
  first_measurement_(true),
  last_measurement_time_(this->now())
{
  // Declare parameters with default values
  this->declare_parameter("mass", 5.0);
  this->get_parameter("mass", mass_);
  this->declare_parameter("Kp", 0.05);
  this->get_parameter("Kp", Kp_);
  this->declare_parameter("Kd", 0.05);
  this->get_parameter("Kd", Kd_);


  sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
    "/yolo/tracking", 10, std::bind(&TrackerNode::detection_callback, this, _1));
}

void TrackerNode::detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
  if (msg->detections.empty()) return;

  auto now = this->now();
  const auto& detection = msg->detections[0];
  double err_x = detection.bbox.center.position.x;
  double err_y = detection.bbox.center.position.y;

  // Intermittent measurement update
  xA_hat_[0] = err_x;
  xA_hat_[1] = err_y;

  // Compute dt from prev detection
  double dt = dt_; // default
  if (!first_measurement_) {
    dt = (now - last_measurement_time_).seconds();
  }
  last_measurement_time_ = now;
  first_measurement_ = false;

  // Velocity estimate
  xA_hat_[2] = (xA_hat_[0] - prev_pos_[0]) / dt;
  xA_hat_[3] = (xA_hat_[1] - prev_pos_[1]) / dt;
  prev_pos_ = {xA_hat_[0], xA_hat_[1]};

  // PD Control
  double ux = -Kp_ * xA_hat_[0] - Kd_ * xA_hat_[2];
  double uy = -Kp_ * xA_hat_[1] - Kd_ * xA_hat_[3] + mass_ * gravity_;

  // Dynamics update
  double ax = ux / mass_;
  double ay = uy / mass_ - gravity_;
  xA_[2] += ax * dt_;
  xA_[3] += ay * dt_;
  xA_[0] += xA_[2] * dt_;
  xA_[1] += xA_[3] * dt_;

  RCLCPP_INFO(this->get_logger(), "Input Forces: [%.2f, %.2f], Follower Position: [%.2f, %.2f]", ux, uy, xA_[0], xA_[1]);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                             // Initialize ROS 2
  auto node = std::make_shared<TrackerNode>();          // Create node instance
  rclcpp::spin(node);                                   // Start spinning (event loop)
  rclcpp::shutdown();                                   // Clean shutdown when done
  return 0;
}