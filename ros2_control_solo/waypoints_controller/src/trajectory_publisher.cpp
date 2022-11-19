#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TrajectoryPublisher : public rclcpp::Node
{
  public:
    PositionPublisher()
    : Node("trajectory_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::Float64MultiArray>("/joint_group_controller/commands", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&PositionPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message_traj = trajectory_msgs::msg::JointTrajectory();
      auto message_point = trajectory_msgs::msg::JointTrajectoryPoint();

      message_point->effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
      message_point->time_from_start = 6;

      message_traj->joint_names = {"FL_HAA", "FL_HFE", "FL_KFE", "FR_HAA", "FR_HFE", "FR_KFE", "HL_HAA", "HL_HFE", "HL_KFE", "HR_HAA", "HR_HFE", "HR_KFE"};
      message_traj->points = message_point;

      publisher_->publish(message_traj);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisher>());
  rclcpp::shutdown();
  return 0;
}