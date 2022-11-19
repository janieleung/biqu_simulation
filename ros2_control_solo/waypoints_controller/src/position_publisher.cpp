#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PositionPublisher : public rclcpp::Node
{
  public:
    PositionPublisher()
    : Node("position_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_controller/commands", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&PositionPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();
      double FL_HAA = 0.0;
      double FL_HFE = 0.0;
      double FL_KFE = 0.0;

      double FR_HAA = 0.0;
      double FR_HFE = 0.0;
      double FR_KFE = 0.0;

      double HL_HAA = 0.0;
      double HL_HFE = 0.0;
      double HL_KFE = 0.0;

      double HR_HAA = 0.0;
      double HR_HFE = 0.0;
      double HR_KFE = 0.0;

      message.data.push_back(FL_HAA);
      message.data.push_back(FL_HFE);
      message.data.push_back(FL_KFE);

      message.data.push_back(FR_HAA);
      message.data.push_back(FR_HFE);
      message.data.push_back(FR_KFE);

      message.data.push_back(HL_HAA);
      message.data.push_back(HL_HFE);
      message.data.push_back(HL_KFE);

      message.data.push_back(HR_HAA);
      message.data.push_back(HR_HFE);
      message.data.push_back(HR_KFE);

      publisher_->publish(message);
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