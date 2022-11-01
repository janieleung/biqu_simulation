#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class VelocityController : public rclcpp::Node
{
  public:
    VelocityController()
    : Node("velocity_controller"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&VelocityController::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // Set joint velocity to 0 to stop robot
      auto message = std_msgs::msg::Float64MultiArray();
      //message.data.push_back(-0.6);
      //message.data.push_back(0.5);
      message.data.push_back(0);
      message.data.push_back(0);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController>());
  rclcpp::shutdown();
  return 0;
}