#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <std_msgs/msg/header.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <parking_interface/msg/parking.hpp>
#include <parking_interface/msg/parkinglst.hpp>
using namespace std;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("radar_pub"), count_(0)
    {
      publisher_ = this->create_publisher<parking_interface::msg::Parking>("radar_parking", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      parking_interface::msg::Parking message;
      message.header.stamp = this->get_clock()->now();
      parking_interface::msg::Parkinglst lst1;
      lst1.confidence = 1;
      lst1.x1 = 188;
      lst1.y1 = 14;
      lst1.x2 = 268;
      lst1.y2 = 14;
      lst1.x3 = 188;
      lst1.y3 = 284;
      lst1.x4 = 268;
      lst1.y4 = 284; 
      message.parking.push_back(lst1);
      parking_interface::msg::Parkinglst lst2;
      lst2.confidence = 1;
      lst2.x1 = 1;
      lst2.y1 = 1;
      lst2.x2 = 99;
      lst2.y2 = 1;
      lst2.x3 = 1;
      lst2.y3 = 291;
      lst2.x4 = 99;
      lst2.y4 = 291; 
      message.parking.push_back(lst2);
      RCLCPP_INFO(this->get_logger(), "radar data publishing...");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}