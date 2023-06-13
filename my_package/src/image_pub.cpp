#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
using namespace std;
using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
int N;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("image_pub"), count_(0)
    {
      //this -> N = rand() % 10 + 1; 
      publisher_ = this->create_publisher<parking_interface::msg::Parking>("image_parking", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      parking_interface::msg::Parking message;
      message.header.stamp = this->get_clock()->now();
      parking_interface::msg::Parkinglst lst1;
      N = rand() % 10 + 1;
      lst1.confidence = 0.85;
      lst1.x1 = 201 + N;
      lst1.y1 = 53 + N; 
      lst1.x2 = 269 + N;
      lst1.y2 = 53 + N;
      lst1.x3 = 201 + N;
      lst1.y3 = 243 + N;
      lst1.x4 = 264 + N;
      lst1.y4 = 244 + N; 
      message.parking.push_back(lst1);
      RCLCPP_INFO(this->get_logger(), "image data publishing...");
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