#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include <std_msgs/msg/header.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <parking_interface/msg/parking.hpp>
#include <parking_interface/msg/parkinglst.hpp>
using namespace std;
using namespace std::chrono_literals;
int N;
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
      20ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      parking_interface::msg::Parking message;
      message.header.stamp = this->get_clock()->now();
      parking_interface::msg::Parkinglst lst1;
      N = rand() % 5+ 1;
      //N = 0;
      parking_interface::msg::Parkinglst lst2;
      lst1.confidence = 1;
      lst1.x1 = 0;
      lst1.y1 = 53 + N;
      lst1.x2 = 150 ;
      lst1.y2 = 53 + N;
      lst1.x3 = 0 ;
      lst1.y3 = 250 + N;
      lst1.x4 = 150 ;
      lst1.y4 = 250 + N; 
      message.parking.push_back(lst1);

      lst2.confidence = 1;
      lst2.x1 = 214 ;
      lst2.y1 = 53 + N;
      lst2.x2 = 290 ;
      lst2.y2 = 53 + N;
      lst2.x3 = 214 ;
      lst2.y3 = 251 + N;
      lst2.x4 = 290 ;
      lst2.y4 = 251 + N; 
      message.parking.push_back(lst2);
      
      parking_interface::msg::Parkinglst lst3;
      lst3.confidence = 1;
      lst3.x1 = 290;
      lst3.y1 = 167 + N;
      lst3.x2 = 358 ;
      lst3.y2 = 167 + N;
      lst3.x3 = 290;
      lst3.y3 = 249 + N;
      lst3.x4 = 358 ;
      lst3.y4 = 249 + N;
      message.parking.push_back(lst3);
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