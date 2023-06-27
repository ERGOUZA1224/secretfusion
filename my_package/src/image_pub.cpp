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
      20ms, std::bind(&MinimalPublisher::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      parking_interface::msg::Parking message;
      message.header.stamp = this->get_clock()->now();
      parking_interface::msg::Parkinglst lst1;
      parking_interface::msg::Parkinglst lst2;
      parking_interface::msg::Parkinglst lst3;
      parking_interface::msg::Parkinglst lst4;
      parking_interface::msg::Parkinglst lst5; 
      //N = rand() % 10 + 1;
      N = 0;
      lst1.confidence = 0.85;
      lst1.x1 = 148;
      lst1.y1 = 53 + N;
      lst1.x2 = 216 ;
      lst1.y2 = 53 + N;
      lst1.x3 = 148;
      lst1.y3 = 243 + N;
      lst1.x4 = 216;
      lst1.y4 = 243 + N;
      message.parking.push_back(lst1);
      lst2.confidence = 0.9;
      lst2.x1 = 0 ;
      lst2.y1 = 53 + N;
      lst2.x2 = 68 ;
      lst2.y2 = 53 + N;
      lst2.x3 = 0 ;
      lst2.y3 = 244 + N;
      lst2.x4 = 68 ;
      lst2.y4 = 244 + N;
      message.parking.push_back(lst2);
      lst3.confidence = 0.86;
      lst3.x1 = 220 ;
      lst3.y1 = 55 + N;
      lst3.x2 = 289 ;
      lst3.y2 = 55 + N;
      lst3.x3 = 220 ;
      lst3.y3 = 240 + N;
      lst3.x4 = 289 ;
      lst3.y4 = 240+ N;
      message.parking.push_back(lst3);
      lst4.confidence = 0.82;
      lst4.x1 = 75;
      lst4.y1 = 54 + N;
      lst4.x2 = 143 ;
      lst4.y2 = 54 + N;
      lst4.x3 = 75 ;
      lst4.y3 = 242 + N;
      lst4.x4 = 143 ;
      lst4.y4 = 242+ N;
      message.parking.push_back(lst4);
      lst5.confidence = 0.88;
      lst5.x1 = 294;
      lst5.y1 = 54 + N;
      lst5.x2 = 362 ;
      lst5.y2 = 54 + N;
      lst5.x3 = 294 ;
      lst5.y3 = 242 + N;
      lst5.x4 = 362 ;
      lst5.y4 = 242+ N;
      message.parking.push_back(lst5);
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