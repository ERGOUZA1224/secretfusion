#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include <std_msgs/msg/header.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <parking_interface/msg/parking.hpp>
#include <parking_interface/msg/parkinglist.hpp>
#include <parking_interface/msg/point2_d.hpp>
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
      parking_interface::msg::Parkinglist lst1;
      N = rand() % 5+ 1;
      //N = 0;
      parking_interface::msg::Parkinglist lst2;
      parking_interface::msg::Parkinglist lst3;
      lst1.id = -1;
      parking_interface::msg::Point2D pt1;
      pt1.x = 0;
      pt1.y = 53 + N;
      lst1.pointlist.push_back(pt1);
      parking_interface::msg::Point2D pt2;
      pt2.x = 0;
      pt2.y = 244 + N;
      lst1.pointlist.push_back(pt2);
      parking_interface::msg::Point2D pt3;
      pt3.x = 68;
      pt3.y = 244 + N;
      lst1.pointlist.push_back(pt3);
      parking_interface::msg::Point2D pt4;
      pt4.x = 68;
      pt4.y = 253 + N;
      lst1.pointlist.push_back(pt4);
      message.parkinglist.push_back(lst1);
      // lst1.pointlist[0].x = 0 ;
      // lst1.pointlist[0].y = 53 + N;      
      // lst1.pointlist[3].x = 150 ;
      // lst1.pointlist[3].y = 53 + N;
      // lst1.pointlist[1].x = 0 ;
      // lst1.pointlist[1].y = 250 + N;
      // lst1.pointlist[2].x = 150 ;
      // lst1.pointlist[2].x = 250 + N;
      // message.parkinglist.push_back(lst1);

      // lst2.pointlist[0].x = 214 ;
      // lst2.pointlist[0].y = 53 + N;      
      // lst2.pointlist[3].x = 290 ;
      // lst2.pointlist[3].y = 53 + N;
      // lst2.pointlist[1].x = 214 ;
      // lst2.pointlist[1].y = 251 + N;
      // lst2.pointlist[2].x = 290 ;
      // lst2.pointlist[2].x = 251 + N;
      // message.parkinglist.push_back(lst2);

      // lst3.pointlist[0].x = 290 ;
      // lst3.pointlist[0].y = 167 + N;      
      // lst3.pointlist[3].x = 358 ;
      // lst3.pointlist[3].y = 167 + N;
      // lst3.pointlist[1].x = 290 ;
      // lst3.pointlist[1].y = 249 + N;
      // lst3.pointlist[2].x = 358 ;
      // lst3.pointlist[2].x = 249 + N;
      // message.parkinglist.push_back(lst3);
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