#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglist.hpp"
#include "parking_interface/msg/point2_d.hpp"
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
      parking_interface::msg::Parkinglist lst1;
      parking_interface::msg::Parkinglist lst2;
      parking_interface::msg::Parkinglist lst3;
      parking_interface::msg::Parkinglist lst4;
      parking_interface::msg::Parkinglist lst5; 
      
      N = rand() % 5 + 1;
      // cout<<"begin offer slot value"<<endl;
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
       
      // cout<<"111"<<endl;    
      // lst1.pointlist[3].x = 68 ;
      // lst1.pointlist[3].y = 53 + N;
      // cout<<"222"<<endl;
      // lst1.pointlist[1].x = 0 ;
      // lst1.pointlist[1].y = 244 + N;
      // lst1.pointlist[2].x = 68 ;
      // lst1.pointlist[2].x = 244 + N;
      message.parkinglist.push_back(lst1);
      // cout<<"begin offer slot value2"<<endl;
      // lst2.pointlist[0].x = 75 ;
      // lst2.pointlist[0].y = 54 + N;      
      // lst2.pointlist[3].x = 143 ;
      // lst2.pointlist[3].y = 54 + N;
      // lst2.pointlist[1].x = 148 ;
      // lst2.pointlist[1].y = 243 + N;
      // lst2.pointlist[2].x = 216 ;
      // lst2.pointlist[2].x = 243 + N;
      // message.parkinglist.push_back(lst2);

      // lst3.pointlist[0].x = 148 ;
      // lst3.pointlist[0].y = 53 + N;      
      // lst3.pointlist[3].x = 216 ;
      // lst3.pointlist[3].y = 53 + N;
      // lst3.pointlist[1].x = 148 ;
      // lst3.pointlist[1].y = 243 + N;
      // lst3.pointlist[2].x = 216 ;
      // lst3.pointlist[2].x = 243 + N;
      // message.parkinglist.push_back(lst3);

      // lst4.pointlist[0].x = 220 ;
      // lst4.pointlist[0].y = 55 + N;      
      // lst4.pointlist[3].x = 289 ;
      // lst4.pointlist[3].y = 55 + N;
      // lst4.pointlist[1].x = 220 ;
      // lst4.pointlist[1].y = 240 + N;
      // lst4.pointlist[2].x = 289 ;
      // lst4.pointlist[2].x = 240 + N;
      // message.parkinglist.push_back(lst4);

      // lst5.pointlist[0].x = 294 ;
      // lst5.pointlist[0].y = 54 + N;      
      // lst5.pointlist[3].x = 362 ;
      // lst5.pointlist[3].y = 54 + N;
      // lst5.pointlist[1].x = 294;
      // lst5.pointlist[1].y = 242 + N;
      // lst5.pointlist[2].x = 362 ;
      // lst5.pointlist[2].x = 242 + N;
      // message.parkinglist.push_back(lst5);
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