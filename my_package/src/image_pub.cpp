#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "parking_interface/msg/parking.hpp"
// #include "parking_interface/msg/parkinglist.hpp"
// #include "parking_interface/msg/point2_d.hpp"
#include "parkingslot_interfaces/msg/parking_slot_array.hpp"
#include "parkingslot_interfaces/msg/corner_detection_out.hpp"
#include "parkingslot_interfaces/msg/point2_d.hpp"
using namespace std;
using namespace std::chrono_literals;

int N;

// Node for publish image data
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("image_pub"), count_(0)
    {
      //this -> N = rand() % 10 + 1; 
      publisher_ = this->create_publisher<parkingslot_interfaces::msg::ParkingSlotArray>("image_parking", 10);
      timer_ = this->create_wall_timer(
      20ms, std::bind(&MinimalPublisher::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      parkingslot_interfaces::msg::ParkingSlotArray message;
      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "map";
      parkingslot_interfaces::msg::CornerDetectionOut lst1;
      parkingslot_interfaces::msg::CornerDetectionOut lst2;
      parkingslot_interfaces::msg::CornerDetectionOut lst3;
      parkingslot_interfaces::msg::CornerDetectionOut lst4;
      parkingslot_interfaces::msg::CornerDetectionOut lst5; 
      
      N = rand() % 10 + 1;
      // cout<<"begin offer slot value"<<endl;
      lst1.slot_id = -1;
      parkingslot_interfaces::msg::Point2D pt1;
      pt1.x = 0 + N;
      pt1.y = 53 + N;
      lst1.corner_point.push_back(pt1);
      parkingslot_interfaces::msg::Point2D pt2;
      pt2.x = 0 + N;
      pt2.y = 244 + N;
      lst1.corner_point.push_back(pt2);
      parkingslot_interfaces::msg::Point2D pt3;
      pt3.x = 68 + N;
      pt3.y = 244 + N;
      lst1.corner_point.push_back(pt3);
      parkingslot_interfaces::msg::Point2D pt4;
      pt4.x = 68 + N;
      pt4.y = 253 + N;
      lst1.corner_point.push_back(pt4);
      message.parking_detection_out_list.push_back(lst1);

      lst2.slot_id = -1;
      pt1.x = 75 + N;
      pt1.y = 54 + N;
      lst2.corner_point.push_back(pt1);
      pt2.x = 75 + N;
      pt2.y = 243 + N;
      lst2.corner_point.push_back(pt2);
      pt3.x = 143 + N;
      pt3.y = 243 + N;
      lst2.corner_point.push_back(pt3);
      pt4.x = 143 + N;
      pt4.y = 54 + N;
      lst2.corner_point.push_back(pt4);
      message.parking_detection_out_list.push_back(lst2);

      lst3.slot_id = -1;
      pt1.x = 148 + N;
      pt1.y = 53 + N;
      lst3.corner_point.push_back(pt1);
      pt2.x = 148 + N;
      pt2.y = 243 + N;
      lst3.corner_point.push_back(pt2);
      pt3.x = 216+ N;
      pt3.y = 243 + N;
      lst3.corner_point.push_back(pt3);
      pt4.x = 216+N;
      pt4.y = 53 + N;
      lst3.corner_point.push_back(pt4);
      message.parking_detection_out_list.push_back(lst3);
      
      lst4.slot_id = -1;
      pt1.x = 220;
      pt1.y = 55 + N;
      lst4.corner_point.push_back(pt1);
      pt2.x = 220;
      pt2.y = 240 + N;
      lst4.corner_point.push_back(pt2);
      pt3.x = 289;
      pt3.y = 240 + N;
      lst4.corner_point.push_back(pt3);
      pt4.x = 289;
      pt4.y = 55 + N;
      lst4.corner_point.push_back(pt4);
      message.parking_detection_out_list.push_back(lst4);
      
      lst5.slot_id = -1;
      pt1.x = 294;
      pt1.y = 54 + N;
      lst5.corner_point.push_back(pt1);
      pt2.x = 294;
      pt2.y = 242 + N;
      lst5.corner_point.push_back(pt2);
      pt3.x = 362;
      pt3.y = 242 + N;
      lst5.corner_point.push_back(pt3);
      pt4.x = 362;
      pt4.y = 54 + N;
      lst5.corner_point.push_back(pt4);
      message.parking_detection_out_list.push_back(lst5);
      
      RCLCPP_INFO(this->get_logger(), "image data publishing...");
      publisher_->publish(message);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<parkingslot_interfaces::msg::ParkingSlotArray>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}