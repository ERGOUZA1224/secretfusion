#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include <std_msgs/msg/header.hpp>
#include "rclcpp/rclcpp.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglist.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

visualization_msgs::msg::Marker markerpoint;

//Node to visualize fused parking slot data on RViz by using Marker
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("vision_pub"), count_(0)
    {
      //this -> N = rand() % 10 + 1; 
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("fused_pub1", 100);
      timer_ = this->create_wall_timer(
      20ms, std::bind(&MinimalPublisher::timer_callback, this));
      sub_fused = this->create_subscription<parking_interface::msg::Parking>(
      "fused_parking", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    }

  private:
    void topic_callback(const parking_interface::msg::Parking::SharedPtr fused_msg)
    {
    
      markerpoint.header.stamp = fused_msg->header.stamp;
      markerpoint.header.frame_id = "map";
      markerpoint.type = visualization_msgs::msg::Marker::POINTS;
      markerpoint.action = visualization_msgs::msg::Marker::ADD;
      markerpoint.id = 0;
      markerpoint.scale.x = 8;
      markerpoint.scale.y = 8;
      markerpoint.scale.z = 8;//大小
      markerpoint.color.g = 1.0f;
      markerpoint.color.a = 1;//必写，否则rviz无法显示
      geometry_msgs::msg::PolygonStamped vision_msg;
      geometry_msgs::msg::Point point;
      for(int i = 0; i < fused_msg->parkinglist.size(); i++){
          for(int j = 0; j < 4; j++){
              point.x = fused_msg->parkinglist[i].pointlist[j].x;
              point.y = fused_msg->parkinglist[i].pointlist[j].y;
              point.z = 0;
              markerpoint.points.push_back(point);
          }
      }
      cout<<"point num: "<<markerpoint.points.size()<<endl;
      publisher_->publish(markerpoint);
      markerpoint.points.clear();

    }

    void timer_callback(){
        
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Subscription<parking_interface::msg::Parking>::SharedPtr sub_fused;
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}