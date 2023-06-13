#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h> 
#include <std_msgs/msg/header.hpp>
#include "rclcpp/rclcpp.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>

using namespace std;
using namespace std::chrono_literals;


rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_visualization;

void vision_callback(const parking_interface::msg::Parking::SharedPtr fused_msg){
    geometry_msgs::msg::PolygonStamped vision_msg;
    geometry_msgs::msg::Point32 point;
    for(int i = 0; i < fused_msg->parking.size(); i++){
        for(int j = 0; j < 4; j++){
            if(j == 0){
                point.x = fused_msg->parking.at(i).x1;
                point.y = fused_msg->parking.at(i).y1;
                point.z = 0;
                vision_msg.polygon.points.push_back(point);
            }
            else if(j == 1){
                point.x = fused_msg->parking.at(i).x2;
                point.y = fused_msg->parking.at(i).y2;
                point.z = 0;
                vision_msg.polygon.points.push_back(point);
            }
            else if(j == 2){
                point.x = fused_msg->parking.at(i).x4;
                point.y = fused_msg->parking.at(i).y4;
                point.z = 0;
                vision_msg.polygon.points.push_back(point);
            }
            else{
                point.x = fused_msg->parking.at(i).x3;
                point.y = fused_msg->parking.at(i).y3;
                point.z = 0;
                vision_msg.polygon.points.push_back(point);
            }
        }
        
    }
    vision_msg.header.stamp = fused_msg->header.stamp;
    vision_msg.header.frame_id = "my_frame";
    pub_visualization -> publish(vision_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("visualization_pub");
    auto sub_fused= n->create_subscription<parking_interface::msg::Parking>("fused_parking", 100,  vision_callback);
    pub_visualization =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("vision_pub", 100);
    rclcpp::spin(n);
    return 0;
}