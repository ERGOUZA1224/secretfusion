#include "ros/ros.h"
#include "std_msgs/String.h"
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"

using namespace std;
using namespace std::chrono_literals;

void LoadData(parking_interface::msg::Parking &parking)
{
    parking_interface::msg::Parkinglst lst1;
    lst1.classID = 1;
    lst1.confidence = 0.8;
    lst1.x1 = 201;
    lst1.y1 = 53;
    lst1.x2 = 269;
    lst1.y2 = 52;
    lst1.x3 = 201;
    lst1.y3 = 243;
    lst1.x4 = 264;
    lst1.y4 = 244;
    parking.parkinglst.push_back(lst1);
    parking.header.stamp = std::chrono::system_clock::now();
}
 
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("Datapublisher");
    rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_img_parking;
	pub_img_parking = n->create_publisher<parking_interface::msg::Parking>("image_parking", 100);
    parking_interface::msg::Parking msg;
    LoadData(msg);
    pub_img_parking->publish(msg);
    rclcpp::spin(n);
    return 0;
}
