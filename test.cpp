#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include "parking_interface/msg/parking.hpp"

#define BUFFER_MAX_VALUE 100
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.8

rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused_parking;
double last_vio_t = -1;
std::deque<parking_interface::msg::Parking::SharedPtr> radDeque;
std::deque<parking_interface::msg::Parking::SharedPtr> imgDeque;
std::mutex m_buf;


void radar_callback(const parking_interface::msg::Parking::SharedPtr rad_msg)
{
    if(radDeque.size() < BUFFER_MAX_VALUE){
        radDeque.push_back(rad_msg);
    }
    else{
        radDeque.pop();
        radDeque.push_back(rad_msg);
    }
}

void image_callback(const parking_interface::msg::Parking::SharedPtr img_msg)
{
    if(imgDeque.size() < BUFFER_MAX_VALUE){
        imgDeque.push_back(img_msg);
    }
    else{
        imgDeque.pop();
        imgDeque.push_back(img_msg);
    }
}


void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::msg::MarkerArray markerArray_msg;
    visualization_msgs::msg::Marker car_mesh;
    car_mesh.header.stamp = rclcpp::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::msg::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car->publish(markerArray_msg);
}





void GPS_callback(const sensor_msgs::msg::NavSatFix::SharedPtr GPS_msg)
{
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("Parkingfusion");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(100));


    auto sub_img = n->create_subscription<parking_interface::msg::Parking>("/gps", rclcpp::QoS(rclcpp::KeepLast(100)), image_callback);

    auto sub_rad = n->create_subscription<parking_interface::msg::Parking>("/vins_estimator/odometry", rclcpp::QoS(rclcpp::KeepLast(100)), radar_callback);





    pub_global_path = n->create_publisher<nav_msgs::msg::Path>("global_path", 100);
    pub_global_odometry = n->create_publisher<nav_msgs::msg::Odometry>("global_odometry", 100);
    pub_car = n->create_publisher<visualization_msgs::msg::MarkerArray>("car_model", 1000);


    global_path = &(globalEstimator.global_path);
    rclcpp::spin(n);
    return 0;
}

