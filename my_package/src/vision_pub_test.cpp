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
#include <visualization_msgs/msg/marker.hpp>
using namespace std;
using namespace std::chrono_literals;


rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_fused1;
rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_fused2;


rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_text1;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_text2;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_text3;

//融合数据可视化
void vision_callback1(const parking_interface::msg::Parking::SharedPtr fused_msg){
    for(int i = 0; i < fused_msg->parking.size(); i++){
        
        geometry_msgs::msg::PolygonStamped vision_msg;
        vision_msg.header.stamp = fused_msg->header.stamp;
        vision_msg.header.frame_id = "my_frame";
        geometry_msgs::msg::Point32 point;
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
        
        //polygon_lst.polygonlst.push_back(vision_msg);
        if(i == 0){
            pub_fused1 -> publish(vision_msg);
        }
        else if(i == 1){
            pub_fused2 -> publish(vision_msg);
        }
        else{
            pub_fused3 -> publish(vision_msg);
        }
    }
        //visualize parking slot id
        visualization_msgs::msg::Marker marker;//定义Marker对象
        marker.header.stamp = fused_msg->header.stamp;
        marker.header.frame_id = "my_frame";
		marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;//选用文本类型
		marker.ns = "basic_shapes";//必写，否则rviz无法显示
		marker.pose.orientation.w = 1.0;//文字的方向
		marker.id = 1;//用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
		marker.scale.x = 20;
		marker.scale.y = 20;
		marker.scale.z = 20;//文字的大小
		marker.color.g = 1.0f;
		marker.color.a = 1;//必写，否则rviz无法显示
        geometry_msgs::msg::Pose pose;
        pose.position.x = 34;
        pose.position.y = 270;
        pose.position.z = 0;
        int iid;
        iid =(int)fused_msg->parking.at(0).slotid;
        marker.text= to_string(iid);//文字内容
        marker.pose=pose;//文字的位置
        pub_text1 -> publish(marker);

         //visualize parking slot id
        visualization_msgs::msg::Marker marker2;//定义Marker对象
        marker2.header.stamp = fused_msg->header.stamp;
        marker2.header.frame_id = "my_frame";
		marker2.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;//选用文本类型
		marker2.ns = "basic_shapes";//必写，否则rviz无法显示
		marker2.pose.orientation.w = 1.0;//文字的方向
		marker2.id = 1;//用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
		marker2.scale.x = 20;
		marker2.scale.y = 20;
		marker2.scale.z = 20;//文字的大小
		marker2.color.g = 1.0f;
		marker2.color.a = 1;//必写，否则rviz无法显示
        geometry_msgs::msg::Pose pose2;
        pose2.position.x = 102;
        pose2.position.y = 270;
        pose2.position.z = 0;
        iid =(int)fused_msg->parking.at(1).slotid;
        marker2.text= to_string(iid);//文字内容
        marker2.pose=pose2;//文字的位置
        pub_text2 -> publish(marker2);

        visualization_msgs::msg::Marker marker3;//定义Marker对象
        marker3.header.stamp = fused_msg->header.stamp;
        marker3.header.frame_id = "my_frame";
		marker3.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;//选用文本类型
		marker3.ns = "basic_shapes";//必写，否则rviz无法显示
		marker3.pose.orientation.w = 1.0;//文字的方向
		marker3.id = 1;//用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
		marker3.scale.x = 20;
		marker3.scale.y = 20;
		marker3.scale.z = 20;//文字的大小
		marker3.color.g = 1.0f;
		marker3.color.a = 1;//必写，否则rviz无法显示
        geometry_msgs::msg::Pose pose3;
        pose3.position.x = 250;
        pose3.position.y = 270;
        pose3.position.z = 0;
        iid =(int)fused_msg->parking.at(2).slotid;
        marker3.text= to_string(iid);//文字内容
        marker3.pose=pose3;//文字的位置
        pub_text3 -> publish(marker3);


}

//环视数据可视化
void vision_callback2(const parking_interface::msg::Parking::SharedPtr fused_msg){
    for(int i = 0; i < fused_msg->parking.size(); i++){
        
        geometry_msgs::msg::PolygonStamped vision_msg;
        vision_msg.header.stamp = fused_msg->header.stamp;
        vision_msg.header.frame_id = "my_frame";
        geometry_msgs::msg::Point32 point;
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
        
        //polygon_lst.polygonlst.push_back(vision_msg);
        if(i == 0){
            pub_img1 -> publish(vision_msg);
        }
        else if(i == 1){
            pub_img2 -> publish(vision_msg);
        }
        else if(i == 2){
            pub_img3 -> publish(vision_msg);
        }
        else if(i == 3){
            pub_img4 -> publish(vision_msg);
        }
        else{
            pub_img5 -> publish(vision_msg);
        }
    }
}

//超声波雷达数据可视化
void vision_callback3(const parking_interface::msg::Parking::SharedPtr fused_msg){
    for(int i = 0; i < fused_msg->parking.size(); i++){
        
        geometry_msgs::msg::PolygonStamped vision_msg;
        vision_msg.header.stamp = fused_msg->header.stamp;
        vision_msg.header.frame_id = "my_frame";
        geometry_msgs::msg::Point32 point;
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
        
        //polygon_lst.polygonlst.push_back(vision_msg);
        if(i == 0){
            pub_rad1 -> publish(vision_msg);
        }
        else if(i == 1){
            pub_rad2 -> publish(vision_msg);
        }
        else{
            pub_rad3 -> publish(vision_msg);
        }
    }
}
    

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("visualization_pub");
    auto sub_fused= n->create_subscription<parking_interface::msg::Parking>("tracked_parking", 100,  vision_callback1);
    auto sub_img= n->create_subscription<parking_interface::msg::Parking>("image_parking", 100,  vision_callback2);
    auto sub_rad= n->create_subscription<parking_interface::msg::Parking>("radar_parking", 100,  vision_callback3);
    pub_fused1 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("fused_pub1", 100);
    pub_fused2 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("fused_pub2", 100);
    pub_img1 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("img_pub1", 100);
    pub_img2 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("img_pub2", 100);
    pub_img3 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("img_pub3", 100);
    pub_rad1 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("rad_pub1", 100);
    pub_rad2 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("rad_pub2", 100);
    pub_rad3 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("rad_pub3", 100);
    pub_img4 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("img_pub4", 100);
    pub_img5 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("img_pub5", 100);
    pub_fused3 =  n->create_publisher<geometry_msgs::msg::PolygonStamped>("fused_pub3", 100);
    pub_text1 = n -> create_publisher<visualization_msgs::msg::Marker>("text1",100);
    pub_text2 = n -> create_publisher<visualization_msgs::msg::Marker>("text2",100);
    pub_text3 = n -> create_publisher<visualization_msgs::msg::Marker>("text3",100);
    rclcpp::spin(n);
    return 0;
}