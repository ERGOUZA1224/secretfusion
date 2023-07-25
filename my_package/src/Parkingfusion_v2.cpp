#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <deque>
#include <vector>
#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
#include <vision_msgs/msg/detection3darray.hpp>
#include <chrono>   //计算时间
using namespace std::chrono;
using namespace std;

#define BUFFER_MAX_VALUE 100
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.8

rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused_parking;
std::deque<parking_interface::msg::Parking::SharedPtr> radDeque;
std::deque<parking_interface::msg::Parking::SharedPtr> imgDeque;
std::deque<vision_msgs::msg::Detection3DArray> detDeque;
parking_interface::msg::Parking::SharedPtr match_radar;
parking_interface::msg::Parking::SharedPtr match_image;
vision_msgs::msg::Detection3DArray::SharedPtr match_det;
int count_ = 1;

/*********************************/
/***********计算IOU部分***********/
/*********************************/

//垂直/水平车位
typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
} box;

float box_iou(const box& box1, const box& box2) {
    float w = std::max(std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min), 0.f);
    float h = std::max(std::min(box1.y_max, box2.y_max) - std::max(box1.y_min, box2.y_min), 0.f);
    float iou = w * h / ((box1.x_max - box1.x_min) * (box1.y_max - box1.y_min)  +
                                         (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) - w * h);
    return iou;
}


float calculate_iou(parking_interface::msg::Parkinglst rad_box, parking_interface::msg::Parkinglst img_box){
    box box1;
    box box2;
    box1 = {rad_box.x1, rad_box.y1, rad_box.x4, rad_box.y4};
    box2 = {img_box.x1, img_box.y1, img_box.x4, img_box.y4};
    //计算iou
    float iou = box_iou(box1, box2);
    return iou;
}

void publish_fused_parking(parking_interface::msg::Parking::SharedPtr img, parking_interface::msg::Parking::SharedPtr rad, vision_msgs::msg::Detection3DArray det)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Begin to fuse");
    parking_interface::msg::Parking fused_frame;
    
    fused_frame.header.stamp= img->header.stamp;
    fused_frame.header.frame_id = "map";

    vector<parking_interface::msg::Parkinglst> lst1 = rad->parking;
    vector<parking_interface::msg::Parkinglst> lst2 = img->parking;
    vector<vision_msgs::msg::Detection3D> lst3 = det -> detections;
    vector<parking_interface::msg::Parkinglst> lst_res;
    clog<<"lst1 size"<<lst1.size()<<";lst2 size"<<lst2.size()<<endl;
    if(lst1.size() == 0 || lst2.size() == 0){
        lst_res = {};
        clog<<"lst_res is empty"<<endl;
    }
    else{
    //LIST_RESULT deault_zero = {0,0,0,0,0,0,0,0,0,0};
    for(int i = 0; i < lst1.size(); i++){
        for(int j = 0; j < lst2.size(); j++){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Begin to fuse,enter loop");
            parking_interface::msg::Parkinglst rad_box = lst1[i];
            parking_interface::msg::Parkinglst img_box = lst2[j];
            // 1. img_box在rad_box内
            if(img_box.x1 >= rad_box.x1 && img_box.y1 >= rad_box.y1
            && img_box.x4 <= rad_box.x4 && img_box.y4 <= rad_box.y4){
                clog<<"img_box confidence:" <<img_box.confidence<<endl;
                // 1.1 confidence >= 0.8
                if(img_box.confidence >= DETECT_THRESH){
                    lst_res.push_back(img_box);
                    //clog<<"img_box"<<img_box.x1<<img_box.y1<<img_box.x2<<img_box.y2<<endl;
                    //clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;

                }
                 // 1.2 confidence <= 0.8
                else{
                    clog<<"no matched point1"<<endl;
                    continue;
                }
            }
            //2. imgbox中有障碍物
            else if(rad_box.x1 >= img_box.x1 && rad_box.y1 >= img_box.y1
            && rad_box.x4 <= img_box.x4 && rad_box.y4 <= img_box.y4){
                continue;
            }
            // 3. img_box与rad_box有部分区域相交或者无区域相交
            else{
                float cur_iou = calculate_iou(rad_box, img_box);
                clog<<"iou: "<<cur_iou<<endl;
                // 2.2 IOU >= thresh && confidence >= 0.8
                if(cur_iou >= IOU_THRESH && img_box.confidence >= DETECT_THRESH){
                    lst_res.push_back(img_box);
                }
                else{
                    clog<<"no matched point2"<<endl;
                    continue;
                }
            }
        }
        }
    }
    //clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;
    fused_frame.parking = lst_res;
    pub_fused_parking->publish(fused_frame);
    //clog<<"fused frame data: "<<fused_frame.header.stamp;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fused frame publishing...");
    //clog<<"fused frame publishing..."<<endl;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::str(fused_frame.header.stamp));
}


void GetLatestFrames()
{
    clog<<"time stamp begin!"<<endl;
    match_image = imgDeque.back();
    long timestamp = match_image->header.stamp.sec * (10 ^ 9) + match_image -> header.stamp.nanosec;
    clog<<"timestamp"<<timestamp<<endl;
    int index = 0;
    long MIN_DIF = 6666666666666;
    long dif;
    for(int i = 0; i < radDeque.size(); i++){
        clog<<"begin find image frame to match"<<endl;
        long radstamp = radDeque.at(i)->header.stamp.sec * (10 ^ 9) + radDeque.at(i)->header.stamp.nanosec; 
        dif = abs(timestamp - radstamp);
        
        if(dif == 0){
            index = i;
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "find matched image");
          match_radar = radDeque.at(index);
            //clog<<"match_image x1:"<<match_image->parking.at(0).x1<<endl;
            //radDeque.pop_back();
            //imgDeque.erase(imgDeque.begin()+index, imgDeque.begin()+index+1);
            return;
        }
        else{
            if(dif < MIN_DIF){
                index = i;
                MIN_DIF = dif;
            }
        }
    }
    
    clog<<"find radar!!!"<<endl;
    clog<<"dif = "<<dif/(10^9)<<endl;
    match_radar = radDeque.at(index);

    for(int j = 0; j < detDeque.size(); j++){
        clog<<"begin find image frame to match"<<endl;
        long detstamp = detDeque.at(j)->header.stamp.sec * (10 ^ 9) + detD = 148;
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
      lst5.x1 = 294;eque.at(j)->header.stamp.nanosec; 
        dif = abs(timestamp - detstamp);
        
        if(dif == 0){
            index = j;
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "find matched image");
          match_det = detDeque.at(index);
            //clog<<"match_image x1:"<<match_image->parking.at(0).x1<<endl;
            //radDeque.pop_back();
            //imgDeque.erase(imgDeque.begin()+index, imgDeque.begin()+index+1);
            return;
        }
        else{
            if(dif < MIN_DIF){
                index = j;
                MIN_DIF = dif;
            }
        }
    }
    clog<<"find detection!!!"<<endl;
    clog<<"dif = "<<dif/(10^9)<<endl;
    match_det = detDeque.at(index);

    //提取要融合的两帧后将他们从buffer中删除
    //radDeque.pop_back();
    //imgDeque.erase(imgDeque.begin()+index, imgDeque.begin()+index+1);
}

void radar_callback(const parking_interface::msg::Parking::SharedPtr rad_msg)
{
    if(radDeque.size() < BUFFER_MAX_VALUE){
        radDeque.push_back(rad_msg);
    }
    else{
        radDeque.pop_front();
        radDeque.push_back(rad_msg);
    }
    
    
}

void image_callback(const parking_interface::msg::Parking::SharedPtr img_msg)
{
    if(imgDeque.size() < BUFFER_MAX_VALUE){
        imgDeque.push_back(img_msg);
    }
    else{
        imgDeque.pop_front();
        imgDeque.push_back(img_msg);
    }
    if(radDeque.size() >= 1 && imgDeque.size() >= 1 && detDeque.size() >= 1)
    {
        GetLatestFrames();
        publish_fused_parking(match_image, match_radar, match_det);
        duration<double> diff = system_clock::now()- starttime;
        cout << "consuming time：" << diff.count() << "s" << endl;
    }
}

void det_callback(const vision_msgs::msg::Detection3DArray::SharedPtr det_msg)
{
    if(detDeque.size() < BUFFER_MAX_VALUE){
        detDeque.push_back(det_msg);
    }
    else{
        detDeque.pop_front();
        detDeque.push_back(det_msg);
    }   
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("Parkingfusion");
    auto sub_img = n->create_subscription<parking_interface::msg::Parking>("image_parking", rclcpp::QoS(rclcpp::KeepLast(100)), image_callback);
    auto sub_rad = n->create_subscription<parking_interface::msg::Parking>("radar_parking", rclcpp::QoS(rclcpp::KeepLast(100)), radar_callback);
    auto sub_det = n->create_subscription<vision_msgs::msg::Detection3DArray>("boundingbox", rclcpp::QoS(rclcpp::KeepLast(100)), det_callback);
    pub_fused_parking = n->create_publisher<parking_interface::msg::Parking>("fused_parking", 100);
    rclcpp::spin(n);
    return 0;
}

