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

using namespace std;

#define BUFFER_MAX_VALUE 100
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.45

rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused_parking;
std::deque<parking_interface::msg::Parking::SharedPtr> radDeque;
std::deque<parking_interface::msg::Parking::SharedPtr> imgDeque;
parking_interface::msg::Parking::SharedPtr match_radar;
parking_interface::msg::Parking::SharedPtr match_image;
int count_ = 1;

/*********************************/
/***********计算IOU部分***********/
/*********************************/
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
/*#define maxn 51
const float eps=1E-6;

int sig(float d){
    return (d > eps) - (d < -eps);
}

struct Point{
    float x,y; Point(){}
    Point(float x, float y):x(x),y(y){}
    bool operator == (const Point&p)const{
        return sig(x - p.x) == 0 && sig(y - p.y) == 0;
    }
};*/


/*calculte iou of polygons*/
/*float cross(Point o, Point a, Point b){  //叉积
    return (a.x-o.x)*(b.y-o.y)-(b.x-o.x)*(a.y-o.y);
}

float area(Point* ps, int n){
    ps[n]=ps[0];
    float res=0;
    for(int i=0;i<n;i++){
        res+=ps[i].x*ps[i+1].y-ps[i].y*ps[i+1].x;
    }
    return res/2.0;
}

int lineCross(Point a,Point b,Point c,Point d,Point&p){
    float s1,s2;
    s1=cross(a,b,c);
    s2=cross(a,b,d);
    if(sig(s1)==0&&sig(s2)==0) return 2;
    if(sig(s2-s1)==0) return 0;
    p.x=(c.x*s2-d.x*s1)/(s2-s1);
    p.y=(c.y*s2-d.y*s1)/(s2-s1);
    return 1;
}

void polygon_cut(Point*p,int&n,Point a,Point b, Point* pp){
//    static Point pp[maxn];
    int m=0;p[n]=p[0];
    for(int i=0;i<n;i++){
        if(sig(cross(a,b,p[i]))>0) pp[m++]=p[i];
        if(sig(cross(a,b,p[i]))!=sig(cross(a,b,p[i+1])))
            lineCross(a,b,p[i],p[i+1],pp[m++]);
    }
    n=0;
    for(int i=0;i<m;i++)
        if(!i||!(pp[i]==pp[i-1]))
            p[n++]=pp[i];
    while(n>1&&p[n-1]==p[0])n--;
}


//返回三角形oab和三角形ocd的有向交面积,o是原点//
float intersectArea(Point a,Point b,Point c,Point d){
    Point o(0,0);
    int s1=sig(cross(o,a,b));
    int s2=sig(cross(o,c,d));
    if(s1==0||s2==0)return 0.0;//退化，面积为0
    if(s1==-1) swap(a,b);
    if(s2==-1) swap(c,d);
    Point p[10]={o,a,b};
    int n=3;
    Point pp[maxn];
    polygon_cut(p,n,o,c, pp);
    polygon_cut(p,n,c,d, pp);
    polygon_cut(p,n,d,o, pp);
    double res=fabs(area(p,n));
    if(s1*s2==-1) res=-res;
    return res;
}

//求两多边形的交面积
float intersectArea(Point*ps1,int n1,Point*ps2,int n2){
    if(area(ps1,n1)<0) reverse(ps1,ps1+n1);
    if(area(ps2,n2)<0) reverse(ps2,ps2+n2);
    ps1[n1]=ps1[0];
    ps2[n2]=ps2[0];
    float res=0;
    for(int i=0;i<n1;i++){
        for(int j=0;j<n2;j++){
            res+=intersectArea(ps1[i],ps1[i+1],ps2[j],ps2[j+1]);
        }
    }
    if(res <= 0.0){
        return 0.0;
    }
    return res;//assumeresispositive!
}

//iou计算
float iou_poly(vector<float> p, vector<float> q) {
    Point ps1[maxn],ps2[maxn];
    int n1 = 4;
    int n2 = 4;
    for (int i = 0; i < 4; i++) {
        ps1[i].x = p[i * 2];
        ps1[i].y = p[i * 2 + 1];

        ps2[i].x = q[i * 2];
        ps2[i].y = q[i * 2 + 1];
    }
    float inter_area = intersectArea(ps1, n1, ps2, n2);
    float union_area = fabs(area(ps1, n1)) + fabs(area(ps2, n2)) - inter_area;
    float iou = inter_area / union_area;

//    cout << "inter_area:" << inter_area << endl;
//    cout << "union_area:" << union_area << endl;
//    cout << "iou:" << iou << endl;
    return iou;
}*/

float calculate_iou(parking_interface::msg::Parkinglst rad_box, parking_interface::msg::Parkinglst img_box){
    //rad_box、img_box转换为多边形
    //vector<float>box1, box2;
    /*float coord_min1 = min(rad_box.x1, rad_box.y1);
    float coord_min2 = min(rad_box.x2, rad_box.y2);
    float coord_min3 = min(rad_box.x3, rad_box.y3);
    float coord_min4 = min(rad_box.x4, rad_box.y4);
    float coord_min5 = min(coord_min1,coord_min2);
    float coord_min6 = min(coord_min3,coord_min4);
    float coord_min = min(coord_min5, coord_min6);
    coord_min1 = min(img_box.x1, img_box.y1);
    coord_min2 = min(img_box.x2, img_box.y2);
    coord_min3 = min(img_box.x3, img_box.y3);
    coord_min4 = min(img_box.x4, img_box.y4);
    coord_min5 = min(coord_min1,coord_min2);
    coord_min6 = min(coord_min3,coord_min4);
    coord_min = min(coord_min, min(coord_min5,coord_min6));
    box1.push_back(rad_box.x1 + coord_min);
    box1.push_back(rad_box.y1 + coord_min);
    box1.push_back(rad_box.x2 + coord_min);
    box1.push_back(rad_box.y2 + coord_min);
    box1.push_back(rad_box.x3 + coord_min);
    box1.push_back(rad_box.y3 + coord_min);
    box1.push_back(rad_box.x4 + coord_min);
    box1.push_back(rad_box.y4 + coord_min);*/
   
    /*box2.push_back(img_box.x1 + coord_min);
    box2.push_back(img_box.y1 + coord_min);
    box2.push_back(img_box.x2 + coord_min);
    box2.push_back(img_box.y2 + coord_min);
    box2.push_back(img_box.x3 + coord_min);
    box2.push_back(img_box.y3 + coord_min);
    box2.push_back(img_box.x4 + coord_min);
    box2.push_back(img_box.y4 + coord_min);*/
    
    box box1;
    box box2;
    box1 = {rad_box.x1, rad_box.y1, rad_box.x4, rad_box.y4};
    box2 = {img_box.x1, img_box.y1, img_box.x4, img_box.y4};
    //计算iou
    float iou = box_iou(box1, box2);
    return iou;
}

void publish_fused_parking(parking_interface::msg::Parking::SharedPtr img, parking_interface::msg::Parking::SharedPtr rad)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Begin to fuse");
    parking_interface::msg::Parking fused_frame;
    //以主传感器timestamp为准
    fused_frame.header.stamp= rad->header.stamp;
    fused_frame.header.frame_id = to_string(count_++);
    vector<parking_interface::msg::Parkinglst> lst1 = rad->parking;
    vector<parking_interface::msg::Parkinglst> lst2 = img->parking;
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
            // 2. img_box与rad_box有部分区域相交或者无区域相交
            else{
                float cur_iou = calculate_iou(rad_box, img_box);
                clog<<"iou: "<<cur_iou<<endl;
                // 2.2 IOU >= 0.8 && confidence >= 0.8
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

    match_radar = radDeque.back();
    long timestamp = match_radar->header.stamp.sec * (10 ^9) + match_radar -> header.stamp.nanosec;
    clog<<"timestamp"<<timestamp<<endl;
    int index = 0;
    long MIN_DIF = 6666666666666;
    long dif;
    for(int i = 0; i < imgDeque.size(); i++){
        clog<<"begin find image frame to match"<<endl;
        long imgstamp = imgDeque.at(i)->header.stamp.sec * (10 ^ 9) + imgDeque.at(i)->header.stamp.nanosec; 
        dif = abs(timestamp - imgstamp);
        
        if(dif == 0){
            index = i;
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "find matched image");
          match_image = imgDeque.at(index);
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
    
    clog<<"find it!!!"<<endl;
    clog<<"dif = "<<dif/(10^9)<<endl;
    match_image = imgDeque.at(index);

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
    if(radDeque.size() >= 1 && imgDeque.size() >= 1)
    {
        GetLatestFrames();
        publish_fused_parking(match_image, match_radar);
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
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("Parkingfusion");
    auto sub_img = n->create_subscription<parking_interface::msg::Parking>("image_parking", rclcpp::QoS(rclcpp::KeepLast(100)), image_callback);
    auto sub_rad = n->create_subscription<parking_interface::msg::Parking>("radar_parking", rclcpp::QoS(rclcpp::KeepLast(100)), radar_callback);
    pub_fused_parking = n->create_publisher<parking_interface::msg::Parking>("fused_parking", 100);
    rclcpp::spin(n);
    return 0;
}

