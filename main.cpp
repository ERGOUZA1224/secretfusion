#include "mainwindow.h"
#include <QApplication>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
//using namespace cv;
#define BUFFER_MAX_VALUE 300
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.8


//车位融合 融合的是 环视车位 和 超声波车位信息

//环视数据
struct LIST_IMAGE_PARKING{
    float classID;
    float confidence;
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;
};

//image check result;
struct ST_IMAGE_PARKING
{
    float timestamp;  //timestamp
    vector<LIST_IMAGE_PARKING> lst_image;
};

//超声波数据
struct LIST_RADAR_PARKING{
    int   classID;
    float box_x;
    float box_y;
    float box_width;
    float box_height;
};

//radar check result;
struct ST_RADAR_PARKING
{
    float timestamp;  //timestamp
    vector<LIST_RADAR_PARKING> lst_radar;
};

struct LIST_RESULT{
    int classID;
    float confidence;
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;
};

// fusion parking result
struct ST_FUSION_PARKING{
    float timestamp;
    vector<LIST_RESULT> lst_result;
};

//接收的环视数据处理为4个点坐标
ST_FUSION_PARKING image2fusion(ST_IMAGE_PARKING img){
    clog<<"begin image2fusion"<<endl;

    vector<LIST_IMAGE_PARKING> lst2 = img.lst_image;
    vector<LIST_RESULT> lst1(lst2.size());
    clog<<"transform coordinate begin"<<endl;
    for(int i = 0; i < lst2.size(); i++){
        lst1.at(i).classID = lst2.at(i).classID;
        lst1.at(i).confidence = lst2.at(i).confidence;
        //clog<< lst1.at(i).classID<<endl;        lst1.at(i).confidence = lst2.at(i).confidence;
        lst1.at(i).x1 = lst2.at(i).x1;
        lst1.at(i).y1 = lst2.at(i).y1;
        lst1.at(i).x2 = lst2.at(i).x2;
        lst1.at(i).y2 = lst2.at(i).y2;
        lst1.at(i).x3 = lst2.at(i).x3;
        lst1.at(i).y3 = lst2.at(i).y3;
        lst1.at(i).x4 = lst2.at(i).x4;
        lst1.at(i).y4 = lst2.at(i).y4;
    }
    clog<<"transform coordinate finished!"<<endl;
    ST_FUSION_PARKING fusion;
    fusion.timestamp = img.timestamp;
    fusion.lst_result = lst1;
    return fusion;
}

//接收的超声波数据处理为4个点坐标
ST_FUSION_PARKING radar2fusion(ST_RADAR_PARKING rad){
    clog<<"begin radar2fusion"<<endl;

    vector<LIST_RADAR_PARKING> lst2 = rad.lst_radar;
    vector<LIST_RESULT> lst1(lst2.size());
    for(int i = 0; i < lst2.size(); i++){
        lst1.at(i).classID = lst2.at(i).classID;
        lst1.at(i).confidence = 1.0;
        lst1.at(i).x1 = lst2.at(i).box_x - lst2.at(i).box_height/2;
        lst1.at(i).y1 = lst2.at(i).box_y + lst2.at(i).box_width/2;
        lst1.at(i).x2 = lst2.at(i).box_x + lst2.at(i).box_height/2;
        lst1.at(i).y2 = lst2.at(i).box_y + lst2.at(i).box_width/2;
        lst1.at(i).x3 = lst2.at(i).box_x - lst2.at(i).box_height/2;
        lst1.at(i).y3 = lst2.at(i).box_y - lst2.at(i).box_width/2;
        lst1.at(i).x4 = lst2.at(i).box_x + lst2.at(i).box_height/2;
        lst1.at(i).y4 = lst2.at(i).box_y - lst2.at(i).box_width/2;
    }
    ST_FUSION_PARKING fusion;
    fusion.timestamp = rad.timestamp;
    fusion.lst_result = lst1;
    return fusion;
}

deque<ST_FUSION_PARKING> image_buffer;
deque<ST_FUSION_PARKING> radar_buffer;

void AddSensorMeasurement_Image(ST_IMAGE_PARKING img)
{
    clog<<"add img"<<endl;
    ST_FUSION_PARKING fusion= image2fusion(img);
    clog<<"trasnform img to st_fusion"<<endl;
    if(image_buffer.size() < BUFFER_MAX_VALUE){
        image_buffer.push_back(fusion);
    }
    else{
        image_buffer.pop_front();
        image_buffer.push_back(fusion);
    }
}
void AddSensorMeasurement_Radar(ST_RADAR_PARKING rad)
{
    ST_FUSION_PARKING fusion= radar2fusion(rad);
    if(radar_buffer.size() < BUFFER_MAX_VALUE){
        radar_buffer.push_back(fusion);
    }
    else{
        radar_buffer.pop_front();
        radar_buffer.push_back(fusion);
    }
}

//deque<ST_FUSION_PARKING> match_buffer;
//void DrawRec(){
//    const char* filename = "home/ddbb/01.jpg";

//    cv::Mat mat = cv::imread(filename);
//    if (mat.empty()) {

//         throw("Faild open file.");
//    }

//}
ST_FUSION_PARKING match_image;
ST_FUSION_PARKING match_radar;

//set radar as the main sensor
void GetLatestFrames()
{
    clog<<"time stamp begin!"<<endl;

    match_radar = radar_buffer.back();

    float timestamp = match_radar.timestamp;
    int index = 0;
    float MIN_DIF =66666;
    for(int i = 0; i < image_buffer.size(); i++){
        clog<<"begin find image frame to match"<<endl;
        float dif = abs(timestamp - image_buffer.at(i).timestamp);
        if(dif == 0.0){
            match_image = image_buffer.at(i);
            clog<<"find it!!!"<<endl;
            clog<<"match_image x1:"<<match_image.lst_result.at(0).x1<<endl;
            radar_buffer.pop_back();
            image_buffer.erase(image_buffer.begin()+index, image_buffer.begin()+index+1);
            return;
        }
        else{
            if(dif < MIN_DIF){
                index = i;
                MIN_DIF = dif;
            }
        }
    }

    match_image = image_buffer.at(index);
    clog<<"time stamp matched!"<<endl;
    //提取要融合的两帧后将他们从buffer中删除
    radar_buffer.pop_back();
    image_buffer.erase(image_buffer.begin()+index, image_buffer.begin()+index+1);
}

/*********************************/
/***********计算IOU部分***********/
/*********************************/
#define maxn 51
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
};

float cross(Point o, Point a, Point b){  //叉积
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
    if(s1*s2==-1) res=-res;return res;
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
}

float calculate_iou(LIST_RESULT rad_box, LIST_RESULT img_box){
    //rad_box、img_box转换为多边形
    vector<float>box1, box2;
    box1.push_back(rad_box.x1);
    box1.push_back(rad_box.y1);
    box1.push_back(rad_box.x2);
    box1.push_back(rad_box.y2);
    box1.push_back(rad_box.x3);
    box1.push_back(rad_box.y3);
    box1.push_back(rad_box.x4);
    box1.push_back(rad_box.y4);
    /*box1.push_back(Point(rad_box.x1, rad_box.y1));
    box1.push_back(Point(rad_box.x2, rad_box.y2));
    box1.push_back(Point(rad_box.x3, rad_box.y3));
    box1.push_back(Point(rad_box.x4, rad_box.y4));*/
    box2.push_back(img_box.x1);
    box2.push_back(img_box.y1);
    box2.push_back(img_box.x2);
    box2.push_back(img_box.y2);
    box2.push_back(img_box.x3);
    box2.push_back(img_box.y3);
    box2.push_back(img_box.x4);
    box2.push_back(img_box.y4);
    /*box2.push_back(Point(img_box.x1, img_box.y1));
    box2.push_back(Point(img_box.x2, img_box.y2));
    box2.push_back(Point(img_box.x3, img_box.y3));
    box2.push_back(Point(img_box.x4, img_box.y4));*/

    //计算iou
    float iou = iou_poly(box1, box2);
    return iou;
}

//假设车位坐标点已按x1值由小到大排序
ST_FUSION_PARKING FuseFrame(ST_FUSION_PARKING img, ST_FUSION_PARKING rad)
{
    clog<<"begin fuse frame"<<endl;
    ST_FUSION_PARKING fused_frame;
    //以主传感器timestamp为准
    fused_frame.timestamp = rad.timestamp;
    vector<LIST_RESULT> lst1 = rad.lst_result;
    vector<LIST_RESULT> lst2 = img.lst_result;
    vector<LIST_RESULT> lst_res;
    //LIST_RESULT deault_zero = {0,0,0,0,0,0,0,0,0,0};
    for(int i = 0; i < lst1.size(); i++){
        for(int j = 0; j < lst2.size(); j++){
            LIST_RESULT rad_box = lst1.at(i);
            LIST_RESULT img_box = lst2.at(j);
            // 1. img_box在rad_box内
            if(img_box.x1 >= rad_box.x1 && img_box.y1 <= rad_box.y1
            && img_box.x4 <= rad_box.x4 && img_box.y4 >= rad_box.y4){
                clog<<"img_box confidence:" <<img_box.confidence<<endl;
                // 1.1 confidence >= 0.8
                if(img_box.confidence >= DETECT_THRESH){
                    lst_res.push_back(img_box);
                    clog<<"img_box"<<img_box.x1<<img_box.y1<<img_box.x2<<img_box.y2<<endl;
                    clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;

                }
                 // 1.2 confidence <= 0.8
                else{
                    clog<<"no matched point1"<<endl;

                    //void DrawRec(){
                    //    const char* filename = "home/ddbb/01.jpg";

                    //    cv::Mat mat = cv::imread(filename);
                    //    if (mat.empty()) {

                    //         throw("Faild open file.");
                    //    }

                    //}              continue;
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
    clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;
    fused_frame.lst_result = lst_res;
    clog<<"fuse frame finished!"<<endl;
    return fused_frame;
}


deque<ST_FUSION_PARKING> fused_buffer;

void collectFusedObject(ST_FUSION_PARKING fused)
{
    if(fused_buffer.size() >= BUFFER_MAX_VALUE){
        fused_buffer.pop_front();
        fused_buffer.push_back(fused);
    }
    else{
        fused_buffer.push_back(fused);
    }
}





int main(int argc, char *argv[])
{
    ST_IMAGE_PARKING img = {2023.0522, {{1, 0.85, 201, 53, 269, 52, 201, 243, 264, 244}}};
    ST_RADAR_PARKING rad = {2023.0522,{{2,208,149,40,270},{1,55,146,108,290}}};
    AddSensorMeasurement_Image(img);
    AddSensorMeasurement_Radar(rad);
    GetLatestFrames();
    ST_FUSION_PARKING fused_frame;
    fused_frame = FuseFrame(match_image, match_radar);
    collectFusedObject(fused_frame);

    return 0;
}
