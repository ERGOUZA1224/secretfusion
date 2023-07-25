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
#include "parking_interface/msg/parkinglist.hpp"
#include "parking_interface/msg/point2_d.hpp"
#include <chrono>   //计算时间
#include <memory>
#include <iomanip> // to format image names using setw() and setfill()
#include <unistd.h>
#include <chrono>
#include <set>

#include "Hungarian.h"
#include "KalmanTracker.h"

#include "opencv2/video/tracking.hpp"
using std::placeholders::_1;
using namespace std::chrono;
using namespace std;
using namespace cv;

int slotID = 1;

typedef struct TrackingBox
{
	int frame;
	int id;
	Rect_<float> box;
}TrackingBox;

// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_tracked;
std::deque<parking_interface::msg::Parking> fused_buffer;
using namespace std::chrono;
using namespace std;

#define BUFFER_MAX_VALUE 100
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.7
// global variables for counting
#define CNUM 20
parking_interface::msg::Parking tracked_parking;
vector<TrackingBox> detData;
vector<KalmanTracker> trackers;
int maxFrame = 1;
HungarianAlgorithm HungAlgo;
int frame_i = 1;

rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused_parking;
std::deque<parking_interface::msg::Parking::SharedPtr> radDeque;
std::deque<parking_interface::msg::Parking::SharedPtr> imgDeque;
parking_interface::msg::Parking::SharedPtr match_radar;
parking_interface::msg::Parking::SharedPtr match_image;
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


//用于计算相交部分占img_box的IOU
float calculate_imgiou(parking_interface::msg::Parkinglist rad_box, parking_interface::msg::Parkinglist img_box){
    box box1;
    box box2;
    box1 = {rad_box.pointlist[0].x, rad_box.pointlist[0].y, rad_box.pointlist[2].x, rad_box.pointlist[2].y};
    box2 = {img_box.pointlist[0].x, img_box.pointlist[0].y, img_box.pointlist[2].x, img_box.pointlist[2].y};
    float w = std::max(std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min), 0.f);
    float h = std::max(std::min(box1.y_max, box2.y_max) - std::max(box1.y_min, box2.y_min), 0.f);
    float iou = w * h /  ((box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) );
    return iou;
}

float calculate_area(parking_interface::msg::Parkinglist rad_box){
    box box2;
    box2 = {rad_box.pointlist[0].x, rad_box.pointlist[0].y, rad_box.pointlist[2].x, rad_box.pointlist[2].x};
    float s = (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) ;
    return s;
}


//倾斜车位
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

float calculate_iou(parking_interface::msg::Parkinglist rad_box, parking_interface::msg::Parkinglist img_box){
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
    box2.push_back(img_box.y2 + coord_min);img
    box2.push_back(img_box.x3 + coord_min);
    box2.push_back(img_box.y3 + coord_min);
    box2.push_back(img_box.x4 + coord_min);
    box2.push_back(img_box.y4 + coord_min);*/
    
    box box1;
    box box2;
    box1 = {rad_box.pointlist[0].x, rad_box.pointlist[0].y, rad_box.pointlist[2].x, rad_box.pointlist[2].y};
    box2 = {img_box.pointlist[0].x, img_box.pointlist[0].y, img_box.pointlist[2].x, img_box.pointlist[2].y};
    //计算iou
    float iou = box_iou(box1, box2);
    return iou;
}




void topic_callback(const parking_interface::msg::Parking fused_msg)
{
	detData.clear();
    //darknet_ros_msgs::BoundingBoxes bboxes;
   // bboxes.bounding_boxes = msg->bounding_boxes;
   parking_interface::msg::Parking pparking;
   pparking.parkinglist = fused_msg. parkinglist;
    // 1. read bounding boxes from object detector, here from YOLO v3 ROS version.
    for (int i = 0; i < int(pparking.parkinglist.size()) ; i++)
    {
        TrackingBox tb;
        tb.frame = frame_i;
        tb.id = pparking.parkinglist[i].id;
        tb.box = Rect_<float>(Point_<float>(pparking.parkinglist[i].pointlist[0].x, pparking.parkinglist[i].pointlist[0].y), 
        Point_<float>(pparking.parkinglist[i].pointlist[2].x, pparking.parkinglist[i].pointlist[2].y));
		detData.push_back(tb);
    }
	frame_i++;

	// cout << "detData.size: " << detData.size() << endl;

	// 2. group detData by frame
	vector<vector<TrackingBox>> detFrameData;
    detFrameData.push_back(detData);

	// 3. update across frames
	int frame_count = 0;
	int max_age = 2;
	int min_hits = 3;
	double iouThreshold = 0.3;
	//vector<KalmanTracker> trackers;
	//KalmanTracker::kf_count = 0; // tracking id relies on this, so we have to reset it in each seq.

	// variables used in the for-loop
	vector<Rect_<float>> predictedBoxes;
	vector<vector<double>> iouMatrix;
	vector<int> assignment;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	vector<TrackingBox> frameTrackingResult;
    vector<parking_interface::msg::Parkinglist> parkinglst;
	unsigned int trkNum = 0;
	unsigned int detNum = 0;

	double cycle_time = 0.0;
	int64 start_time = 0;
	set<int> manageId;

	//////////////////////////////////////////////
	// main loop
	for (int fi = 0; fi < maxFrame; fi++)
	{

		frame_count++;
		assignment.clear();
		unmatchedTrajectories.clear();
		unmatchedDetections.clear();
		allItems.clear();
		matchedItems.clear();
		// I used to count running time using clock(), but found it seems to conflict with cv::cvWaitkey(),
		// when they both exists, clock() can not get right result. Now I use cv::getTickCount() instead.
		start_time = getTickCount();
		//cout<<"tracker size: "<<trackers.size()<<endl;
		if (trackers.size() == 0) // the first frame met
		{
			// initialize kalman trackers using first detections.
			for (unsigned int i = 0; i < detFrameData[fi].size(); i++)
			{
				KalmanTracker trk = KalmanTracker(detFrameData[fi][i].box);
				trackers.push_back(trk);
			}
			// output the first frame detections
			for (unsigned int id = 0; id < detFrameData[fi].size(); id++)
			{
				TrackingBox tb = detFrameData[fi][id];
			}
			
			continue;
		}

		///////////////////////////////////////
		// 3.1. get predicted locations from existing trackers.
		predictedBoxes.clear(); 
		//cout<<"step 3.1 begin"<<endl;
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			Rect_<float> pBox = (*it).predict();
			if (pBox.x >= 0 && pBox.y >= 0)
			{
				predictedBoxes.push_back(pBox);
				it++;
			}
			else
			{
				it = trackers.erase(it);
				//cerr << "Box invalid at frame: " << frame_count << endl;
			}
		}

		///////////////////////////////////////
		// 3.2. associate detections to tracked object (both represented as bounding boxes)
		// dets : detFrameData[fi]
		trkNum = predictedBoxes.size();
		detNum = detFrameData[fi].size();

		// cout << "detNum: " << detNum << endl;
		// cout << "trkNum" << trkNum << endl;

		iouMatrix.clear();
		iouMatrix.resize(trkNum, vector<double>(detNum, 0));

		for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
		{
			for (unsigned int j = 0; j < detNum; j++)
			{
				// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
				iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detFrameData[fi][j].box);
			}
		}

		// solve the assignment problem using hungarian algorithm.
		// the resulting assignment is [track(prediction) : detection], with len=preNum
		
		//assignment.clear();
		HungAlgo.Solve(iouMatrix, assignment);

		// find matches, unmatched_detections and unmatched_predictions
		// unmatchedTrajectories.clear();
		// unmatchedDetections.clear();
		// allItems.clear();
		// matchedItems.clear();

		if (detNum > trkNum) //	there are unmatched detections
		{
			for (unsigned int n = 0; n < detNum; n++)
				allItems.insert(n);

			for (unsigned int i = 0; i < trkNum; ++i)
				matchedItems.insert(assignment[i]);

			set_difference(allItems.begin(), allItems.end(),
				matchedItems.begin(), matchedItems.end(),
				insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
		}
		else if (detNum < trkNum) // there are unmatched trajectory/predictions
		{
			for (unsigned int i = 0; i < trkNum; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatchedTrajectories.insert(i);
		}
		else
			;

		// filter out matched with low IOU
		matchedPairs.clear();
		for (unsigned int i = 0; i < trkNum; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
			if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
			{
				unmatchedTrajectories.insert(i);
				unmatchedDetections.insert(assignment[i]);
			}
			else
				matchedPairs.push_back(cv::Point(i, assignment[i]));
		}

		int i = 0;
		for (auto umd : unmatchedDetections){
			cout << "unmatchedDetections" << endl;
			cout << "i = " << i << " , value = " << umd << endl;
			i ++ ;
		}
		i = 0;
		for (auto umd : unmatchedTrajectories){
			cout << "unmatchedTrajectories" << endl;
			cout << "i = " << i << " , value = " << umd << endl;
			i++;
		}

		///////////////////////////////////////
		// 3.3. updating trackers

		// update matched trackers with assigned detections.
		// each prediction is corresponding to a tracker
		int detIdx, trkIdx;
		for (unsigned int i = 0; i < matchedPairs.size(); i++)
		{
			trkIdx = matchedPairs[i].x;
			detIdx = matchedPairs[i].y;
			trackers[trkIdx].update(detFrameData[fi][detIdx].box);
			manageId.insert(trackers[trkIdx].m_id);
		}
		// cout << "11111" << endl;
		// cout << trackers.size() << endl;
		// create and initialise new trackers for unmatched detections
		for (auto umd : unmatchedDetections)
		{
			// KalmanTracker tracker = KalmanTracker(detFrameData[fi][umd].box);
			// trackers.push_back(tracker);
			while(1)
			{
				set<int>::iterator Iter = manageId.find(KalmanTracker::kf_count);
				if(Iter != manageId.end())
				{
					//ROS_INFO("find %d, %d",KalmanTracker::kf_count,(*Iter));
					//KalmanTracker::kf_count++; mody
				}
				else
				{
					//ROS_INFO("not find %d, %d",KalmanTracker::kf_count,(*Iter));
					break;
				}
			}

			manageId.insert(KalmanTracker::kf_count);
			KalmanTracker tracker = KalmanTracker(detFrameData[fi][umd].box);
			//ROS_INFO("UM m_id %d **",tracker.m_id);
			
			trackers.push_back(tracker);
			
			cout<<"Add New Detection"<<endl;
		}

		// get trackers' output
		detData.clear();
		detFrameData.clear();
		frameTrackingResult.clear();
// 		for (auto it = trackers.begin(); it != trackers.end();)
// 		{
// 			if (((*it).m_time_since_update < 1) &&
// 				((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
// 			{
// 				TrackingBox res;
// 				res.box = (*it).get_state();
// 				res.id = (*it).m_id + 1;
// 				res.frame = frame_count;
// 				frameTrackingResult.push_back(res);
// 				it++;
// 			}
// 			else
// 				it++;
// 			// remove dead tracklet
// 			if (it != trackers.end() && (*it).m_time_since_update > max_age)
// 				it = trackers.erase(it);
// //		    if (display) // read image, draw results and show them
// //		    {
// //			    cv::rectangle(img, tb.box, randColor[tb.id % CNUM], 2, 8, 0);
// //		    }

// 		}
//        result_boxes.publish(box);
	vector<parking_interface::msg::Parkinglist> lst1;
	
	for (auto it = trackers.begin(); it != trackers.end();)
		{
			//ROS_INFO("tb id ");
			if (((*it).m_time_since_update < 1) &&
				((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
			{
				//darknet_ros_msgs::BoundingBox boundingBox;
				TrackingBox res;
				res.box = (*it).get_state();
				res.id = (*it).m_id + 1;
				res.frame = frame_count;
				// if(display){
				// //cv::rectangle(frame, res.box, randColor[res.id % CNUM], 2, 8, 0);
				// cv::rectangle(frame, res.box, Scalar_<int>(0,0,255), 2, 8, 0);
				// cv::putText(frame,std::to_string(res.id),Point_<int>(int(res.box.x),int(res.box.y)),cv::FONT_ITALIC,1,randColor[res.id % CNUM],2);
				// cv::imshow("view", frame);
				// cv::waitKey(1);
				// }
				frameTrackingResult.push_back(res);
				parking_interface::msg::Parkinglist reslst;
				//cout<<"1 id:"<< res.id<<endl;
				reslst.id = res.id;
                parking_interface::msg::Point2D pt1,pt2,pt3,pt4;
				//reslst.confidence = 1;
				pt1.x = res.box.x;
				pt1.y = res.box.y;
				pt2.x = res.box.x;
				pt2.y = res.box.y + res.box.height;
				pt3.x = res.box.x + res.box.width;
				pt3.y = res.box.y + res.box.height;
                pt4.x = res.box.x + res.box.width;
				pt4.y = res.box.y;
                reslst.pointlist.push_back(pt1);
                reslst.pointlist.push_back(pt2);
                reslst.pointlist.push_back(pt3);
                reslst.pointlist.push_back(pt4);
				lst1.push_back(reslst);
				it++;
				
			}
			else
				it++;
			if(it != trackers.end() && (*it).m_time_since_update > max_age)
				{
				trackers.erase(it);
				manageId.erase((*it).m_id);
				
				//it--;
				cout<<"ERASE"<<endl;
				}
			cout<<"lst1size: "<<lst1.size()<<endl;
		}
			// remove dead tracklet
			//ROS_INFO("time %d",(*it).m_time_since_update);
			//if (it != trackers.end() && (*it).m_time_since_update > max_age)
			//{
			//	manageId.erase((*it).m_id); // filter id remove
			//	ROS_INFO("m_time: %d ERASE",(*it).m_time_since_update);
			//	it = trackers.erase(it);
				
			//}
			// for (auto it = trackers.begin(); it != trackers.end();)
			// {
			// 	darknet_ros_msgs::BoundingBox boundingBox;
			// 	boundingBox.id = (*it).m_id;
			// 	boundingBox.xmin = (*it).get_state().x;
			// 	boundingBox.ymin = (*it).get_state().y;
			// 	boundingBox.xmax = (*it).get_state().x + (*it).get_state().width;
			// 	boundingBox.ymax = (*it).get_state().y + (*it).get_state().height;
			// 	tracked_bboxes.bounding_boxes.push_back(boundingBox);
			// }trackers
			//ROS_INFO("tb id ");
//		    if (display) // read image, draw results and show them
//		    {
//			    cv::rectangle(img, tb.box, randColor[tb.id % CNUM], 2, 8, 0);
//		    }
			
		
    // int frame = frameTrackingResult.front().frame;
    // vector<parking_interface::msg::Parkinglst> res;
    // parking_interface::msg::Parking tracked_parking;
    // for(auto tb : frameTrackingResult){
    //       parking_interface::msg::Parkinglst reslst;
    //       reslst.slotid = tb.id;
    //       reslst.confidence = 1;
    //       reslst.x1 = tb.box.x;
    //       reslst.y1 = tb.box.y;
    //       reslst.x2 = tb.box.x + tb.box.height;
    //       reslst.y2 = tb.box.y;
    //       reslst.x3 = tb.box.x;
    //       reslst.y3 = tb.box.y + tb.box.width;
    //       reslst.x4 = tb.box.x + tb.box.height;
    //       reslst.y4 = tb.box.y + tb.box.width;
    //       res.push_back(reslst);
    //     }
	// 	tracked_parking.parking = res;
	//  	pub_tracked -> publish(tracked_parking);
		// for(int i = 0; i < lst1.size(); i++){
		// 	cout<<"id: "<<lst1[i].slotid<<endl;
		// }
		// tracked_parking.parking.clear();
		tracked_parking.parkinglist = lst1;
		cycle_time = (double)(getTickCount() - start_time);
        double fps = (1.0/cycle_time)*getTickFrequency();
		pub_fused_parking -> publish(tracked_parking);

		cout<<"tracker size: "<<trackers.size()<<endl;
		for(int i = 0; i < tracked_parking.parkinglist.size(); i++){
			
			cout<<"slot id:"<<(int)tracked_parking.parkinglist[i].id<<endl;
		}
		tracked_parking.parkinglist.clear();
		//cout<<"tracked_parking id: "<<tracked_parking.parking.back().slotid<<endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current : %.1f", fps);

	}
}

void publish_fused_parking(parking_interface::msg::Parking::SharedPtr img, parking_interface::msg::Parking::SharedPtr rad)
{
    parking_interface::msg::Parking fused_frame;
    
    fused_frame.header.stamp= rad->header.stamp;
    fused_frame.header.frame_id = "map";

    vector<parking_interface::msg::Parkinglist> lst1 = rad->parkinglist;
    vector<parking_interface::msg::Parkinglist> lst2 = img->parkinglist;
    vector<parking_interface::msg::Parkinglist> lst_res;
    if(lst1.size() == 0 || lst2.size() == 0){
      clog<<"lst_res is empty"<<endl;
      //发布空白数据
      lst_res.clear();
    }
    else{
        for(int i = 0; i < lst1.size(); i++){
            for(int j = 0; j < lst2.size(); j++){
                parking_interface::msg::Parkinglist rad_box = lst1[i];
                parking_interface::msg::Parkinglist img_box = lst2[j];
                //有相交
                if(calculate_iou(rad_box,img_box) > 0){
                    // 1. rad_box > img_box
                    if(calculate_area(rad_box) >= calculate_area(img_box)){
                        // 1.1 img_box刚好在rad_box内
                        if(img_box.pointlist[0].x >= rad_box.pointlist[0].x && img_box.pointlist[0].y >= rad_box.pointlist[0].y
                        && img_box.pointlist[2].x <= rad_box.pointlist[2].x && img_box.pointlist[2].y <= rad_box.pointlist[2].y){
                            lst_res.push_back(img_box);
                        }
                        // 1.2 相交且IOU大于0.9，无障碍物区域够停车
                        else if(calculate_imgiou(rad_box, img_box) >= 0.8){
                            lst_res.push_back(img_box);
                        }
                        //1.3 无障碍物区域不够停车
                        else{
                           ;
                        }
                    }
                    //2. img_box > rad_box
                    else{
                        //2.1 rad_box刚好在img_box内，空白区域够停车
                        if((rad_box.pointlist[0].x >= img_box.pointlist[0].x && rad_box.pointlist[0].y >= img_box.pointlist[0].y
                        && rad_box.pointlist[2].x <= img_box.pointlist[2].x && rad_box.pointlist[2].y <= img_box.pointlist[2].y) 
                        && (calculate_iou(rad_box, img_box) >= 0.9)){
                            lst_res.push_back(img_box);
                        }
                        // 2.2 rad_box与img_box相交，IOU大于0.85
                        else if(calculate_imgiou(rad_box, img_box) >= 0.8){
                            lst_res.push_back(img_box);
                        }
                        // 2.3 img_box内无空白区域或空白区域不够停车
                        else{
                            ;
                        }
                    } 
                }
                //无相交
                else{
                    ;
                }
            }
        }
    }
    fused_frame.parkinglist = lst_res;
    //pub_fused_parking->publish(fused_frame);
    topic_callback(fused_frame);
    cout<<"fused_parking num :" << fused_frame.parkinglist.size()<<endl;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fused frame publishing...");
}
/*
void publish_fused_parking_v1(parking_interface::msg::Parking::SharedPtr img, parking_interface::msg::Parking::SharedPtr rad)
{
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Begin to fuse");
    parking_interface::msg::Parking fused_frame;
    
    fused_frame.header.stamp= rad->header.stamp;
    fused_frame.header.frame_id = "map";

    vector<parking_interface::msg::Parkinglst> lst1 = rad->parking;
    vector<parking_interface::msg::Parkinglst> lst2 = img->parking;
    vector<parking_interface::msg::Parkinglst> lst_res;
    //clog<<"lst1 size"<<lst1.size()<<";lst2 size"<<lst2.size()<<endl;
    if(lst1.size() == 0 || lst2.size() == 0){
   
      clog<<"lst_res is empty"<<endl;
    }
    else{
    //LIST_RESULT deault_zero = {0,0,0,0,0,0,0,0,0,0};
    for(int i = 0; i < lst1.size(); i++){
        for(int j = 0; j < lst2.size(); j++){
            //RCLCPP_INFO(rclcpp::get_lo/* gger("rclcpp"), "Begin to fuse,enter loop");
            parking_interface::msg::Parkinglst rad_box = lst1[i];
            parking_interface::msg::Parkinglst img_box = lst2[j];
            // 1. img_box在rad_box内
            if(img_box.x1 >= rad_box.x1 && img_box.y1 >= rad_box.y1
            && img_box.x4 <= rad_box.x4 && img_box.y4 <= rad_box.y4){
              //  clog<<"img_box confidence:" <<img_box.confidence<<endl;
                // 1.1 confidence >= 0.8
                //if(img_box.confidence >= DETECT_THRESH){
                    lst_res.push_back(img_box);
                    //clog<<"img_box"<<img_box.x1<<img_box.y1<<img_box.x2<<img_box.y2<<endl;
                    //clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;

                //}
                 // 1.2 confidence <= 0.8slot id:1
                // else{
                //    // clog<<"no matched point1"<<endl;
                //     continue;
                // }
            }
            //2. imgbox中有障碍物
           else if((rad_box.x1 >= img_box.x1 && rad_box.y1 >= img_box.y1
            && rad_box.x4 <= img_box.x4 && rad_box.y4 <= img_box.y4) && (calculate_iou(rad_box, img_box) >= 0.9)){
                 lst_res.push_back(img_box);
            }
            // 3. img_box与rad_box有部分区域相交或者无区域相交
            else{
                float cur_iou = calculate_iou(rad_box, img_box);
                clog<<"iou: "<<cur_iou<<endl;
                // 2.2 IOU >= thresh && confidence >= 0.8
                if(cur_iou >= IOU_THRESH){
                    lst_res.push_back(img_box);
                }
                else{
                   // clog<<"no matched point2"<<endl;
                    continue;
                }
            }
        }
        }
    }
    //clog<<"lst_res"<<lst_res.at(0).x1<<lst_res.at(0).y1<<lst_res.at(0).x2<<lst_res.at(0).y2<<endl;
    fused_frame.parking = lst_res;
    //pub_fused_parking->publish(fused_frame);
    cout<<"fused_parking num :" << fused_frame.parking.size()<<endl;
    //clog<<"fused frame data: "<<fused_frame.header.stamp;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fused frame publishing...");
    //clog<<"fused frame publishing..."<<endl;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::str(fused_frame.header.stamp));
}
*/

void GetLatestFrames()
{
    //clog<<"time stamp begin!"<<endl;

    match_radar = radDeque.back();
    long timestamp = match_radar->header.stamp.sec * (10 ^ 9) + match_radar -> header.stamp.nanosec;
    //clog<<"timestamp"<<timestamp<<endl;
    int index = 0;
    long MIN_DIF = 6666666666666;
    long dif;
    for(int i = 0; i < imgDeque.size(); i++){
        //clog<<"begin find image frame to match"<<endl;
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
    
    //clog<<"find it!!!"<<endl;
    //clog<<"dif = "<<dif/(10^9)<<endl;
    match_image = imgDeque.at(index);

    //提取要融合的两帧后将他们从buffer中删除
    //radDeque.pop_back();
    //imgDeque.erase(imgDeque.begin()+index, imgDeque.begin()+index+1);
}



void radar_callback(const parking_interface::msg::Parking::SharedPtr rad_msg)
{
    auto starttime = system_clock::now();
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
        		duration<double> diff = system_clock::now()- starttime;
        cout << "consuming time：" << diff.count() << "s" << endl;
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