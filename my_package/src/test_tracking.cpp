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
#include <Eigen/Dense>
#include <functional>
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglist.hpp"
#include "parking_interface/msg/point2_d.hpp"
#include "parkingslot_interfaces/msg/parking_slot_array.hpp"
#include "parkingslot_interfaces/msg/corner_detection_out.hpp"
#include "parkingslot_interfaces/msg/point2_d.hpp"
#include <chrono>   //计算时间
#include <memory>
#include <iomanip> // to format image names using setw() and setfill()
#include <unistd.h>
#include <chrono>
#include <set>
#include <climits>
#include <cfloat>
#include "Hungarian.h"
#include "KalmanTracker.h"
#include "opencv2/video/tracking.hpp"



using std::placeholders::_1;
using namespace std::chrono;
using namespace std;
using namespace cv;
using namespace Eigen;

#define BUFFER_MAX_VALUE 100
#define DETECT_THRESH 0.8
#define IOU_THRESH 0.7
#define CNUM 20
#define   BEV_RESOLUTION       (30) 
#define   BASELINK_CENTER_X    (13)
#define   BASELINK_CENTER_Y    (21.1)
#define PI 3.14159265

typedef struct TrackingBox
{
	int frame;
	int id;
	Rect_<float> box;
}TrackingBox;

//垂直/水平车位
typedef struct {
    float  x_min;
    float y_min;
    float x_max;
    float y_max;
} box;


int slotID = 1;
int maxFrame = 1;
int frame_i = 1;
int count_ = 1;
vector<TrackingBox> detData;
vector<KalmanTracker> trackers;
HungarianAlgorithm HungAlgo;
std::deque<parking_interface::msg::Parking> fused_buffer;
parking_interface::msg::Parking tracked_parking;
rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused_parking;
std::deque<parking_interface::msg::Parking::SharedPtr> radDeque;
std::deque<parkingslot_interfaces::msg::ParkingSlotArray::SharedPtr> imgDeque;
parking_interface::msg::Parking::SharedPtr match_radar;
parkingslot_interfaces::msg::ParkingSlotArray::SharedPtr match_image;


/*************************************************************************************************************
 * Funciton Name:  GetLatestFrames
 * Descripiton:        match timestamp
 * Input:                     
 * Output:                 
 * Return： void
 * Other:     
**************************************************************************************************************/
void GetLatestFrames()
{
    //clog<<"time stamp begin!"<<endl;
    match_radar = radDeque.back();
    long timestamp = match_radar->header.stamp.sec * (10 ^ 9) + match_radar -> header.stamp.nanosec;
    //clog<<"timestamp"<<timestamp<<endl;
    int index = 0;
    long MIN_DIF = LONG_MAX;
    long dif;
    for(unsigned int i = 0; i < imgDeque.size(); i++)
    {
        //clog<<"begin find image frame to match"<<endl;
        long imgstamp = imgDeque.at(i)->header.stamp.sec * (10 ^ 9) + imgDeque.at(i)->header.stamp.nanosec; 
        dif = abs(timestamp - imgstamp);      
        if(dif == 0)
        {
         index = i;
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "find matched image");
          match_image = imgDeque.at(index);
         return;
        }
        else
        {
            if(dif < MIN_DIF)
            {
                index = i;
                MIN_DIF = dif;
            }
        }
    }
    match_image = imgDeque.at(index);
}


/*************************************************************************************************************
 * Funciton Name:  GetIOU
 * Descripiton: Function for calculate IOU  for tracking   
 * Input:                     
 * Output:                 
 * Return：
 * Other:     
**************************************************************************************************************/
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

/*************************************************************************************************************
 * Funciton Name:  box_iou
 * Descripiton:   calculate iou of two boxes       
 * Input:    box                 
 * Output:   iou of two boxes             
 * Return：  float
 * Other:     
**************************************************************************************************************/
float box_iou(const box& box1, const box& box2) 
{
    float w = std::max(std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min), 0.f);
    float h = std::max(std::min(box1.y_max, box2.y_max) - std::max(box1.y_min, box2.y_min), 0.f);
    float iou = w * h / ((box1.x_max - box1.x_min) * (box1.y_max - box1.y_min)  +
                                         (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) - w * h);
    return iou;
}

/*************************************************************************************************************
 * Funciton Name:  calculate_imgiou
 * Descripiton:       用于计算相交部分占img_box的IOU     
 * Input:    Parkinglist, CornerDetectionOut                 
 * Output:   intersection of area of image data             
 * Return：  float
 * Other:     
**************************************************************************************************************/
float calculate_imgiou(parking_interface::msg::Parkinglist rad_box, parkingslot_interfaces::msg::CornerDetectionOut img_box)
{
    box box1;
    box box2;
    box1 = {(float)rad_box.pointlist[0].x, (float)rad_box.pointlist[0].y, (float)rad_box.pointlist[2].x,  (float)rad_box.pointlist[2].y};
    box2 = {(float)img_box.corner_point[0].x, (float)img_box.corner_point[0].y, (float)img_box.corner_point[2].x, (float)img_box.corner_point[2].y};
    float w = std::max(std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min), 0.f);
    float h = std::max(std::min(box1.y_max, box2.y_max) - std::max(box1.y_min, box2.y_min), 0.f);
    float iou = w * h /  ((box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) );
    return iou;
}

/*************************************************************************************************************
 * Funciton Name:  BevTransformBaseLink_Eigen
 * Descripiton:          Bev坐标系转化为车身坐标系
 * Input:       Point2D of Bev position              
 * Output:      Point2D of Baselink position           
 * Return：
 * Other:     
**************************************************************************************************************/
parkingslot_interfaces::msg::Point2D BevTransformBaseLink_Eigen(parkingslot_interfaces::msg::Point2D BevPosition)
{
    parkingslot_interfaces::msg::Point2D BaseLinkPosition;
    BaseLinkPosition.x = BevPosition.x  /  BEV_RESOLUTION  -  BASELINK_CENTER_X;
    BaseLinkPosition.y = BASELINK_CENTER_Y-  BevPosition.y  / BEV_RESOLUTION ;
    return BaseLinkPosition;
}

/*************************************************************************************************************
 * Funciton Name:  BaseLinkTransformMap_Eigen
 * Descripiton:         车身坐标系转换为map坐标系
 * Input:      Point2D of Base Link Position            
 * Output:     Point2D of Map position            
 * Return：    Point2D
 * Other:    参考 https://blog.csdn.net/zyq880625/article/details/127739314
**************************************************************************************************************/
parking_interface::msg::Point2D BaseLinkTransformMap_Eigen(parking_interface::msg::Point2D SensorBaseLinkPosition)
{
    Vector3d BaseLinkMapPosition;
    BaseLinkMapPosition[0] = 0;
    BaseLinkMapPosition[1] = 0;
    BaseLinkMapPosition[2] = 0;

	Matrix<double, 2, 2> RemotMatrix;
  
    float threa  =  BaseLinkMapPosition[2]  * PI   / 180;
	RemotMatrix  <<  cos(threa) ,  -sin(threa),
					                            sin(threa) ,  cos(threa);

    Vector2d MapPosition(BaseLinkMapPosition[0], BaseLinkMapPosition[1]);
    Vector2d BaseLinkPosition_2d(SensorBaseLinkPosition.x, SensorBaseLinkPosition.y);
    Vector2d ResultMapPosition  =    RemotMatrix  *  BaseLinkPosition_2d + MapPosition ;

    parking_interface::msg::Point2D SensorMapPosition;
    SensorMapPosition.x =  ResultMapPosition[0] ;
    SensorMapPosition.y=  ResultMapPosition[1] ;
    return SensorMapPosition;

}

/*************************************************************************************************************
 * Funciton Name:  calculate_area
 * Descripiton:          calculate area of radar data
 * Input:      Parkinglist               
 * Output:     area of radar data            
 * Return：     float
 * Other:     
**************************************************************************************************************/
float calculate_area(parking_interface::msg::Parkinglist rad_box)
{
    box box2;
    box2 = {(float)rad_box.pointlist[0].x, (float)rad_box.pointlist[0].y, (float)rad_box.pointlist[2].x, (float)rad_box.pointlist[2].y};
    float s = (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) ;
    return s;
}

/*************************************************************************************************************
 * Funciton Name:  calculate_area
 * Descripiton:         calculate area of image data
 * Input:      CornerDetectionout               
 * Output:     area of image data          
 * Return： float
 * Other:     
**************************************************************************************************************/
float calculate_area(parkingslot_interfaces::msg::CornerDetectionOut img_box)
{
    box box2;
    box2 = {(float)img_box.corner_point[0].x, (float)img_box.corner_point[0].y, (float)img_box.corner_point[2].x, (float)img_box.corner_point[2].y};
    float s = (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min) ;
    return s;
}

/*************************************************************************************************************
 * Funciton Name:  calculate_iou
 * Descripiton:         
 * Input:  Parkinglist, CornerDetectionout                   
 * Output: value of iou                
 * Return：float
 * Other:     
**************************************************************************************************************/
float calculate_iou(parking_interface::msg::Parkinglist rad_box, parkingslot_interfaces::msg::CornerDetectionOut img_box){
    box box1;
    box box2;
    box1 = {(float)rad_box.pointlist[0].x, (float)rad_box.pointlist[0].y, (float)rad_box.pointlist[2].x, (float)rad_box.pointlist[2].y};
    box2 = {(float)img_box.corner_point[0].x, (float)img_box.corner_point[0].y, (float)img_box.corner_point[2].x, (float)img_box.corner_point[2].y};
    //计算iou
    float iou = box_iou(box1, box2);
    return iou;
}

/*************************************************************************************************************
 * Funciton Name:  tracking_callback
 * Descripiton:   function for tracking     
 * Input:   Parking without slot id                  
 * Output:  Parking with slot id               
 * Return：
 * Other:     
*************************************************************************************************************/
void tracking_callback(const parking_interface::msg::Parking fused_msg)
{
	detData.clear();
    cout<<"begin tracking"<<endl;
   parking_interface::msg::Parking pparking;
   pparking.parkinglist = fused_msg.parkinglist;
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

	// main loop
	for (int fi = 0; fi < maxFrame; fi++)
	{
		frame_count++;
		assignment.clear();
		unmatchedTrajectories.clear();
		unmatchedDetections.clear();
		allItems.clear();
		matchedItems.clear();
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
				//TrackingBox tb = detFrameData[fi][id];
			}
			
			continue;
		}
        // cout<<"trackers size "<<trackers.size()<<endl;
		///////////////////////////////////////
		// 3.1. get predicted locations from existing trackers.
		predictedBoxes.clear(); 
		//cout<<"step 3.1 begin"<<endl;
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			Rect_<float> pBox = (*it).predict();
            // cout<<"pBox "<<pBox.x<<" "<<pBox.y<<endl;
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
        {

        }
    

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
		for (auto umd : unmatchedDetections)
        {
			cout << "unmatchedDetections" << endl;
			cout << "i = " << i << " , value = " << umd << endl;
			i ++ ;
		}

		i = 0;
		for (auto umd : unmatchedTrajectories)
        {
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
		
		tracked_parking.parkinglist = lst1;
		cycle_time = (double)(getTickCount() - start_time);
        double fps = (1.0/cycle_time)*getTickFrequency();
		pub_fused_parking -> publish(tracked_parking);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fused frame publishing...");
		cout<<"tracker size: "<<trackers.size()<<endl;
		for(unsigned int i = 0; i < tracked_parking.parkinglist.size(); i++)
        {
			
			cout<<"slot id:"<<(int)tracked_parking.parkinglist[i].id<<endl;
		}
		tracked_parking.parkinglist.clear();
		//cout<<"tracked_parking id: "<<tracked_parking.parking.back().slotid<<endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current : %.1f", fps);

	}
}


/*************************************************************************************************************
 * Funciton Name:  publish_fused_parking_only_image
 * Descripiton:    when fusion module only subscribe image parking slot data.     
 * Input:    ParkingSlotArray                 
 * Output:   Parking              
 * Return：
 * Other:     
**************************************************************************************************************/

void publish_fused_parking_only_image(parkingslot_interfaces::msg::ParkingSlotArray::SharedPtr img)
{
    parking_interface::msg::Parking fused_frame;
    
    fused_frame.header.stamp= img->header.stamp;
    fused_frame.header.frame_id = "map";

    vector<parkingslot_interfaces::msg::CornerDetectionOut> lst2 = img->parking_detection_out_list;
    vector<parking_interface::msg::Parkinglist> lst_res;

    for( unsigned int j = 0; j < lst2.size(); j++)
    {
            parkingslot_interfaces::msg::CornerDetectionOut img_box = lst2[j];
             parking_interface::msg::Parkinglist lst;
            for(unsigned int i = 0; i < 4 ; i++)
            {
                parking_interface::msg::Point2D pt1;
                pt1.x = img_box.corner_point[i].x;
                pt1.y = img_box.corner_point[i].y;
                lst.pointlist.push_back(pt1);
            }
            lst_res.push_back(lst);
    }

    fused_frame.parkinglist = lst_res;
    pub_fused_parking -> publish(fused_frame);
    //tracking_callback(fused_frame);
}

/*************************************************************************************************************
 * Funciton Name:  publish_fused_parking
 * Descripiton:  fusion module subscribe image data and radar data.       
 * Input:   ParkingSlotArray, Parking                  
 * Output:  Parking              
 * Return： void
 * Other:     
**************************************************************************************************************/
void publish_fused_parking(parkingslot_interfaces::msg::ParkingSlotArray::SharedPtr img, parking_interface::msg::Parking::SharedPtr rad)
{
    parking_interface::msg::Parking fused_frame;
    
    fused_frame.header.stamp= img->header.stamp;
    fused_frame.header.frame_id = "map";

    vector<parking_interface::msg::Parkinglist> lst1 = rad->parkinglist;
    vector<parkingslot_interfaces::msg::CornerDetectionOut> lst2 = img->parking_detection_out_list;
    vector<parking_interface::msg::Parkinglist> lst_res;

    if(lst1.size() == 0 || lst2.size() == 0)
    {
      clog<<"lst_res is empty"<<endl;
      //发布空白数据
      lst_res.clear();
    }
    else
    {
        for(unsigned int i = 0; i < lst1.size(); i++)
        {
            for( unsigned int j = 0; j < lst2.size(); j++)
            {
                parking_interface::msg::Parkinglist rad_box = lst1[i];
                parkingslot_interfaces::msg::CornerDetectionOut img_box = lst2[j];
                //有相交
                if(calculate_iou(rad_box,img_box) > 0)
                {
                    // 1. rad_box > img_box
                    if(calculate_area(rad_box) >= calculate_area(img_box))
                    {
                        // 1.1 img_box刚好在rad_box内
                        if(img_box.corner_point[0].x >= rad_box.pointlist[0].x && img_box.corner_point[0].y >= rad_box.pointlist[0].y
                        && img_box.corner_point[2].x <= rad_box.pointlist[2].x && img_box.corner_point[2].y <= rad_box.pointlist[2].y)
                        {
                            parking_interface::msg::Parkinglist lst;
                            for(unsigned int i = 0; i < 4 ; i++)
                            {
                                parking_interface::msg::Point2D pt1;
                                pt1.x = img_box.corner_point[i].x;
                                pt1.y = img_box.corner_point[i].y;
                                lst.pointlist.push_back(pt1);
                            }
                            lst_res.push_back(lst);
                        }
                        // 1.2 相交且IOU大于0.9，无障碍物区域够停车
                        else if(calculate_imgiou(rad_box, img_box) >= 0.8)
                        {
                            parking_interface::msg::Parkinglist lst;
                            for(unsigned int i = 0; i < 4 ; i++)
                            {
                                parking_interface::msg::Point2D pt1;
                                pt1.x = img_box.corner_point[i].x;
                                pt1.y = img_box.corner_point[i].y;
                                lst.pointlist.push_back(pt1);
                            }
                            lst_res.push_back(lst);

                        }
                        //1.3 无障碍物区域不够停车
                        else
                        {
                           ;
                        }
                    }
                    //2. img_box > rad_box
                    else
                    {
                        //2.1 rad_box刚好在img_box内，空白区域够停车
                        if((rad_box.pointlist[0].x >= img_box.corner_point[0].x && rad_box.pointlist[0].y >= img_box.corner_point[0].y
                        && rad_box.pointlist[2].x <= img_box.corner_point[2].x && rad_box.pointlist[2].y <= img_box.corner_point[2].y) 
                        && (calculate_iou(rad_box, img_box) >= 0.9))
                        {
                             parking_interface::msg::Parkinglist lst;
                            for(unsigned int i = 0; i < 4 ; i++)
                            {
                                parking_interface::msg::Point2D pt1;
                                pt1.x = img_box.corner_point[i].x;
                                pt1.y = img_box.corner_point[i].y;
                                lst.pointlist.push_back(pt1);
                            }
                            lst_res.push_back(lst);
                        }
                        // 2.2 rad_box与img_box相交，IOU大于0.8
                        else if(calculate_imgiou(rad_box, img_box) >= 0.8)
                        {
                             parking_interface::msg::Parkinglist lst;
                            for(unsigned int i = 0; i < 4 ; i++)
                            {
                                parking_interface::msg::Point2D pt1;
                                pt1.x = img_box.corner_point[i].x;
                                pt1.y = img_box.corner_point[i].y;
                                lst.pointlist.push_back(pt1);
                            }
                            lst_res.push_back(lst);

                        }
                        // 2.3 img_box内无空白区域或空白区域不够停车
                        else
                        {
                            ;
                        }
                    } 
                }
                //无相交
                else
                {
                    ;
                }
            }
        }
    }
    fused_frame.parkinglist = lst_res;
    //pub_fused_parking->publish(fused_frame);
    //cout<<"publishinfo to tracking module once"<<endl;
    tracking_callback(fused_frame);
   //cout<<"fused_parking num :" << fused_frame.parkinglist.size()<<endl;
}



//Node to subscribe image and radar data,and publish fused data.
class FuseTracking : public rclcpp::Node
{
  public:
    FuseTracking() : Node("FuseTracking")
    {
      sub_img  = this->create_subscription<parkingslot_interfaces::msg::ParkingSlotArray>(  "image_parking", 10, std::bind(&FuseTracking::image_callback1, this, _1));
      sub_rad  = this->create_subscription<parking_interface::msg::Parking>(  "radar_parking", 10, std::bind(&FuseTracking::radar_callback1, this, _1));

      pub_fused_parking = this->create_publisher<parking_interface::msg::Parking>("fused_parking", 100);
      timer_ = this->create_wall_timer( 500ms, std::bind(&FuseTracking::timer_callback1, this));
    }

   public:
   void image_callback1(const parkingslot_interfaces::msg::ParkingSlotArray::SharedPtr img_msg)
   {
             cout<<"receive image data666"<<endl;
            if(imgDeque.size() < BUFFER_MAX_VALUE)
            {
                imgDeque.push_back(img_msg);
            }
            else
            {
                imgDeque.pop_front();
                imgDeque.push_back(img_msg);
            }

          if(radDeque.size() == 0 && imgDeque.size() >= 1)
           {
              cout<<"receive image data 0000000000"<<endl;
                publish_fused_parking_only_image(img_msg);
           }
          
   }      
   

   public:
   void radar_callback1(const parking_interface::msg::Parking::SharedPtr rad_msg);
   void timer_callback1();

    rclcpp::Subscription<parkingslot_interfaces::msg::ParkingSlotArray>::SharedPtr sub_img;
    rclcpp::Subscription<parking_interface::msg::Parking>::SharedPtr sub_rad;

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    size_t count_;
};




void FuseTracking::radar_callback1(const parking_interface::msg::Parking::SharedPtr rad_msg)
{

    cout<<"receive radar data5555"<<endl;
    auto starttime = system_clock::now();
    if(radDeque.size() < BUFFER_MAX_VALUE)
    {
        radDeque.push_back(rad_msg);
    }
    else
    {
        radDeque.pop_front();
        radDeque.push_back(rad_msg);
    }
    if(radDeque.size() >= 1 && imgDeque.size() >= 1)
    {
        GetLatestFrames();
        publish_fused_parking(match_image, match_radar);
        duration<double> diff = system_clock::now()- starttime;
        cout << "inference consuming time：" << diff.count() << "s" << endl;
    }
}
void FuseTracking::timer_callback1()
{

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FuseTracking>());
  rclcpp::shutdown();
  return 0;
}
