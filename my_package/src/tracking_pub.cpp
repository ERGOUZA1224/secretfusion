#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
#include <deque>
#include <vector>

#include <iostream>
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

#define BUFFER_MAX_VALUE 60;
rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_tracked;
std::deque<parking_interface::msg::Parking> fused_buffer;



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

// global variables for counting
#define CNUM 20
parking_interface::msg::Parking tracked_parking;
vector<TrackingBox> detData;
vector<KalmanTracker> trackers;
int maxFrame = 1;
HungarianAlgorithm HungAlgo;
int frame_i = 1;

void topic_callback(const parking_interface::msg::Parking::SharedPtr fused_msg)
{
	auto starttime = system_clock::now();
	detData.clear();
    //darknet_ros_msgs::BoundingBoxes bboxes;
   // bboxes.bounding_boxes = msg->bounding_boxes;
   parking_interface::msg::Parking pparking;
   pparking.parking = fused_msg -> parking;
    // 1. read bounding boxes from object detector, here from YOLO v3 ROS version.
    for (int i = 0; i < int(pparking.parking.size()) ; i++)
    {
        TrackingBox tb;
        tb.frame = frame_i;
        tb.id = pparking.parking[i].slotid;
        tb.box = Rect_<float>(Point_<float>(pparking.parking[i].x1, pparking.parking[i].y1), Point_<float>(pparking.parking[i].x4, pparking.parking[i].y4));
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
    vector<parking_interface::msg::Parkinglst> parkinglst;
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
	vector<parking_interface::msg::Parkinglst> lst1;
	
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
				parking_interface::msg::Parkinglst reslst;
				//cout<<"1 id:"<< res.id<<endl;
				reslst.slotid = res.id;
				reslst.confidence = 1;
				reslst.x1 = res.box.x;
				reslst.y1 = res.box.y;
				reslst.x2 = res.box.x + res.box.height;
				reslst.y2 = res.box.y;
				reslst.x3 = res.box.x;
				reslst.y3 = res.box.y + res.box.width;
				reslst.x4 = res.box.x + res.box.height;
				reslst.y4 = res.box.y + res.box.width;
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
		tracked_parking.parking = lst1;
		cycle_time = (double)(getTickCount() - start_time);
        double fps = (1.0/cycle_time)*getTickFrequency();
		pub_tracked -> publish(tracked_parking);

		cout<<"tracker size: "<<trackers.size()<<endl;
		for(int i = 0; i < tracked_parking.parking.size(); i++){
			
			cout<<"slot id:"<<(int)tracked_parking.parking[i].slotid<<endl;
			cout<<"coord:"<<tracked_parking.parking[i].x1<<" "<<tracked_parking.parking[i].y1<<endl;
		}
		tracked_parking.parking.clear();
		//cout<<"tracked_parking id: "<<tracked_parking.parking.back().slotid<<endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current : %.1f", fps);
		duration<double> diff = system_clock::now()- starttime;
        cout << "consuming time：" << diff.count() << "s" << endl;
	}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("tracking_pub");
  auto sub_fused= n->create_subscription<parking_interface::msg::Parking>("fused_parking", 100,  topic_callback);
  pub_tracked = n->create_publisher<parking_interface::msg::Parking>("tracked_parking", 100);
  
  //cout<<"tracked_parking id: "<<tracked_parking.parking.back().slotid<<endl;
  
  rclcpp::spin(n); 
  return 0;
}