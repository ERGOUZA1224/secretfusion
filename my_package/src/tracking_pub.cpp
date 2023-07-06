#include <fstream>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "YOLOv5Detector.h"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
using std::placeholders::_1;
#include "FeatureTensor.h"
#include "BYTETracker.h" //bytetrack
#include "tracker.h"//deepsort
//Deep SORT parameter

const int nn_budget=100;
const float max_cosine_distance=0.2;
rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_tracked;

/*void get_detections(DETECTBOX box,float confidence,DETECTIONS& d)
{
    DETECTION_ROW tmpRow;
    tmpRow.tlwh = box;//DETECTBOX(x, y, w, h);

    tmpRow.confidence = confidence;
    d.push_back(tmpRow);
}*/

parking_interface::msg:Parking test_bytetrack(std::vector<detect_result>& results,BYTETracker& tracker)
{
    //车位信息转换为detect results
    /*std::vector<detect_result> results;
    for(int i = 0; i < fused.size(); i++){
        detect_result dec1;
        dec1.classId = 0;
        dec1.confidence = fused[i].confidence;
        cv::Rect_<float> box;
        box.x = fused[i].x1;
        box.y = fused[i].y1;
        box.width = fused[i].x4 - fused[i].x1;
        box.height = fused[i].y4 - fused[i].y1;
        dec1.box = box;
        results.pushback(dec1);
    }*/

    std::vector<STrack> output_stracks = tracker.update(results);
    std::vector<parking_interface::msg:Parkinglst> tracked_lst;
    parking_interface::msg::Parking tracked_frame;
    for (unsigned long i = 0; i < output_stracks.size(); i++)
    {
        std::vector<float> tlwh = output_stracks[i].tlwh;
        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
            //cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
            //cv::putText(frame, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5),
            //        0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            //cv::rectangle(frame, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
            tracked_lst.x1 = tlwh[0];
            tracked_lst.y1 = tlwh[1];
            tracked_lst.x2 = tlwh[0] + tlwh[2];
            tracked_lst.y2 = tlwh[1];
            tracked_lst.x3 = tlwh[2];
            tracked_lst.y3 = tlwh[3] + tlwh[3];
            tracked_lst.x4 = tlwh[0] + tlwh[2];
            tracked_lst.y4 = tlwh[1] + tlwh[3];
            tracked_lst.slotID  = output_stracks[i].track_id;
        }
        tracked_frame.parking.push_back(tracked_lst);
    }
    return tracked_frame;
}

void topic_callback(const parking_interface::msg::Parking::SharedPtr fused_msg){

    parking_interface::msg::Parking tracked;
    //bytetrack
    int fps=20;
    BYTETracker bytetracker(fps, 30);
    std::vector<parking_interface::msg:Parkinglst> fused = fused_msg -> parking;
    std::vector<detect_result> results;
    for(int i = 0; i < fused.size(); i++){
        detect_result dec1;
        dec1.classId = 0;
        dec1.confidence = fused[i].confidence;
        cv::Rect_<float> box;
        box.x = fused[i].x1;
        box.y = fused[i].y1;
        box.width = fused[i].x4 - fused[i].x1;
        box.height = fused[i].y4 - fused[i].y1;
        dec1.box = box;
        results.pushback(dec1);
    }
    tracked = test_bytetrack(results,bytetracker);
    tracked.header = fused_msg->header;
    pub_tracked -> publish(tracked);
}


/*int main(int argc, char *argv[])
{
    //bytetrack
    int fps=20;
    BYTETracker bytetracker(fps, 30);
    std::vector<detect_result> results;
    test_bytetrack(frame, results,bytetracker);
    capture.release();
    video.release();
}*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("tracking_pub");
  auto sub_fused= n->create_subscription<parking_interface::msg::Parking>("fused_parking", 100,  topic_callback);
  pub_tracked = n->create_publisher<parking_interface::msg::Parking>("tracked_parking", 100);
  rclcpp::spin(n); 
  return 0;
}