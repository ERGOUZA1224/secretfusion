#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "parking_interface/msg/parking.hpp"
#include "parking_interface/msg/parkinglst.hpp"
using std::placeholders::_1;

#define BUFFER_MAX_VALUE 60;
rclcpp::Publisher<parking_interface::msg::Parking>::SharedPtr pub_fused;
std::deque<parking_interface::msg::Parking> fused_buffer;
int slotID = 1;

//IOU
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

void topic_callback(const parking_interface::msg::Parking::SharedPtr fused_msg){
  parking_interface::msg::Parking tracked_parking;

  //init parking lot number from 1
  if(fused_buffer.size() == 0){
    for(int i = 0; i < fused_msg.parking.size(); i++){
        fused_msg.parking.at(i).slotID = slotID;
        slotID = slotID++;
    }
    tracked_parking = fused_msg;
  }

  //receive fused frame
  if(fused_buffer.size() < BUFFER_MAX_VALUE){
        fused_buffer.push_back(fused_msg);
    }
    else{
        fused_buffer.pop_front();
        fused_buffer.push_back(fused_msg);
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

  //compare with lastest frame
  if(fused_buffer.size() >= 2){
    parking_interface::msg::Parking frame1 = fused_buffer.pop_back();
    parking_interface::msg::Parking frame2 = fused_buffer.pop_back(); //frame2 is tracked
    vector<parking_interface::msg::Parkinglst> lst1 = frame1->parking;
    vector<parking_interface::msg::Parkinglst> lst2 = frame2->parking;
    for(int i = 0; i < lst1.size(); i++){
        for(int j = 0; j < lst2.size(); j++){
          parking_interface::msg::Parkinglst new_box = lst1[i];
          parking_interface::msg::Parkinglst old_box = lst2[j];
          float cur_iou = calculate_iou(new_box, old_box);
          
      }
    }
  }




  pub_fused -> publish(tracked_parking);

}


void updateID(parking_interface::msg::Parkinglst parkinglst){

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("tracking_pub");
  auto sub_fused= n->create_subscription<parking_interface::msg::Parking>("fused_parking", 100,  topic_callback);
  rclcpp::spin(n); 
  return 0;
}