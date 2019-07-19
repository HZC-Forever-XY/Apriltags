#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagSelected.h"
#include <geometry_msgs/Pose.h>
#include "apriltag_ros/continuous_detector.h"
#include <pluginlib/class_list_macros.h>

class SelectedOutput
{
public:
  SelectedOutput(){
    pub = n.advertise<apriltag_ros::AprilTagSelected>("/detected_camera", 1);
    //Topic you want to subscribe
    sub1 = n.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1, &SelectedOutput::detectionCallback1, this);
    sub2 = n.subscribe<apriltag_ros::AprilTagDetectionArray>("my_tag_detections", 1, &SelectedOutput::detectionCallback2, this);
    ROS_INFO("fsfs");
  }//在这里定义了一个节点，分别订阅了两个话题，对应两个摄像头的侦测信息，回调函数就是检测是否识别到了二维码。
 //只要定义detected_camera就好，如果里面的数据成员which_one=1就是左边摄像头读到信息，如果为2就是右边摄像头读到了信息。
  void detectionCallback1(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  { 
    //apriltag_ros::AprilTagSelected result_msg;
    if(!msg->detections.empty()){
      ROS_INFO("The left camera detected the apriltag.");
      result_msg.which_one=1;
      result_msg.detections=msg->detections;
      pub.publish(result_msg);
    }
  }

  void detectionCallback2(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
    //apriltag_ros::AprilTagSelected result_msg;
    if(!msg->detections.empty()){
      ROS_INFO("The right camera detected the apriltag.");
      result_msg.which_one=2;
      result_msg.detections=msg->detections;
      pub.publish(result_msg);
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub1, sub2;
  apriltag_ros::AprilTagSelected result_msg;
};
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "selected_output");

  SelectedOutput SeOut;
 
  ros::spin();
 
  return 0;
}