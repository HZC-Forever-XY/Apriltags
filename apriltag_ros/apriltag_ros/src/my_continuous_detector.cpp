#include "apriltag_ros/my_continuous_detector.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::My_ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{

My_ContinuousDetector::My_ContinuousDetector ()
{
}

void My_ContinuousDetector::onInit ()
{//初始化函数
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  /*if(draw_tag_detections_image_==1)
   ROS_INFO("right");
   else
   ROS_INFO("flase");*/
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &My_ContinuousDetector::imageCallback, this);//订阅了来自摄像头的消息
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("my_tag_detections", 1);//发布了消息（这就是扫描二维码的结果）
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }
}

void My_ContinuousDetector::imageCallback (//这个函数将会重复运行（摄像头正常工作的话）
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);//将摄像头发布消息转换为opencv所用的数据类型格式。
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(
      tag_detector_->detectTags(cv_image_,camera_info));//将消息发布出去，在common_functions函数里会有对其的操作（识别二维码）
     // if (cv_image_->image.rows > 60 && cv_image_->image.cols > 60);
     //新的内容

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    //cv::imwrite("fssf.jpg",cv_image_->image);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }//没有东西去订阅这个话题，这只是单纯发布了他
}

} // namespace apriltag_ros
