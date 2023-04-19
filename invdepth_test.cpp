#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// #include <>

void depthImageCallback(const sensor_msgs::ImageConstPtr &msg){
cv_bridge::CvImagePtr depthImagePtr_;
try{
    depthImagePtr_=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
}catch(const cv_bridge::Exception & e  ){
    ROS_ERROR(" %s",e.what());
}
}
int main(){
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ros::Subscriber depth_image_sub_=nh.subscribe("/davis/left/depth_image_rect",1,&depthImageCallback);
    ros::spin();
return 0;
}