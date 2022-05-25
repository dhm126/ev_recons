#ifndef ESVO_CORE_CONTAINER_KEYFRAME_H
#define ESVO_CORE_CONTAINER_KEYFRAME_H
#include <dvs_msgs/Event.h>
#include <vector>
#include <Eigen/Eigen>
#include <map>
#include <esvo_core/container/DepthPoint.h>
#include <esvo_core/container/CameraSystem.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <g2o/types/slam3d/se3quat.h>

typedef std::pair<int ,int > coord;

namespace  esvo_core{
namespace  container{
class DepthPoint;

class KeyFrame{

    public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    
    KeyFrame();
    KeyFrame(const Eigen::Matrix<double,4,4> & Twc);
    KeyFrame(const size_t &idx,
             const ros::Time &ts,
             const Eigen::Matrix<double,4,4> & Twc,
             const std::vector<Eigen::Vector3d> & mappoints);
    // KeyFrame(const cv::Mat & TSOBS);
    virtual ~KeyFrame();

    void ImageLog();
     
    
    int getkfMapszie();

    void setTransformation(const Eigen::Matrix4d & Tcw);
    g2o::SE3Quat convert2SE3Quart(const Eigen::Matrix4d & T_c_w);
    Eigen::Matrix3d getRotationMatrix() const;
    Eigen::Vector3d getTranslation() const;
    bool needKeyFrame( Eigen::Matrix4d &tcw,const Eigen::Matrix4d &last_tcw);
    
    //接口太多了先写成public的
    public:
    //points coordinate and world 3d position
    std::vector <std::pair<coord,Eigen::Vector3d>> vkfdps_;
    
    //current timestamp
    ros::Time ts_;
    //current wolrd pose 
    Eigen::Matrix<double,4,4> kfT_w_c;
    //keyframe id
    size_t kfid_;
    //current timesurface observations
    cv::Mat curTsFrame_;
    //covisibility connection
    std::pair<int,int> radius_;
    //depthPoint 
    std::vector<DepthPoint> kfvdp_;
    //covisibility connection  int =kfid
    std::map<DepthPoint*,std::vector<int>> nieba_;
    //using for loopclosing reconstruction 
    private:
    std::vector<dvs_msgs::Event> vkfEvents_;
    
        };
    }

}
#endif 