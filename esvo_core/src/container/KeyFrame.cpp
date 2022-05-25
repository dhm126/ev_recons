#include <esvo_core/container/KeyFrame.h> 

namespace esvo_core{
namespace container{

int mnid=0;
KeyFrame::KeyFrame(){
 this->kfid_=mnid++;
 this->ts_=ros::Time::now();
 this->kfT_w_c=Eigen::Matrix<double,4,4>::Identity();
 this->vkfdps_.clear();
}    

KeyFrame::KeyFrame(const Eigen::Matrix4d & Twc){    
 this->kfid_=mnid++;
 this->ts_=ros::Time::now();
 this->kfT_w_c=Twc;    
}

KeyFrame::~KeyFrame(){};

void KeyFrame::ImageLog(){
            if (this->curTsFrame_.empty()) {
                ROS_INFO("%d this keyframe dont have tsobs info ",this->kfid_);
                return;
            }else
            {
              std::stringstream ss;
              ss.str(std::string());
              ss << "/tmp/kfimgelog" << std::setfill('0') << std::setw(1) <<this->kfid_<< ".png";
              cv::imwrite(ss.str(),this->curTsFrame_);
            }
    }

int KeyFrame::getkfMapszie(){
    
    return this->vkfdps_.size();
}

Eigen::Matrix3d KeyFrame::getRotationMatrix() const{
    Eigen::Matrix3d R;
    R<<this->kfT_w_c(0,0),this->kfT_w_c(0,1),this->kfT_w_c(0,2),
    this->kfT_w_c(1,0),this->kfT_w_c(1,1),this->kfT_w_c(1,2),
    this->kfT_w_c(2,0),this->kfT_w_c(2,1),this->kfT_w_c(2,2);
    
    return R;
}

Eigen::Vector3d KeyFrame::getTranslation() const {
    Eigen::Vector3d t;
    t<<this->kfT_w_c(3,0),this->kfT_w_c(3,1),this->kfT_w_c(3,2);
    return t;


}

void KeyFrame::setTransformation(const Eigen::Matrix4d &Twc){

this->kfT_w_c=Twc;
}
bool KeyFrame::needKeyFrame(Eigen::Matrix4d &tcw,const Eigen::Matrix4d &last_tcw){
    double diff=(tcw.topRightCorner<3,1>() - last_tcw.topRightCorner<3,1>()).norm();
 if(diff>0.6) return true;
 else return false;

}


}//namespace container
}//namespace esvo_core