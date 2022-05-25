#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <esvo_core/loopdetector.h>

#include <dvs_msgs/Event.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <esvo_core/tools/fsolver.h>

#include <thread>

using namespace std;
using namespace esvo_core;

dl::dl(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_private):
  nh_(nh),
  pnh_(nh_private),
  //calibFile_(tools::param(pnh_,"calibinfoFIle",std::string(""))),
  //camSYSPtr_(new CameraSystem(calibFile_,false )),
  my_voc_("/home/zhouyum/recon/src/dvs_mosaic/src/tsvoc.yml.gz"),
  my_database_(my_voc_,5,true)
//  fsolver_(346,240)
//camsysPtr
{
  
  //voc_file="/home/zhouyum/recon/src/dvs_mosaic/src/tsvoc.yml.gz";

  //my_voc_(voc_file);
  //DBoW3::Database my_database_(my_voc_,5,true);
  

  std::vector<cv::Mat> images_;
  //change topic name in  need 
  acumulatedEvents_sub_=nh_.subscribe("/davis/left/image_raw",1,&dl::accumulatedEventCallback,this);

  image_transport::ImageTransport it_(nh_);
  image_pub=it_.advertise("/imageInLoopcandidates",0);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/loop_closing/pose_pub", 1);//save pose pub 

  camera_Matrix_=cv::Mat::eye(3,3,CV_64F);
  // cv::Mat cameraMatrix=cv::Mat::zeros(3,3,CV_64FC1);
  camera_Matrix_.at<double>(0,0)=226.38018519795807;
  camera_Matrix_.at<double>(1,1)=226.15002947047415;
  camera_Matrix_.at<double>(0,2)=173.6470807871759;
  camera_Matrix_.at<double>(2,0)=133.73271487507847;
  // camera_Matrix_.at<float>(2,2)=1.;
  ROS_INFO("statring loopdetecor");

}

dl::~dl(){
  image_pub.shutdown();
}

void dl::accumulatedEventCallback(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr img ;
  try
  {
      img=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
  }
  catch(const cv_bridge::Exception& e)
  {
  
    ROS_ERROR("cv_bridege get exception  %s \n", e.what());  
  } 
  // img->image.convertTo(img->image,CV_8UC1);
  images_.push_back(img->image);

  // ROS_INFO("LOADING IMAGE number %d ! ",NUM_IMAGE);

   if(img->image.empty()) ROS_ERROR("CANNOT GET IMAGES ");

   DetectionResult Dresult;
   std::vector<cv::KeyPoint> kps_;
   cv::Mat desc_;

   extract(img->image,kps_,desc_);
   
   int size_lc=images_.size()-1;
   
   ros::Time t_lc=ros::Time::now();
   
   mSudaro_.insert(std::make_pair(size_lc,t_lc));
   
   bool detection=loopdetect(kps_,desc_,Dresult);
   
   if(detection){
      cv::Mat outImageQ,outImageM;
      Eigen::Matrix4d t_rel;
      t_rel = calculateRelativePose(m_image_keys[Dresult.match],m_image_keys[Dresult.query]);  
       //std::vector<cv::DMatch> fmatches; 
       //cv::Ptr<cv::DescriptorMatcher> matcher ;//=cv::DescriptorMatcher::create()
       //cv::DescriptorMatcher Matcher;
      //  matcher.knnMatch(m_image_descriptors[Dresult.match],m_image_descriptors[Dresult.query],fmatches);
       //matcher->match(m_image_descriptors[Dresult.match],m_image_descriptors[Dresult.query],fmatches);
      //cv::Mat img_matches;
      // cv::drawMatches(images_[Dresult.match],m_image_keys[Dresult.match],images_[Dresult.query],m_image_keys[Dresult.query],fmatches,img_matches);
        
      if( std::fabs (t_rel(0,0) * t_rel(1,1) * t_rel(2,2)-1 )<0.2 )
      {
        // std::cout<<t_rel<<std::endl;
        //  cv::drawKeypoints(images_[Dresult.query],m_image_keys[Dresult.query],outImageQ);
        //  cv::drawKeypoints(images_[Dresult.match],m_image_keys[Dresult.match],outImageM);
      
      
        // std::stringstream ss;
        //  std::stringstream ss2;
        //        ss.str(std::string());
        //        ss << "/tmp/log1140" << std::setfill('0') << std::setw(1) <<Dresult.match<< ".png";
        //        cv::imwrite(ss.str(),outImageQ);
        //        ss2 << "/tmp/log1240" << std::setfill('0') << std::setw(1) <<Dresult.match<< ".png";
        //        cv::imwrite(ss2.str(),outImageM);
      }
      else return;
      // std::cout<<"t_rel =="<<t_rel<<"\t";
       if (t_rel.determinant()<=0) ROS_INFO("maybe we havn't get the full result");
       else {
        ros::Time t_pub=mSudaro_.find(Dresult.match)->second;
        publishPose(t_rel,t_pub);
       }
      
    
  }
  
    while (images_.size()>image_total_length)
    {
        auto it =images_.begin();
        images_.erase(it);
    }   

// publishMap(img->image);
// VLOG(1)<<"publish map \n ";//verbose


}//image callback

/**
 @brief extract image keypoints and compute descriptors with ORB 
*/
void dl::extract(const cv::Mat &img , std::vector<cv::KeyPoint> &kps,  cv::Mat &desc)
{
  cv::Ptr<cv::Feature2D> detector=cv::ORB::create();
  detector->detectAndCompute(img,cv::Mat(),kps,desc); 

  m_image_keys.push_back(kps);
  m_image_descriptors.push_back(desc);
 }

/**
 @brief loopdetect with given keypoints,descriptors to get a query result with id 
*/
bool dl::loopdetect(const std::vector<cv::KeyPoint> &kps ,const cv::Mat &descriptors,DetectionResult &match){

  DBoW3::BowVector bowec;
  DBoW3::FeatureVector fvec; 
  
  DBoW3::EntryId entry_id=my_database_.size();
  match.query=entry_id;
  if(my_voc_.empty()){
  ROS_ERROR("VOCALBULARY CHECK FAILED ");

  }
  my_database_.getVocabulary()->transform(descriptors,bowec);
  
  DBoW3::QueryResults qr;
  int max_id=entry_id;
  my_database_.query(bowec,qr,50,max_id);
  
  my_database_.add(bowec,fvec);
  
  //存在能用的qr
  if(!qr.empty()){
    double scoreof2=1.;
    //double maxScore_=0.;
    scoreof2=my_database_.getVocabulary()->score(bowec,last_bowvec_);
  
    maxScore_=maxScore_>scoreof2?maxScore_:scoreof2;
     
    if (scoreof2>maxScore_*0.7){
      //todo::the lowest score threshold 
     removeLowsocres(qr, double (maxScore_*0.5));
     
     //防止remove掉所有的qr 不过一般不可能
     if (!qr.empty())
      {
       //zhao 匹配到回环殿的mathc id
        match.match=qr[0].Id;

        std::vector<island> islands;
        computeIslands(qr, islands);// 从小到大排列相似图像排列在后面,时间接近的视作一个匹配组
          // this modifies qret size and changes the score order
          // get best island island=连续匹配 类似 consistent group
          if(!islands.empty())
          {
            const island & ile_ = 
            *std::max_element(islands.begin(), islands.end());
            // check temporal consistency of this island
            updateTemporalWindow(ile_,entry_id);//当前图像的entry_id
            
            // get the best candidate (maybe match)
            match.match = ile_.best_entry;
            //cout<<"best candidate ="<<match.match<<"\n query_id"<<match.query;
            
  
            if(entry_id-ile_.best_entry< 10 ) { 
              match.status =MATCH_TOO_CLOSE;
              return false;
              }
              else{          
              if (getConsistentEntries()> 3 ){
                bool loopdetection;
                
                cv::FlannBasedMatcher flann;
                
                loopdetection=isGeometricallyConsistent_Flann(ile_.best_entry,kps,descriptors,flann);
                  if(loopdetection) {
                    match.status=LOOPDETECTED;
                    
                    //  cout<<"[ Result ]best candidate ="<<match.match<<"\n entry_id = "<<match.query;
                
              }
              else match.status=NO_GEOMETRY_CONSISTENCY;
            }else match.status=NO_TEMPORAL_CONSISTENCY;
            }
          }else match.status=NO_GROUP;
      }else match.status=DATABASE_LOW_SCORE;//qr.empty()
   }else match.status=DATABASE_LOW_SCORE;
 }else match.status=DATABASE_NO_RESULT;//!qr.empty()

//if(int (entry_id)+1 > 20 ){//num of close match 
last_bowvec_=bowec;
//}
return match.detection();

}//function loop_detect

void dl::removeLowsocres(DBoW3::QueryResults & q,double threshold){
  DBoW3::Result aux(0, threshold);
  DBoW3::QueryResults::iterator qit = 
    lower_bound(q.begin(), q.end(), aux, DBoW3::Result::geq);
  
  // qit = first element < m_alpha_minus || end
  
  if(qit != q.end())
  {
    int valid_entries = qit - q.begin();
    q.resize(valid_entries);
  }
 } 

/**
 @brief check consistent group  
 @param ile 当前计算的island
 @param ids 当前图像入口 entry_id  
*/
void dl::updateTemporalWindow(const island & ile,DBoW3::EntryId ids){
if(window_.nentries == 0 || int(ids - window_.last_query_id ) >  3)//上次query_id
    //distance between querys 
  {
    window_.nentries = 1;
  }
  else
  {//a1 a2 b1 b2 时间戳
    DBoW3::EntryId a1 = window_.last_matched_island.first_id;
    DBoW3::EntryId a2 = window_.last_matched_island.last_id;
    DBoW3::EntryId b1 = ile.first_id;
    DBoW3::EntryId b2 = ile.last_id;
    //mathc a与b接近重叠
    bool fit = (b1 <= a1 && a1 <= b2) || (a1 <= b1 && b1 <= a2);
    
    if(!fit)
    {
      int d1 = (int)a1 - (int)b2;
      int d2 = (int)b1 - (int)a2;
      int gap = (d1 > d2 ? d1 : d2);
      
      fit = (gap <= 3);//max_distance between groups
    }
    // a1 a2 b1 b2
    if(fit) window_.nentries++;
    else window_.nentries = 1;
  }
  //update
   window_.last_query_id=ids;
   window_.last_matched_island=ile;
}

//感觉如果分数一直较高的话很容易找到附近的图像进行匹配
//计算island中排列 并分离island对 论文要求：close time 的图像作为一个island 匹配对
void dl::computeIslands(  DBoW3::QueryResults & qr,std::vector<island> & islands){
    islands.clear();
  
  if(qr.size() == 1)
  { 
    islands.push_back(island(qr[0].Id, qr[0].Id, calculateIslandScore(qr, 0, 0)));
    islands.back().best_entry = qr[0].Id;
    islands.back().best_score = qr[0].Score;
  }
  else if(!qr.empty())
  {
    // sort query results in ascending order of ids
    std::sort(qr.begin(), qr.end(), DBoW3::Result::ltId);//按query entry_id 从小到达排列   
    
    // create long enough islands
    DBoW3::QueryResults::const_iterator dit = qr.begin();
    DBoW3::EntryId  first_island_entry =  dit->Id;
    DBoW3::EntryId  last_island_entry =  dit->Id;
    
    // these are indices of q
    unsigned int i_first = 0;
    unsigned int i_last = 0;
    
    double best_score = dit->Score;
    DBoW3::EntryId best_entry = dit->Id;

    ++dit;//dit =begin()+1
    for(unsigned int idx = 1; dit != qr.end(); ++dit, ++idx)
    {//只有遇到较少分数被remove掉的情况时dit才会和last_island_entry 分开
      if((int)dit->Id - last_island_entry < 3 )//intra_group_gap
      {  
        // go on until find the end of the island ||finding he best_socre and best_entry
        last_island_entry = dit->Id;
        // first_island_entry=first_island_entry+1;
        i_last = idx;//idx=dit
        if(dit->Score > best_score)//update highest scores
        {
          best_score = dit->Score;
          best_entry = dit->Id;//
        }
      }
      else
      { // 出现分数较小时 只要last 和first 不相等就可以组成一个island
        // end of island reached
        int length = last_island_entry - first_island_entry + 1;
        if(length >= 1)//min_matches_per_group)
        {//islands first entry last entry score
          islands.push_back( island(first_island_entry, last_island_entry,
          calculateIslandScore(qr,i_first, i_last)) );//sum of islands
          islands.back().best_score = best_score;
          islands.back().best_entry = best_entry;
        }
        
        // prepare next island
        first_island_entry = last_island_entry = dit->Id;
        i_first = i_last = idx;
        best_score = dit->Score;
        best_entry = dit->Id;
      }
    }//end of iter pit
    // add last island
    if(last_island_entry - first_island_entry + 1 >= 
      1)//min_matches_per_group)//根据阈值调整island长度
    {//island值存放first 和last entry_id 以及island最高分
      islands.push_back( island(first_island_entry, last_island_entry,
        calculateIslandScore(qr, i_first, i_last)) );

      islands.back().best_score = best_score;
      islands.back().best_entry = best_entry;
    }
  }
}

//检测entries 连续性
int  dl::getConsistentEntries(){
  return window_.nentries;
} 

double dl::calculateIslandScore(const DBoW3::QueryResults &wqr,int first,int last){
  double sum = 0;
  while(first <= last) sum += wqr[first++].Score;
  return sum;
}

//初始化flannbased matcher
void dl::SetFlann( cv::FlannBasedMatcher &flann,const cv::Mat & descriptors){

  flann.clear();
  flann.add(descriptors);
  flann.train();

}

//flannbased 检查是否符合连续组之间的要求
// 
bool dl::isGeometricallyConsistent_Flann
  ( const DBoW3::EntryId old_entry,
    const std::vector<cv::KeyPoint> &keys, 
    const cv::Mat  & descriptors,
    cv::FlannBasedMatcher &flann_structure
    ) 
{
  std::vector<unsigned int> i_old, i_cur; // indices of correspondences

  const std::vector<cv::KeyPoint> &old_keys = m_image_keys[old_entry];
  const cv::Mat &desc_old=m_image_descriptors[old_entry];
  //const vector<TDescriptor>& old_descs = m_image_descriptors[old_entry];
  const std::vector<cv::KeyPoint> &cur_keys = keys;
  
  std::vector<std::vector<cv::DMatch> > matches;
  cv::Mat desc2, desc_old2;

  descriptors.convertTo(desc2,CV_32FC1);
  desc_old.convertTo(desc_old2,CV_32FC1);
  // descriptors.convertTo(desc2,CV_32FC1,1.0/255);
  // desc_old.convertTo(desc_old2,CV_32FC1,1.0/255);

  // descriptors.convertTo(desc2,CV_8U);
  // desc_old.convertTo(desc_old2,CV_8U);
  
  // flann_structure.knnMatch(desc_old,descriptors,matches, 2,cv::Mat(),false);//单张descs能找到至多2个匹配对 
  flann_structure.knnMatch(desc_old2,desc2,matches, 2,cv::Mat(),false);//单张descs能找到至多2个匹配对 
  
  for(int old_idx = 0; old_idx < (int)matches.size(); old_idx++)
  {
    if(!matches[old_idx].empty())
    {
      int cur_idx = matches[old_idx][0].trainIdx;//<DMatch>train 点 idx
      float dist = matches[old_idx][0].distance;//<>Dmatch两个匹配点之间的距离
      
      bool ok = true;
      if(matches[old_idx].size() >= 2)
      {
        float dist_ratio = dist / matches[old_idx][1].distance;//dMAtch 的第二个
        ok = dist_ratio <= 0.6;//消除一些误匹配的情况 parameters  neigbours_ratio
      }
      
      if(ok)
      {
        vector<unsigned int>::iterator cit =
          std::find(i_cur.begin(), i_cur.end(), cur_idx);
        
        if(cit == i_cur.end())
        {
          i_old.push_back(old_idx);
          i_cur.push_back(cur_idx);
        }
        else
        {
          int idx = i_old[ cit - i_cur.begin() ];
          if(dist < matches[idx][0].distance)
          {
            i_old[ cit - i_cur.begin() ] = old_idx;
          }
        }
      }
    }
  }
  
  if((int)i_old.size() >= 12 )//12 parameters minFpoints 计算基础矩阵所需要的最少点数量
  {
    // add matches to the vectors for calculating the fundamental matrix
    vector<unsigned int>::const_iterator oit, cit;
    oit = i_old.begin();
    cit = i_cur.begin();
    
    vector<cv::Point2f> old_points, cur_points;
    old_points.reserve(i_old.size());
    cur_points.reserve(i_cur.size());
    
    for(; oit != i_old.end(); ++oit, ++cit)
    {
      const cv::KeyPoint &old_k = old_keys[*oit];
      const cv::KeyPoint &cur_k = cur_keys[*cit];
      
      old_points.push_back(old_k.pt);
      cur_points.push_back(cur_k.pt);
    }
    
    cv::Mat oldMat(old_points.size(), 2, CV_32F, &old_points[0]);
    cv::Mat curMat(cur_points.size(), 2, CV_32F, &cur_points[0]);
    return true;
    //  return fsolver_.checkFundamentalMat(oldMat, curMat, 
      //  2.0, 12,0.99, 500);      
  }
  
  return false;
}

double dl::distance(const cv::Mat &a,
  const cv::Mat &b)
{
  // Bit count function got from:
  // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
  // This implementation assumes that a.cols (CV_8U) % sizeof(uint64_t) == 0

  const uint64_t *pa, *pb;
  pa = a.ptr<uint64_t>(); // a & b are actually CV_8U
  pb = b.ptr<uint64_t>();

  uint64_t v, ret = 0;
  for(size_t i = 0; i < a.cols / sizeof(uint64_t); ++i, ++pa, ++pb)
  {
    v = *pa ^ *pb;
    v = v - ((v >> 1) & (uint64_t)~(uint64_t)0/3);
    v = (v & (uint64_t)~(uint64_t)0/15*3) + ((v >> 2) &
      (uint64_t)~(uint64_t)0/15*3);
    v = (v + (v >> 4)) & (uint64_t)~(uint64_t)0/255*15;
    ret += (uint64_t)(v * ((uint64_t)~(uint64_t)0/255)) >>
      (sizeof(uint64_t) - 1) * CHAR_BIT;
  }

  return static_cast<double>(ret);
}

/** @brief 寻找entry周边匹配对
*/
void dl::getMatches_neighratio(
  const std::vector<cv::Mat> &A, const std::vector<unsigned int> &i_A,
  const std::vector<cv::Mat> &B, const std::vector<unsigned int> &i_B,
  std::vector<unsigned int> &i_match_A, std::vector<unsigned int> &i_match_B)  
{
  i_match_A.resize(0);
  i_match_B.resize(0);
  i_match_A.reserve( std::min(i_A.size(), i_B.size()) );
  i_match_B.reserve( std::min(i_A.size(), i_B.size()) );
  
  std::vector<unsigned int>::const_iterator ait, bit;
  unsigned int i, j;
  i = 0;
  for(ait = i_A.begin(); ait != i_A.end(); ++ait, ++i)
  {
    int best_j_now = -1;
    double best_dist_1 = 1e9;
    double best_dist_2 = 1e9; 
    
    j = 0;
    for(bit = i_B.begin(); bit != i_B.end(); ++bit, ++j)
    {
      double d = distance(A[*ait], B[*bit]);
            
      // in i
      if(d < best_dist_1)
      {
        best_j_now = j;
        best_dist_2 = best_dist_1;
        best_dist_1 = d;
      }
      else if(d < best_dist_2)
      {
        best_dist_2 = d;
      }
    }
    
    if(best_dist_1 / best_dist_2 <= 0.6)
    {
      unsigned int idx_B = i_B[best_j_now];
      bit = find(i_match_B.begin(), i_match_B.end(), idx_B);
      
      if(bit == i_match_B.end())
      {
        i_match_B.push_back(idx_B);
        i_match_A.push_back(*ait);
      }
      else
      {
        unsigned int idx_A = i_match_A[ bit - i_match_B.begin() ];
        double d = distance(A[idx_A], B[idx_B]);
        if(best_dist_1 < d)
        {
          i_match_A[ bit - i_match_B.begin() ] = *ait;
        }
      }
        
    }
  }
}

void dl::publishMap(const cv::Mat & image ){
if (image_pub.getNumSubscribers()>0)
 cv_bridge::CvImage cv_image_pub;

std_msgs::Header header;
header.stamp=ros::Time::now();
// cv_image_pub.header.stamp=ros::Time::now();

sensor_msgs::ImagePtr msg=cv_bridge::CvImage(header,"mono8",image).toImageMsg();
image_pub.publish(msg);
}

//calcaulate essentila matrix 
Eigen::Matrix4d dl::calculateRelativePose(
  //const cv::Mat & essentialMat,
  const std::vector<cv::KeyPoint> &kps1,
  const std::vector<cv::KeyPoint> &kps2
  
){
  std::vector<cv::Point2f> pp1;
  std::vector<cv::Point2f> pp2;

  int min_points=std::min(kps1.size(),kps2.size());
  
    for(int i=0;i<min_points;i++){
      pp1.push_back(cv::Point2f(kps1[i].pt));
      pp2.push_back(cv::Point2f(kps2[i].pt));
    }
  
  //   //todo find camera matrix 
  

  // // <<226.38018519795807, 0.0, 173.6470807871759, 0.0, 226.15002947047415, 133.73271487507847, 0, 0, 1;//=&K_;
  // // cameraMatrix.
  cv::Mat R,t,mask,ff;//inliners;
  std::vector<uchar>inliners;
  ff=cv::findFundamentalMat(pp1,pp2,inliners,cv::RANSAC);
  int ptrueSize=pp1.size()-inliners.size();
  std::vector<cv::Point2f> inl1;
  std::vector<cv::Point2f>  inl2;
  
  for (int i ,lainaCount= 0; i < pp1.size(); i++)
  {
    if(inliners[i]!=0){
      inl1.push_back(cv::Point2f(pp1[i]));
      inl2.push_back(cv::Point2f(pp2[i]));
    }
    
  } 
  cv::Mat E =cv::findEssentialMat(inl1,inl2,camera_Matrix_,cv::RANSAC,0.999,1.0,mask);
  // cv::Mat fake_E=camera_Matrix_.t()  * ff *camera_Matrix_;
  cv::recoverPose(E,inl1,inl2,camera_Matrix_,R,t);
  
  for(int i=0;i<inl1.size();i++){
    cv::Mat y1 =(cv::Mat_<double>(3,1)<<inl1[i].x,inl1[i].y,1);
    cv::Mat y2 =(cv::Mat_<double>(3,1)<<inl2[i].x,inl2[i].y,1);
    
    cv::Mat t_x=cv::Mat_<double>(3,3)<<(0,-t.at<double>(0,2),t.at<double>(1,0),
                                      t.at<double>(2,0), 0, -t.at<double>(0,0),
                                      -t.at<double>(1,0),t.at<double>(0,0),0);
    cv::Mat d =y2.t() * camera_Matrix_.inv() * t_x * R  * camera_Matrix_.inv() * y1;
    // std::cout<<"eppolar constraint =="<<d<<std::endl;
  }
  
  Eigen::Matrix3d Reg;
  Eigen::Vector3d teg;
  Eigen::Matrix4d Tcw;

  Tcw.setIdentity();
  cv::cv2eigen(R,Reg);
  cv::cv2eigen(t,teg);
  Tcw.block<3,3>(0,0)=Reg;
  Tcw.block<3,1>(0,3)=teg;
  
    return Tcw;
  }

void dl::publishPose( Eigen::Matrix4d & tcw,const ros::Time &t_lc)
{ 
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  ps_ptr->header.stamp = t_lc;
  ps_ptr->header.frame_id = "lc";
  
  Eigen::Matrix3d rotation_matrix=tcw.topLeftCorner<3,3>();
  Eigen::Vector3d translation =tcw.topRightCorner<3,1>();
  Eigen::Quaterniond qd(rotation_matrix);
  
  ps_ptr->pose.position.x = translation.x();
  ps_ptr->pose.position.y = translation.y();
  ps_ptr->pose.position.z = translation.z();
  
  ps_ptr->pose.orientation.x = qd.x();
  ps_ptr->pose.orientation.y = qd.y();
  ps_ptr->pose.orientation.z = qd.z();
  ps_ptr->pose.orientation.w = qd.w();
  
  pose_pub_.publish(ps_ptr);
}