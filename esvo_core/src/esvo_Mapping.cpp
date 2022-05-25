#include <esvo_core/esvo_Mapping.h>
#include <esvo_core/DVS_MappingStereoConfig.h>
#include <esvo_core/tools/params_helper.h>
// #include 
#include <minkindr_conversions/kindr_tf.h>

#include <geometry_msgs/TransformStamped.h>

#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <iterator>
#include <memory>
#include <algorithm>
#include <utility>

//#define ESVO_CORE_MAPPING_DEBUG
// #define ESVO_CORE_MAPPING_LOG
 #define ESVO_CORE_LOOPCLOSING_LOG
namespace esvo_core
{
esvo_Mapping::esvo_Mapping(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_private)
  : nh_(nh),
    pnh_(nh_private),
    TS_left_sub_(nh_, "time_surface_left", 10),
    TS_right_sub_(nh_, "time_surface_right", 10),
    TS_sync_(ExactSyncPolicy(10), TS_left_sub_, TS_right_sub_),
    it_(nh),
    calibInfoDir_(tools::param(pnh_, "calibInfoDir", std::string(""))),
    camSysPtr_(new CameraSystem(calibInfoDir_, false)),
    dpConfigPtr_(new DepthProblemConfig(
      tools::param(pnh_, "patch_size_X", 25),
      tools::param(pnh_, "patch_size_Y", 25),
      tools::param(pnh_, "LSnorm", std::string("Tdist")),
      tools::param(pnh_, "Tdist_nu", 0.0),
      tools::param(pnh_, "Tdist_scale", 0.0),
      tools::param(pnh_, "ITERATION_OPTIMIZATION", 10),
      tools::param(pnh_, "RegularizationRadius", 5),
      tools::param(pnh_, "RegularizationMinNeighbours", 8),
      tools::param(pnh_, "RegularizationMinCloseNeighbours", 8))),
    dpSolver_(camSysPtr_, dpConfigPtr_, NUMERICAL, NUM_THREAD_MAPPING),
    dFusor_(camSysPtr_, dpConfigPtr_),
    dRegularizor_(dpConfigPtr_),
    op_(camSysPtr_),
    ebm_(camSysPtr_, NUM_THREAD_MAPPING, tools::param(pnh_, "SmoothTimeSurface", false)),
    pc_(new PointCloud()),
    pc_near_(new PointCloud()),
    pc_global_(new PointCloud()),
    depthFramePtr_(new DepthFrame(camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_))
{
  // frame id
  dvs_frame_id_        = tools::param(pnh_, "dvs_frame_id", std::string("dvs"));
  world_frame_id_      = tools::param(pnh_, "world_frame_id", std::string("world"));
  pc_->header.frame_id = world_frame_id_;
  pc_near_->header.frame_id = world_frame_id_;
  pc_global_->header.frame_id = world_frame_id_;

  /**** mapping parameters ***/
  // range and visualization threshold
  invDepth_min_range_   = tools::param(pnh_, "invDepth_min_range", 0.16);
  invDepth_max_range_   = tools::param(pnh_, "invDepth_max_range", 2.0);
  patch_area_           = tools::param(pnh_, "patch_size_X", 25) *  tools::param(pnh_, "patch_size_Y", 25);
  residual_vis_threshold_ = tools::param(pnh_, "residual_vis_threshold", 15);
  cost_vis_threshold_   = pow(residual_vis_threshold_,2) * patch_area_;
  stdVar_vis_threshold_ = tools::param(pnh_, "stdVar_vis_threshold", 0.005);
  age_max_range_        = tools::param(pnh_, "age_max_range", 5);
  age_vis_threshold_    = tools::param(pnh_, "age_vis_threshold", 0);
  fusion_radius_        = tools::param(pnh_, "fusion_radius", 0);
  maxNumFusionFrames_   = tools::param(pnh_, "maxNumFusionFrames", 10);
  FusionStrategy_      = tools::param(pnh_, "FUSION_STRATEGY", std::string("CONST_FRAMES"));
  maxNumFusionPoints_  = tools::param(pnh_, "maxNumFusionPoints", 2000);
  INIT_SGM_DP_NUM_Threshold_ = tools::param(pnh_, "INIT_SGM_DP_NUM_THRESHOLD", 500);
  // options
  bDenoising_          = tools::param(pnh_, "Denoising", false);
  bRegularization_     = tools::param(pnh_, "Regularization", false);
  resetButton_         = tools::param(pnh_, "ResetButton", false);
  // visualization parameters
  bVisualizeGlobalPC_ = tools::param(pnh_, "bVisualizeGlobalPC", false);
  visualizeGPC_interval_ = tools::param(pnh_, "visualizeGPC_interval", 3);
  visualize_range_ = tools::param(pnh_, "visualize_range", 2.5);
  numAddedPC_threshold_ = tools::param(pnh_, "NumGPC_added_per_refresh", 1000);
  // module parameters
  PROCESS_EVENT_NUM_   = tools::param(pnh_, "PROCESS_EVENT_NUM", 500);
  TS_HISTORY_LENGTH_  = tools::param(pnh_, "TS_HISTORY_LENGTH", 100);
  mapping_rate_hz_     = tools::param(pnh_, "mapping_rate_hz", 20);
  // Event Block Matching (BM) parameters
  BM_half_slice_thickness_ = tools::param(pnh_, "BM_half_slice_thickness", 0.001);
  BM_patch_size_X_ = tools::param(pnh_, "patch_size_X", 25);
  BM_patch_size_Y_ = tools::param(pnh_, "patch_size_Y", 25);
  BM_min_disparity_ = tools::param(pnh_, "BM_min_disparity", 3);
  BM_max_disparity_ = tools::param(pnh_, "BM_max_disparity", 40);
  BM_step_          = tools::param(pnh_, "BM_step", 1);
  BM_ZNCC_Threshold_= tools::param(pnh_, "BM_ZNCC_Threshold", 0.1);
  BM_bUpDownConfiguration_ = tools::param(pnh_, "BM_bUpDownConfiguration", false);

  // SGM parameters (Used by Initialization)
  num_disparities_ = 16 * 3;
  block_size_ = 11;
  P1_ = 8 * 1 * block_size_ * block_size_;
  P2_ = 32 * 1 * block_size_ * block_size_;
  uniqueness_ratio_ = 11;
  sgbm_ = cv::StereoSGBM::create(0, num_disparities_, block_size_, P1_, P2_,
                                 -1, 0, uniqueness_ratio_);

  // calcualte the min,max disparity
  double f = (camSysPtr_->cam_left_ptr_->P_(0,0) + camSysPtr_->cam_left_ptr_->P_(1,1))/2;
  double b = camSysPtr_->baseline_;
  size_t minDisparity = max(size_t(std::floor(f*b*invDepth_min_range_)), (size_t)0);
  size_t maxDisparity = size_t(std::ceil(f*b*invDepth_max_range_));
  minDisparity = max(minDisparity, BM_min_disparity_);
  maxDisparity = min(maxDisparity, BM_max_disparity_);

#ifdef  ESVO_CORE_MAPPING_DEBUG
  LOG(INFO) << "f: " << f << " " << " b: " << b;
  LOG(INFO) << "invDepth_min_range_: " << invDepth_min_range_;
  LOG(INFO) << "invDepth_max_range_: " << invDepth_max_range_;
  LOG(INFO) << "minDisparity: " << minDisparity;
  LOG(INFO) << "maxDisparity: " << maxDisparity;
#endif

  // initialize Event Batch Matcher
  ebm_.resetParameters(BM_patch_size_X_, BM_patch_size_Y_, minDisparity, maxDisparity,
                       BM_step_, BM_ZNCC_Threshold_, BM_bUpDownConfiguration_);

  // system status
  ESVO_System_Status_ = "INITIALIZATION";
  nh_.setParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);

  // callback functions
  events_left_sub_  = nh_.subscribe<dvs_msgs::EventArray>("events_left", 0, boost::bind(&esvo_Mapping::eventsCallback, this, _1, boost::ref(events_left_)));
  events_right_sub_ = nh_.subscribe<dvs_msgs::EventArray>("events_right", 0, boost::bind(&esvo_Mapping::eventsCallback, this, _1, boost::ref(events_right_)));
  stampedPose_sub_  = nh_.subscribe("stamped_pose", 0, &esvo_Mapping::stampedPoseCallback, this);
  loopPose_sub_ = nh_.subscribe("/loop_closing/pose_pub", 0, &esvo_Mapping::loopResultCallback, this);

  TS_sync_.registerCallback(boost::bind(&esvo_Mapping::timeSurfaceCallback, this, _1, _2));
  // TF
  tf_ = std::make_shared<tf::Transformer>(true, ros::Duration(100.0));

  // result publishers
  invDepthMap_pub_ = it_.advertise("Inverse_Depth_Map", 1);
  stdVarMap_pub_ = it_.advertise("Standard_Variance_Map", 1);
  ageMap_pub_ = it_.advertise("Age_Map", 1);
  costMap_pub_ = it_.advertise("cost_map", 1);
  pc_pub_ = nh_.advertise<PointCloud>("/esvo_mapping/pointcloud_local", 1);
  if(bVisualizeGlobalPC_)
  {
    gpc_pub_ = nh_.advertise<PointCloud>("/esvo_mapping/pointcloud_global", 1);
    pc_global_->reserve(5000000);
    t_last_pub_pc_ = 0.0;
  }

  // multi-thread management
  mapping_thread_future_ = mapping_thread_promise_.get_future();
  reset_future_ = reset_promise_.get_future();

  // stereo mapping detached thread
  std::thread MappingThread(&esvo_Mapping::MappingLoop, this,
                            std::move(mapping_thread_promise_), std::move(reset_future_));
  MappingThread.detach();

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&esvo_Mapping::onlineParameterChangeCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<DVS_MappingStereoConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // T_world_cur_.setIdentity();
  bInsertKeyframe_=true;
  bLoop_over_=false;
  bDoubleInsert_=false;
}

esvo_Mapping::~esvo_Mapping()
{
  pc_pub_.shutdown();
  invDepthMap_pub_.shutdown();
  stdVarMap_pub_.shutdown();
  ageMap_pub_.shutdown();
  costMap_pub_.shutdown();
}

void esvo_Mapping::MappingLoop(
  std::promise<void> prom_mapping,
  std::future<void> future_reset)
{
  ros::Rate r(mapping_rate_hz_);

  while (ros::ok())
  {
    // reset mapping rate
    if(changed_frame_rate_)
    {
      r = ros::Rate(mapping_rate_hz_);
      changed_frame_rate_ = false;
    }
    // check system status
    nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
    //    LOG(INFO) << "SYSTEM STATUS (MappingLoop): " << ESVO_System_Status_;
    if(ESVO_System_Status_ == "TERMINATE")
    {
      LOG(INFO) << "The Mapping node is terminated manually...";
      break;
    }
    //
    if(TS_history_.size() >= 10)/* To assure the esvo_time_surface node has been working. */
    {
      while(true)
      {
        if(data_mutex_.try_lock())//若mutex已经锁定，则返回不为0错误值；反之返回0表示加锁;在有数据callback時將data_mutex锁住
        {
          dataTransferring();//event transfer:load event for initialization and BM 
          data_mutex_.unlock();
          break;
        }
        else
        {
          if(future_reset.wait_for(std::chrono::nanoseconds(1)) == std::future_status::ready)
          {
            prom_mapping.set_value();
            return;
          }
        }
      }
      // To check if the most current TS observation has been loaded by dataTransferring()
      if(TS_obs_.second.isEmpty())
      {
        r.sleep();
        continue;
      }
      // Do initialization (State Machine)
      if(ESVO_System_Status_ == "INITIALIZATION" || ESVO_System_Status_ == "RESET")
      {
#ifdef ESVO_CORE_MAPPING_DEBUG
        TicToc tt;
        tt.tic();
#endif
        if(InitializationAtTime(TS_obs_.first))//do initialization
        {
//          INITIALIZATION_COUNTER_++;
          LOG(INFO) << "Initialization is successfully done!"; //(" << INITIALIZATION_COUNTER_ << ").";
#ifdef ESVO_CORE_MAPPING_DEBUG
          LOG(INFO) << "Initialization takes: " << tt.toc() << " ms.";
#endif
        }
        else
          LOG(INFO) << "Initialization fails once.";
      }
      // Do mapping
      if(ESVO_System_Status_ == "WORKING"){
        
    	  if(needKeyFrame() && bInsertKeyframe_) {
          // static int taitu;
          
          KeyFrame kf(TS_obs_.second.tr_.getTransformationMatrix());
          kf.curTsFrame_=TS_obs_.second.cvImagePtr_left_->image;
          // if(!last_kf_time_.isZero()||!last_kf_tcw.isZero())
          // velo_=(kf.kfT_w_c.block<3,1>(0,3).norm()-last_kf_tcw.block<3,1>(0,3).norm())/(kf.ts_-last_kf_time_).toSec();
          
          // last_kf_time_=kf.ts_;
          // last_kf_tcw=kf.kfT_w_c;
          dkfs_.push_back(kf);
          bInsertion_.push_back(false);
          ROS_INFO("INSERT KEYfRAME -------------\n");

        }
        
        MappingAtTime(TS_obs_.first);
        // std::cout<<"loop size"<<mPose_lc_.size()<<"\t";
        if(mPose_lc_.size()>6 && !bLoop_over_)
        {
            //std::lock_guard<std::mutex> lock(data_mutex_);
            
            //  todo::calc SE3
            ros::Time t_inj=mPose_lc_.rbegin()->first;
            
            auto pos_inj=dkfs_.begin();//positon lc 
            
            for(int i=0;i<dkfs_.size();++i){
            if(dkfs_[i].ts_<t_inj )              
              pos_inj++;
            }
            double t_diff= std::fabs(t_inj.toSec()-pos_inj->ts_.toSec());
#ifdef ESVO_CORE_LOOPCLOSING LOG
            std::cout<<"time gap between lc and cur="<<t_diff<<std::endl;
            std::cout<<pos_inj->ts_.toNSec()<<std::endl;
#endif
            if( t_diff>0.5 ) {
              // ROS_ERROR("too large the time gap is ,cannot calc relative pose !"); 
            }
            else
            {
              //tlc transform match to query 
              Eigen::Matrix4d T_cur=TS_obs_.second.tr_.getTransformationMatrix();
              Eigen::Matrix4d  T_lc=mPose_lc_.rbegin()->second.inverse();

              double slc=(T_cur.topRightCorner<3,1>()-pos_inj->kfT_w_c.topRightCorner<3,1>()).norm();
              // auto it_ax=dkfs_.end()-1;
              // Eigen::Matrix4d  T_b21=it_ax->kfT_w_c;
              
              //std::cout<<mPose_lc_.rbegin()->first.toNSec();
#ifdef ESVO_CORE_LOOPCLOSING_LOG              
              std::cout<<"what should be here=="<<T_cur<<std::endl;
              std::cout<<"relative scale =="<<slc<<std::endl;
#endif
              T_lc.block<3,1>(0,3) *=slc*0.02; // nu scale of SE3 calculated 
              
              KeyFrame kf_lc(T_lc);
              kf_lc.curTsFrame_=TS_obs_.second.cvImagePtr_left_->image;
              kf_lc.kfT_w_c  = pos_inj->kfT_w_c * kf_lc.kfT_w_c;
#ifdef ESVO_CORE_LOOPCLOSING_LOG              
              std::cout<<"time for inqusition in "<<TS_obs_.first.toNSec()<<std::endl;
              std::cout<<"difference between two injiection and current "<<kf_lc.kfT_w_c.topRightCorner<3,1>().norm()-TS_obs_.second.tr_.getTransformationMatrix().topRightCorner<3,1>().norm()<<std::endl;
              std::cout<<"size of position injection == "<<pos_inj->kfid_<<std::endl;
#endif
              // if((std::fabs (kf_lc.kfT_w_c.norm()-dkfs_.end()->kfT_w_c.norm())) > 0.2 )
              // {
              //   std::cout<<"translation between loop candidate and last keyframe is too big\n ";
              //   continue;
              // }
              // else
              
              
              
              for (int j=0;j<pos_inj->kfvdp_.size();j++){
              //back to camera frame Rt*(p-t)
    
                DepthPoint dp_lc= pos_inj->kfvdp_[j];  
                //kf_lc corrected  world pose 
                dp_lc.updatePose(pos_inj->kfT_w_c);
                dp_lc.update_p_world();
              
                kf_lc.kfvdp_.push_back(dp_lc);

              }
              dkfs_.push_back(kf_lc);

              // std::cout<<"inserted keyframe number is "<<dkfs_.back().kfid_<<"wwww"<<kf_lc.vkfdps_.empty()<<std::endl;
              ESVO_System_Status_="LOOPCLOSING";
              
            }
        }

        
	    }

      if(ESVO_System_Status_=="LOOPCLOSING"){
     	  //r.sleep();
        bInsertKeyframe_=false;
        bLoop_over_=true;

        ROS_INFO("---------start looclosing thread-----------");
        
        std::thread loop(&esvo_Mapping::runPGoptimization,this);
        loop.detach();
        ESVO_System_Status_="WORKING";
      }
    }
    else //haven't receive timesurface 
    {
      if(future_reset.wait_for(std::chrono::nanoseconds(1)) == std::future_status::ready)
      {
        prom_mapping.set_value();
        return;
      }
    }
    r.sleep();
  }
}

void esvo_Mapping::MappingAtTime(const ros::Time& t)
{
  TicToc tt_mapping;
  double t_overall_count = 0;
  /************************************************/
  /************ set the new DepthFrame ************/
  /************************************************/
  DepthFrame::Ptr depthFramePtr_new = std::make_shared<DepthFrame>(
    camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_);
  depthFramePtr_new->setId(TS_obs_.second.id_);
  depthFramePtr_new->setTransformation(TS_obs_.second.tr_);
  depthFramePtr_ = depthFramePtr_new;


  std::vector<EventMatchPair> vEMP;// the container that stores the result of BM.
  /****************************************************/
  /*************** Block Matching (BM) ****************/
  /****************************************************/
  double t_BM = 0.0;
  double t_BM_denoising = 0.0;

  // Denoising operations
  if(bDenoising_)// Set it to "True" to deal with flicker effect caused by VICON.
  {
    tt_mapping.tic();
    // Draw one mask image for denoising.
    cv::Mat denoising_mask;
    createDenoisingMask(vALLEventsPtr_left_, denoising_mask,
                        camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_);

    // Extract denoised events (appear on edges likely).
    vDenoisedEventsPtr_left_.clear();
    extractDenoisedEvents(vCloseEventsPtr_left_, vDenoisedEventsPtr_left_, denoising_mask, PROCESS_EVENT_NUM_);
    totalNumCount_ = vDenoisedEventsPtr_left_.size();

    t_BM_denoising = tt_mapping.toc();
  }
  else
  {
    vDenoisedEventsPtr_left_.clear();
    vDenoisedEventsPtr_left_.reserve(PROCESS_EVENT_NUM_);//每次处理process_event_num个事件
    vDenoisedEventsPtr_left_.insert(
      vDenoisedEventsPtr_left_.end(), vCloseEventsPtr_left_.begin(),
      vCloseEventsPtr_left_.begin() + min(vCloseEventsPtr_left_.size(), PROCESS_EVENT_NUM_));
  }
  
  // block matching
  tt_mapping.tic();
  ebm_.createMatchProblem(&TS_obs_, &st_map_, &vDenoisedEventsPtr_left_);//未定义的denoised events 后插入close events ptr 
  ebm_.match_all_HyperThread(vEMP);//result of block matching 
#ifdef ESVO_CORE_MAPPING_DEBUG
    LOG(INFO) << ">>>> The number of input events: " << vDenoisedEventsPtr_left_.size();
    LOG(INFO) << "++++ Block Matching (BM) generates " << vEMP.size() << " candidates.";
#endif
  t_BM = tt_mapping.toc();
  t_overall_count += t_BM_denoising;
  t_overall_count += t_BM;

  /**************************************************************/
  /*************  Nonlinear Optimization & Fusion ***************/
  /**************************************************************/
  double t_optimization = 0;
  double t_solve, t_fusion, t_regularization;
  t_solve = t_fusion = t_regularization = 0;
  size_t numFusionCount = 0; // To count the total number of fusion (in terms of fusion between two estimates, i.e. a priori and a propagated one).

  tt_mapping.tic();
  // nonlinear opitmization
  std::vector<DepthPoint> vdp;
  vdp.reserve(vEMP.size());//有多少个em对就有多少个深度点
  dpSolver_.solve(&vEMP, &TS_obs_, vdp); // hyper-thread version
#ifdef ESVO_CORE_MAPPING_DEBUG
  LOG(INFO) << "Nonlinear optimization returns: " << vdp.size() << " estimates.";
#endif
//culling depth point  due to setted threshold
  dpSolver_.pointCulling(vdp, stdVar_vis_threshold_, cost_vis_threshold_,
                         invDepth_min_range_, invDepth_max_range_);//筛选
#ifdef ESVO_CORE_MAPPING_DEBUG
    LOG(INFO) << "After culling, vdp.size: " << vdp.size();
#endif
  t_solve = tt_mapping.toc();

  // Fusion (strategy 1: const number of point)
  if(FusionStrategy_ == "CONST_POINTS")
  {
    size_t numFusionPoints = 0;
    tt_mapping.tic();
    dqvDepthPoints_.push_back(vdp);
    for(size_t n = 0; n < dqvDepthPoints_.size(); n++)
      numFusionPoints += dqvDepthPoints_[n].size();//deque *vector =fusion所有的vdp
    while(numFusionPoints > 1.5 * maxNumFusionPoints_)//
    {
      dqvDepthPoints_.pop_front();//pop多余的0.5 max_numfusion points
      numFusionPoints = 0;
      for(size_t n = 0; n < dqvDepthPoints_.size(); n++)
        numFusionPoints += dqvDepthPoints_[n].size();
    }
  }//原来是这样 const frame 就是这点的observation的共视帧 但由于TS采样过于密集(100hz)这样的共视帧并没有什么用
  else if(FusionStrategy_ == "CONST_FRAMES") // (strategy 2: const number of frames)
  {
    tt_mapping.tic();
    dqvDepthPoints_.push_back(vdp);//
    while(dqvDepthPoints_.size() > maxNumFusionFrames_)// dq size?按frame数量pop 深度点 default 10
      dqvDepthPoints_.pop_front();
  }
  else
  {
    LOG(INFO) << "Invalid FusionStrategy is assigned.";
    exit(-1);
  }

  // apply fusion and count the total number of fusion.
  numFusionCount = 0;
  for(auto it = dqvDepthPoints_.rbegin(); it != dqvDepthPoints_.rend(); it++)
  {
    
    numFusionCount += dFusor_.update(*it, depthFramePtr_, fusion_radius_);
    // LOG(INFO) << "num depth point: " << depthFramePtr_->dMap_->size();//反向深度估计 too much depth points here 
    //    LOG(INFO) << "numFusionCount: "  << numFusionCount;
  }

 		
  TotalNumFusion_ += numFusionCount;
  // Now let's play gently. We do not clean the resulting depth map at the beginning (as long as the number of "frames"
  // to be fused is less than maxNumFusionFrames_), because the depth estimates given by SGM (initialization) would
  // not be fused if the T-distribution statistics is used. Therefore, to guarantee a stable tracking at the beginning,
  // we have to preserve as much depth points as possible. Note that, typically, most depth estimates got cleaned because
  // of their "age", which is an attribute reflecting the "goodness of fit" besides standard variance.
  if(dqvDepthPoints_.size() >= maxNumFusionFrames_)
    depthFramePtr_->dMap_->clean(pow(stdVar_vis_threshold_,2), age_vis_threshold_, invDepth_max_range_, invDepth_min_range_);
  t_fusion = tt_mapping.toc();
  
  // regularization
  if(bRegularization_)
  {
    tt_mapping.tic();
    //
    dRegularizor_.apply(depthFramePtr_->dMap_);
    t_regularization = tt_mapping.toc();
  }
  // count time
  t_optimization = t_solve + t_fusion + t_regularization;
  t_overall_count += t_optimization;
  


  for (int i=0;i<depthFramePtr_->dMap_->rows();i++){
    for (int j=0;j<depthFramePtr_->dMap_->cols();j++){
      if(depthFramePtr_->dMap_->exists(i,j)){ 
        if(dkfs_.empty()) break;
        else
        {
          if(!bInsertion_[dkfs_.back().kfid_])
          {
            // DepthPoint *dp_cure= new DepthPoint(depthFramePtr_->dMap_->get(i,j));
            DepthPoint dp_cur=depthFramePtr_->dMap_->get(i,j);
            // Eigen::Matrix4d trans_l=TS_obs_.second.tr_.getTransformationMatrix();
            // dp_cure->updatePose(trans_l);
          
            // dp_cure->update_p_world();
            // Eigen::Vector3d p_worldcur=dp_cure->p_world();

            
            // for(std::deque<KeyFrame>::const_iterator it_befin=dkfs_.begin();it_befin!=dkfs_.end();it_befin++)
            // {
            // Eigen::Vector3d p_ob=it_befin->kfT_w_c.block<3,3>(0,0).transpose() * p_worldcur
            //                       - it_befin->kfT_w_c.block<3,1>(0,3);
            // Eigen::Vector2d x_ob;
            // camSysPtr_->cam_left_ptr_->world2Cam(p_ob,x_ob);
             
            // if(x_ob.x()>0&&x_ob.y()>0&&x_ob.x()<camSysPtr_->cam_left_ptr_->width_&&p_ob.y()<camSysPtr_->cam_left_ptr_->height_)
            //   dkfs_.back().nieba_[dp_cure].push_back(it_befin->kfid_);
             
            // }

            dkfs_.back().kfvdp_.push_back(dp_cur);
            // delete dp_cure;
          } 
      }
      }
    }
  }
//observation =0
  // if(dkfs_.back().nieba_.empty())
  // {
  // bDoubleInsert_=true;
  // ROS_INFO("has bad observation for current dps ,need a new one");
  // dkfs_.pop_back();
  // }

  bInsertion_.back()= true ;
  

  // publish results
  std::thread tPublishMappingResult(&esvo_Mapping::publishMappingResults, this,
                                    depthFramePtr_->dMap_, depthFramePtr_->T_world_frame_, t);
  tPublishMappingResult.detach();

#ifdef  ESVO_CORE_MAPPING_LOG
  LOG(INFO) << "\n";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "--------------Computation Cost (Mapping)---------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Denoising: " << t_BM_denoising << " ms, (" << t_BM_denoising / t_overall_count * 100 << "%).";
  LOG(INFO) << "Block Matching (BM): " << t_BM << " ms, (" << t_BM / t_overall_count * 100 << "%).";
  LOG(INFO) << "BM success ratio: " << vEMP.size() << "/" << totalNumCount_ << "(Successes/Total).";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Update: " << t_optimization << " ms, (" << t_optimization / t_overall_count * 100
            << "%).";
  LOG(INFO) << "-- nonlinear optimization: " << t_solve << " ms, (" << t_solve / t_overall_count * 100
            << "%).";
  LOG(INFO) << "-- fusion (" << numFusionCount << ", " << TotalNumFusion_ << "): " << t_fusion << " ms, (" << t_fusion / t_overall_count * 100
            << "%).";
  LOG(INFO) << "-- regularization: " << t_regularization << " ms, (" << t_regularization / t_overall_count * 100
            << "%).";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "Total Computation (" << depthFramePtr_->dMap_->size() << "): " << t_overall_count << " ms.";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "------------------------------END---------------------------";
  LOG(INFO) << "------------------------------------------------------------";
  LOG(INFO) << "\n";
#endif
}

bool esvo_Mapping::InitializationAtTime(const ros::Time &t)//仅对当前时间戳下ts作gsm
{
  // create a new depth frame
  DepthFrame::Ptr depthFramePtr_new = std::make_shared<DepthFrame>(
    camSysPtr_->cam_left_ptr_->height_, camSysPtr_->cam_left_ptr_->width_);//depth frame
  depthFramePtr_new->setId(TS_obs_.second.id_);//ts_id
  depthFramePtr_new->setTransformation(TS_obs_.second.tr_);//ts_.transformation
  depthFramePtr_ = depthFramePtr_new;

  // call SGM on the current Time Surface observation pair.
  cv::Mat dispMap, dispMap8;
  sgbm_->compute(TS_obs_.second.cvImagePtr_left_->image, TS_obs_.second.cvImagePtr_right_->image, dispMap);
  dispMap.convertTo(dispMap8, CV_8U, 255/(num_disparities_*16.));//直接拿左右目ts求取视差disp

  // get the event map (binary mask)
  cv::Mat edgeMap;
  
  std::vector<std::pair<size_t, size_t> > vEdgeletCoordinates;
  //getEventMap and event coordinates
  createEdgeMask(vEventsPtr_left_SGM_, camSysPtr_->cam_left_ptr_,
                 edgeMap, vEdgeletCoordinates, true, 0);

  // Apply logical "AND" operation and transfer "disparity" to "invDepth".
  std::vector<DepthPoint> vdp_sgm;
  vdp_sgm.reserve(vEdgeletCoordinates.size());
  double var_SGM = pow(0.001,2);
  for(size_t i = 0; i < vEdgeletCoordinates.size(); i++)
  {
    size_t x = vEdgeletCoordinates[i].first;
    size_t y = vEdgeletCoordinates[i].second;

    double disp = dispMap.at<short>(y,x) / 16.0;
    if(disp < 0)
      continue;
    DepthPoint dp(x,y);
    Eigen::Vector2d p_img(x*1.0,y*1.0);
    dp.update_x(p_img);
    double invDepth = disp / (camSysPtr_->cam_left_ptr_->P_(0,0) * camSysPtr_->baseline_);// inv =disp/投影矩陣P*基线长度
    if(invDepth < invDepth_min_range_ || invDepth > invDepth_max_range_)
      continue;
    Eigen::Vector3d p_cam;
    camSysPtr_->cam_left_ptr_->cam2World(p_img, invDepth, p_cam);//像素平面 to world_frame
    dp.update_p_cam(p_cam);
    dp.update(invDepth, var_SGM);// assume the statics of the SGM's results are Guassian.
    dp.residual() = 0.0;
    dp.age() = age_vis_threshold_;
    Eigen::Matrix<double, 4, 4> T_world_cam = TS_obs_.second.tr_.getTransformationMatrix();
    dp.updatePose(T_world_cam);
    vdp_sgm.push_back(dp);
  }
  LOG(INFO) << "********** Initialization (SGM) returns " << vdp_sgm.size() << " points.";
  if(vdp_sgm.size() < INIT_SGM_DP_NUM_Threshold_)
    return false;
  // push the "masked" SGM results to the depthFrame
  dqvDepthPoints_.push_back(vdp_sgm);

  dFusor_.naive_propagation(vdp_sgm, depthFramePtr_);
  // publish the invDepth map
  std::thread tPublishMappingResult(&esvo_Mapping::publishMappingResults, this,
                                    depthFramePtr_->dMap_, depthFramePtr_->T_world_frame_, t);
  tPublishMappingResult.detach();
  return true;
}

bool esvo_Mapping::dataTransferring()//load events  tr
{
  TS_obs_ = std::make_pair(ros::Time(), TimeSurfaceObservation());// clean the TS obs.
  if(TS_history_.size() <= 10)/* To assure the esvo_time_surface node has been working. */
    return false;
  totalNumCount_ = 0;

  // load current Time-Surface Observation
  auto it_end = TS_history_.rbegin();
  it_end++;// in case that the tf is behind the most current TS.
  auto it_begin = TS_history_.begin();
  while(TS_obs_.second.isEmpty())
  {
    Transformation tr;
    if(ESVO_System_Status_ == "INITIALIZATION")
    {
      tr.setIdentity();
      it_end->second.setTransformation(tr);//timehistory.setTrans(tr_=tr)
      TS_obs_ = *it_end;
    }
    if(ESVO_System_Status_ == "WORKING")
    {
      if(getPoseAt(it_end->first, tr, dvs_frame_id_))
      {
        it_end->second.setTransformation(tr);
        TS_obs_ = *it_end;
        tcw_= TS_obs_.second.tr_.getTransformationMatrix();
      }
      else
      {
        // check if the tracking node is still working normally
        nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
        if(ESVO_System_Status_ != "WORKING")
          return false;
      }
    }
    if(it_end->first == it_begin->first)
      break;
    it_end++;
  }
  if(TS_obs_.second.isEmpty())
    return false;

  /****** Load involved events *****/
  // SGM
  if(ESVO_System_Status_ == "INITIALIZATION")
  {
    vEventsPtr_left_SGM_.clear();
    ros::Time t_end    = TS_obs_.first;
    ros::Time t_begin(std::max(0.0, t_end.toSec() - 2 * BM_half_slice_thickness_));
    auto ev_end_it     = tools::EventBuffer_lower_bound(events_left_, t_end);
    auto ev_begin_it   = tools::EventBuffer_lower_bound(events_left_, t_begin);
    const size_t MAX_NUM_Event_INVOLVED = 30000;
    vEventsPtr_left_SGM_.reserve(MAX_NUM_Event_INVOLVED);
    while(ev_end_it != ev_begin_it && vEventsPtr_left_SGM_.size() <= PROCESS_EVENT_NUM_)
    {
      vEventsPtr_left_SGM_.push_back(ev_end_it._M_cur);
      ev_end_it--;
    }
  }

  // BM
  if(ESVO_System_Status_ == "WORKING")
  {
    // copy all involved events' pointers
    vALLEventsPtr_left_.clear();  // Used to generate denoising mask (only used to deal with flicker induced by VICON.)
    vCloseEventsPtr_left_.clear();// Will be denoised using the mask above.

    // load allEvent
    ros::Time t_end    = TS_obs_.first;
    ros::Time t_begin(std::max(0.0, t_end.toSec() - 10 * BM_half_slice_thickness_));
    auto ev_end_it     = tools::EventBuffer_lower_bound(events_left_, t_end);//>=t_end
    auto ev_begin_it   = tools::EventBuffer_lower_bound(events_left_, t_begin);//>=t_begin
    const size_t MAX_NUM_Event_INVOLVED = PROCESS_EVENT_NUM_;//10000
    vALLEventsPtr_left_.reserve(MAX_NUM_Event_INVOLVED);
    vCloseEventsPtr_left_.reserve(MAX_NUM_Event_INVOLVED);
    while(ev_end_it != ev_begin_it && vALLEventsPtr_left_.size() < MAX_NUM_Event_INVOLVED)//load
    {
      vALLEventsPtr_left_.push_back(ev_end_it._M_cur);
      vCloseEventsPtr_left_.push_back(ev_end_it._M_cur);
      ev_end_it--;
    }
    totalNumCount_ = vCloseEventsPtr_left_.size();
#ifdef ESVO_CORE_MAPPING_DEBUG
    LOG(INFO) << "Data Transferring (vALLEventsPtr_left_): " << vALLEventsPtr_left_.size();
    LOG(INFO) << "Data Transforming (vCloseEventsPtr_left_): " << vCloseEventsPtr_left_.size();
#endif
    
    // Ideally, each event occurs at an unique perspective (virtual view) -- pose.
    // In practice, this is intractable in real-time application.
    // We made a trade off by assuming that events occurred within (0.05 * BM_half_slice_thickness_) ms share an identical pose (virtual view).
    // Here we load transformations for all virtual views.
    st_map_.clear();
    ros::Time t_tmp = t_begin;
    while(t_tmp.toSec() <= t_end.toSec())
    {
      Transformation tr;
      if(getPoseAt(t_tmp, tr, dvs_frame_id_))
        st_map_.emplace(t_tmp, tr);//zhelimiande zhi yingagidou chabuduo 
      else
      {
        nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
        if(ESVO_System_Status_ != "WORKING")
          return false;
      }
      //+ slice thickness every time
      t_tmp = ros::Time(t_tmp.toSec() + 0.05 * BM_half_slice_thickness_);//get pose at t_tmp ,t_tmp ++
    }
#ifdef ESVO_CORE_MAPPING_DEBUG
    LOG(INFO) << "Data Transferring (stampTransformation map): " << st_map_.size();
#endif
  }
  return true;
}

void esvo_Mapping::stampedPoseCallback(
  const geometry_msgs::PoseStampedConstPtr &ps_msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // To check inconsistent timestamps and reset.
  static constexpr double max_time_diff_before_reset_s = 0.5;
  const ros::Time stamp_first_event = ps_msg->header.stamp;

  std::string *err_tf = new std::string();
//  int iGetLastest_common_time =
//    tf_->getLatestCommonTime(dvs_frame_id_.c_str(), ps_msg->header.frame_id, tf_lastest_common_time_, err_tf);
  delete err_tf;

  if( tf_lastest_common_time_.toSec() != 0)
  {
    const double dt = stamp_first_event.toSec() - tf_lastest_common_time_.toSec();
    if(dt < 0 || std::fabs(dt) >= max_time_diff_before_reset_s)
    {
      ROS_INFO("Inconsistent event timestamps detected <stampedPoseCallback> (new: %f, old %f), resetting.",
               stamp_first_event.toSec(), tf_lastest_common_time_.toSec());
      reset();
    }
  }

  // add pose to tf
  tf::Transform tf(
    tf::Quaternion(
      ps_msg->pose.orientation.x,
      ps_msg->pose.orientation.y,
      ps_msg->pose.orientation.z,
      ps_msg->pose.orientation.w),
    tf::Vector3(
      ps_msg->pose.position.x,
      ps_msg->pose.position.y,
      ps_msg->pose.position.z));
  tf::StampedTransform st(tf, ps_msg->header.stamp, ps_msg->header.frame_id, dvs_frame_id_.c_str());
  tf_->setTransform(st);
}
void esvo_Mapping::loopResultCallback(const geometry_msgs::PoseStampedConstPtr & lc_msg){

  const ros::Time t_lc_saki=lc_msg->header.stamp;
  Eigen::Matrix4d Tcw_lc=Eigen::Matrix4d::Identity();
  Eigen::Vector3d trans_lc;
  //  Eigen::Matrix3d rot_lc;
  Eigen::Quaterniond quad_lc=Eigen::Quaterniond(lc_msg->pose.orientation.w,lc_msg->pose.orientation.x,lc_msg->pose.orientation.y,lc_msg->pose.orientation.z);
  
  trans_lc<<lc_msg->pose.position.x,lc_msg->pose.position.y,lc_msg->pose.position.z;
  Eigen::Matrix3d rot_lc=Eigen::Matrix3d(quad_lc);

  Tcw_lc.block<3,3>(0,0)=rot_lc;
  Tcw_lc.block<3,1>(0,3)=trans_lc;

  mPose_lc_.insert(std::make_pair(t_lc_saki,Tcw_lc)); 
  
  // std::cout<<"get pose at loopclosing ==="<<mPose_lc_.size()<<std::endl;
}

// return the pose of the left event cam at time t.
bool esvo_Mapping::getPoseAt(
  const ros::Time &t,
  esvo_core::Transformation &Tr,// T_world_virtual
  const std::string& source_frame )
{
  std::string* err_msg = new std::string();
  if(!tf_->canTransform(world_frame_id_, source_frame, t, err_msg))
  {
#ifdef ESVO_CORE_MAPPING_LOG
    LOG(WARNING) << t.toNSec() << " : " << *err_msg;
#endif
    delete err_msg;
    return false;
  }
  else
  {
    tf::StampedTransform st;
    tf_->lookupTransform(world_frame_id_, source_frame, t, st);
    tf::transformTFToKindr(st, &Tr);
    return true;
  }
}

void esvo_Mapping::eventsCallback(
  const dvs_msgs::EventArray::ConstPtr& msg,
  EventQueue& EQ)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  static constexpr double max_time_diff_before_reset_s = 0.5;
  const ros::Time stamp_first_event = msg->events[0].ts;

  // check time stamp inconsistency
  if(!msg->events.empty() && !EQ.empty())
  {
    const double dt = stamp_first_event.toSec() - EQ.back().ts.toSec();
    if(dt < 0 || std::fabs(dt) >= max_time_diff_before_reset_s)
    {
      ROS_INFO("Inconsistent event timestamps detected <eventCallback> (new: %f, old %f), resetting.",
               stamp_first_event.toSec(), events_left_.back().ts.toSec());
      reset();
    }
  }

  // add new ones and remove old ones
  for(const dvs_msgs::Event& e : msg->events)
  {
    EQ.push_back(e);
    int i = EQ.size() - 2;
    while(i >= 0 && EQ[i].ts > e.ts) // we may have to sort the queue, just in case the raw event messages do not come in a chronological order.
    {
      EQ[i+1] = EQ[i];
      i--;
    }
    EQ[i+1] = e;
  }
  clearEventQueue(EQ);
}

void
esvo_Mapping::clearEventQueue(EventQueue& EQ)
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 3000000;
  if (EQ.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t NUM_EVENTS_TO_REMOVE = EQ.size() - MAX_EVENT_QUEUE_LENGTH;
    EQ.erase(EQ.begin(), EQ.begin() + NUM_EVENTS_TO_REMOVE);
  }
}

void esvo_Mapping::timeSurfaceCallback(
  const sensor_msgs::ImageConstPtr& time_surface_left,
  const sensor_msgs::ImageConstPtr& time_surface_right)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // check time-stamp inconsistency
  if(!TS_history_.empty())
  {
    static constexpr double max_time_diff_before_reset_s = 0.5;
    const ros::Time stamp_last_image = TS_history_.rbegin()->first;
    const double dt = time_surface_left->header.stamp.toSec() - stamp_last_image.toSec();
    if(dt < 0 || std::fabs(dt) >= max_time_diff_before_reset_s)
    {
      ROS_INFO("Inconsistent frame timestamp detected <timeSurfaceCallback> (new: %f, old %f), resetting.",
               time_surface_left->header.stamp.toSec(), stamp_last_image.toSec());
      reset();
    }
  }

  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try
  {
    cv_ptr_left  = cv_bridge::toCvCopy(time_surface_left,  sensor_msgs::image_encodings::MONO8);
    cv_ptr_right = cv_bridge::toCvCopy(time_surface_right, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // push back the new time surface map
  ros::Time t_new_TS = time_surface_left->header.stamp;
  // Made the gradient computation optional which is up to the jacobian choice.
  if(dpSolver_.getProblemType() == NUMERICAL)
    TS_history_.emplace(t_new_TS, TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, TS_id_));
  else
    TS_history_.emplace(t_new_TS, TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, TS_id_, true));
  TS_id_++;

  // keep TS_history's size constant
  while(TS_history_.size() > TS_HISTORY_LENGTH_)
  {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
}

void esvo_Mapping::reset()
{
  // mutual-thread communication with MappingThread.
  LOG(INFO) << "Coming into reset()";
  reset_promise_.set_value();
  LOG(INFO) << "(reset) The mapping thread future is waiting for the value.";
  mapping_thread_future_.get();
  LOG(INFO) << "(reset) The mapping thread future receives the value.";

  // clear all maintained data
  events_left_.clear();
  events_right_.clear();
  TS_history_.clear();
  tf_->clear();
  pc_->clear();
  pc_near_->clear();
  pc_global_->clear();
  TS_id_ = 0;
  depthFramePtr_->clear();
  dqvDepthPoints_.clear();

  ebm_.resetParameters(BM_patch_size_X_, BM_patch_size_Y_,
                       BM_min_disparity_, BM_max_disparity_, BM_step_, BM_ZNCC_Threshold_, BM_bUpDownConfiguration_);
  bInsertKeyframe_=true;
  bLoop_over_=false;

  for(int i = 0;i < 2;i++)
    LOG(INFO) << "****************************************************";
  LOG(INFO) << "****************** RESET THE SYSTEM *********************";
  for(int i = 0;i < 2;i++)
    LOG(INFO) << "****************************************************\n\n";

  // restart the mapping thread
  reset_promise_ = std::promise<void>();
  mapping_thread_promise_ = std::promise<void>();
  reset_future_ = reset_promise_.get_future();
  mapping_thread_future_ = mapping_thread_promise_.get_future();
  ESVO_System_Status_ = "INITIALIZATION";
  nh_.setParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
  std::thread MappingThread(&esvo_Mapping::MappingLoop, this,
                            std::move(mapping_thread_promise_), std::move(reset_future_));
  MappingThread.detach();
}

void esvo_Mapping::onlineParameterChangeCallback(DVS_MappingStereoConfig &config, uint32_t level)
{
  bool online_parameters_changed = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if(invDepth_min_range_ != config.invDepth_min_range ||
       invDepth_max_range_ != config.invDepth_max_range ||
       residual_vis_threshold_ != config.residual_vis_threshold ||
       stdVar_vis_threshold_ != config.stdVar_vis_threshold ||
       age_max_range_ != config.age_max_range ||
       age_vis_threshold_ != config.age_vis_threshold ||
       fusion_radius_ != config.fusion_radius ||
       maxNumFusionFrames_ != config.maxNumFusionFrames ||
       bDenoising_ != config.Denoising ||
       bRegularization_ != config.Regularization ||
       resetButton_ != config.ResetButton ||
       PROCESS_EVENT_NUM_ != config.PROCESS_EVENT_NUM ||
       TS_HISTORY_LENGTH_ != config.TS_HISTORY_LENGTH ||
       BM_min_disparity_ != config.BM_min_disparity ||
       BM_max_disparity_ != config.BM_max_disparity ||
       BM_step_ != config.BM_step ||
       BM_ZNCC_Threshold_ != config.BM_ZNCC_Threshold)
    {
      online_parameters_changed = true;
    }

    invDepth_min_range_ = config.invDepth_min_range;
    invDepth_max_range_ = config.invDepth_max_range;
    residual_vis_threshold_ = config.residual_vis_threshold;
    cost_vis_threshold_ = patch_area_ * pow(residual_vis_threshold_,2);
    stdVar_vis_threshold_ = config.stdVar_vis_threshold;
    age_max_range_ = config.age_max_range;
    age_vis_threshold_ = config.age_vis_threshold;
    fusion_radius_ = config.fusion_radius;
    maxNumFusionFrames_ = config.maxNumFusionFrames;
    bDenoising_ = config.Denoising;
    bRegularization_ = config.Regularization;
    resetButton_ = config.ResetButton;
    PROCESS_EVENT_NUM_ = config.PROCESS_EVENT_NUM;
    TS_HISTORY_LENGTH_ = config.TS_HISTORY_LENGTH;
    BM_min_disparity_ = config.BM_min_disparity;
    BM_max_disparity_ = config.BM_max_disparity;
    BM_step_ = config.BM_step;
    BM_ZNCC_Threshold_ = config.BM_ZNCC_Threshold;
  }

  if(config.mapping_rate_hz != mapping_rate_hz_)
  {
    changed_frame_rate_ = true;
    online_parameters_changed = true;
    mapping_rate_hz_ = config.mapping_rate_hz;
  }

  if(online_parameters_changed)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    LOG(INFO) << "onlineParameterChangeCallback ==============";
    reset();
  }
}

void esvo_Mapping::publishMappingResults(
  DepthMap::Ptr depthMapPtr,
  Transformation tr,
  ros::Time t)
{
  cv::Mat invDepthImage, stdVarImage, ageImage, costImage, eventImage, confidenceMap;

  visualizor_.plot_map(depthMapPtr, tools::InvDepthMap, invDepthImage,
                       invDepth_max_range_, invDepth_min_range_, stdVar_vis_threshold_, age_vis_threshold_);
  publishImage(invDepthImage, t, invDepthMap_pub_);

  visualizor_.plot_map(depthMapPtr, tools::StdVarMap,stdVarImage,
                       stdVar_vis_threshold_, 0.0, stdVar_vis_threshold_);
  publishImage(stdVarImage, t, stdVarMap_pub_);

  visualizor_.plot_map(depthMapPtr, tools::AgeMap,ageImage, age_max_range_, 0, age_vis_threshold_);
  publishImage(ageImage, t, ageMap_pub_);

  visualizor_.plot_map(depthMapPtr, tools::CostMap, costImage, cost_vis_threshold_, 0.0, cost_vis_threshold_);
  publishImage(costImage, t, costMap_pub_);

  if(ESVO_System_Status_ == "INITIALIZATION")
    publishPointCloud(depthMapPtr, tr, t);
  if(ESVO_System_Status_ == "WORKING")
  {
    if(FusionStrategy_ == "CONST_FRAMES")
    {
      if(dqvDepthPoints_.size() == maxNumFusionFrames_)
        publishPointCloud(depthMapPtr, tr, t);
    }
    if(FusionStrategy_ == "CONST_POINTS")
    {
      size_t numFusionPoints = 0;
      for(size_t n = 0; n < dqvDepthPoints_.size(); n++)
        numFusionPoints += dqvDepthPoints_[n].size();
      if(numFusionPoints > 0.5 * maxNumFusionPoints_)
        publishPointCloud(depthMapPtr, tr, t);
    }
  }
}

void esvo_Mapping::publishPointCloud(
  DepthMap::Ptr& depthMapPtr,
  Transformation & tr,
  ros::Time& t)
{
  sensor_msgs::PointCloud2::Ptr pc_to_publish (new sensor_msgs::PointCloud2);
  Eigen::Matrix<double, 4, 4> T_world_result = tr.getTransformationMatrix();

  pc_->clear();
  pc_->reserve(50000);
  pc_near_->clear();
  pc_near_->reserve(50000);

  double FarthestDistance = 0.0;
  Eigen::Vector3d FarthestPoint;

  for(auto it = depthMapPtr->begin();it != depthMapPtr->end();it++)
  {
    Eigen::Vector3d p_world = T_world_result.block<3,3>(0,0) * it->p_cam()
                              + T_world_result.block<3,1>(0,3);
    
    pc_->push_back(pcl::PointXYZ(p_world(0), p_world(1), p_world(2)));

    if(it->p_cam().norm() < visualize_range_)
      pc_near_->push_back(pcl::PointXYZ(p_world(0), p_world(1), p_world(2)));
    // For debug
//    if(it->p_cam().norm() > FarthestDistance)
//    {
//      FarthestDistance = it->p_cam().norm();
//      FarthestPoint = it->p_cam();
//    }
  }
#ifdef ESVO_CORE_MAPPING_DEBUG
  LOG(INFO) << "The farthest point (p_cam): " << FarthestPoint.transpose();
#endif

  // publish the local 3D map which is used by the tracker.
  if (!pc_->empty())
  {
#ifdef ESVO_CORE_MAPPING_DEBUG
  LOG(INFO) << "<<<<<<<<<(pointcloud)<<<<<<<<" << pc_->size() << " points are published";
#endif
    pcl::toROSMsg(*pc_, *pc_to_publish);
    pc_to_publish->header.stamp = t;
    pc_pub_.publish(pc_to_publish);
  }

  // publish global pointcloud
  if(bVisualizeGlobalPC_)
  {
    if(t.toSec() - t_last_pub_pc_ > visualizeGPC_interval_)
    {
      PointCloud::Ptr pc_filtered(new PointCloud());
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(pc_near_);
      // sor.setLeafSize(0.03, 0.03, 0.03);// Used in small scale environment.
      sor.setLeafSize(0.3, 0.3, 0.3);// Used in large scale environment.
      sor.filter(*pc_filtered);

      // copy the most current pc tp pc_global
      size_t pc_length = pc_filtered->size();
      size_t numAddedPC = min(pc_length, numAddedPC_threshold_) - 1;
      pc_global_->insert(pc_global_->end(), pc_filtered->end() - numAddedPC, pc_filtered->end());
      // publish point cloud
      pcl::toROSMsg(*pc_global_, *pc_to_publish);
      pc_to_publish->header.stamp = t;
      gpc_pub_.publish(pc_to_publish);
      t_last_pub_pc_ = t.toSec();
    }
  }
}

void
esvo_Mapping::publishImage(
  const cv::Mat &image,
  const ros::Time & t,
  image_transport::Publisher & pub,
  std::string encoding)
{
  if(pub.getNumSubscribers() == 0)
  {
//    LOG(INFO) << "------------------------------: " << pub.getTopic();
    return;
  }
//  LOG(INFO) << "+++++++++++++++++++++++++++++++: " << pub.getTopic();

  std_msgs::Header header;
  header.stamp = t;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encoding.c_str(), image).toImageMsg();
  pub.publish(msg);
}

void esvo_Mapping::createEdgeMask(
  std::vector<dvs_msgs::Event *> &vEventsPtr,
  PerspectiveCamera::Ptr &camPtr,
  cv::Mat& edgeMap,
  std::vector<std::pair<size_t, size_t> >& vEdgeletCoordinates,
  bool bUndistortEvents,
  size_t radius)
{
  size_t col = camPtr->width_;
  size_t row = camPtr->height_;
  int dilate_radius = (int) radius;
  edgeMap = cv::Mat(cv::Size(col, row), CV_8UC1, cv::Scalar(0));
  vEdgeletCoordinates.reserve(col*row);

  auto it_tmp = vEventsPtr.begin();
  while (it_tmp != vEventsPtr.end())
  {
    // undistortion + rectification
    Eigen::Matrix<double,2,1> coor;
    if(bUndistortEvents)
      coor = camPtr->getRectifiedUndistortedCoordinate((*it_tmp)->x, (*it_tmp)->y);
    else
      coor = Eigen::Matrix<double,2,1>((*it_tmp)->x, (*it_tmp)->y);

    // assign
    int xcoor = std::floor(coor(0));
    int ycoor = std::floor(coor(1));

    for(int dy = -dilate_radius; dy <= dilate_radius; dy++)
      for(int dx = -dilate_radius; dx <= dilate_radius; dx++)
      {
        int x = xcoor + dx;
        int y = ycoor + dy;

        if(x < 0 || x >= col || y < 0 || y >= row)
        {}
        else
        {
          edgeMap.at<uchar>(y, x) = 255;
          vEdgeletCoordinates.emplace_back((size_t)x, (size_t)y);
        }
      }
    it_tmp++;
  }
}

void esvo_Mapping::createDenoisingMask(
  std::vector<dvs_msgs::Event *>& vAllEventsPtr,
  cv::Mat& mask,
  size_t row, size_t col)
{
  cv::Mat eventMap;
  visualizor_.plot_eventMap(vAllEventsPtr, eventMap, row, col);//alle eventptr plot on event map
  cv::medianBlur(eventMap, mask, 3);
}

void esvo_Mapping::extractDenoisedEvents(
  std::vector<dvs_msgs::Event *> &vCloseEventsPtr,
  std::vector<dvs_msgs::Event *> &vEdgeEventsPtr,
  cv::Mat& mask,
  size_t maxNum)
{
  vEdgeEventsPtr.reserve(vCloseEventsPtr.size());
  for(size_t i = 0;i < vCloseEventsPtr.size();i++)
  {
    if(vEdgeEventsPtr.size() >= maxNum)
      break;
    size_t x = vCloseEventsPtr[i]->x;
    size_t y = vCloseEventsPtr[i]->y;
    if(mask.at<uchar>(y,x) == 255)
      vEdgeEventsPtr.push_back(vCloseEventsPtr[i]);
  }
}
bool esvo_Mapping::needKeyFrame(){

  static size_t maxInterval=40;
  cur_id=TS_obs_.second.id_;
  if(bDoubleInsert_)
    {
      bDoubleInsert_=false;
      return true;
    }

  // if((tcw_.topRightCorner<3,1>()-last_tcw_.topRightCorner<3,1>()).norm()<0.03)
  //   {
  //     ROS_INFO("in nearly not moving");
  //     last_tcw_=tcw_;  
  //     return true;
  //   }

  if (!((cur_id-old_id) > maxInterval)) return false ;
  else  {
    old_id=cur_id;
    
    //else if (cur_id/old_id >1.12) return true;
    return true;
    }            

return false ;

}
void esvo_Mapping::runPGoptimization(){
  { 
        
      if(dkfs_.empty()) return;
      else {
        auto it_begin=dkfs_.begin();
        auto it_end=dkfs_.end();
        
        while(it_begin!=it_end){
          if(it_begin->kfT_w_c.isZero() ||it_begin->kfvdp_.empty() ) {
            ROS_ERROR("EMPTY ELEMENT IN KEYFRAME which is %d",it_begin->kfid_);
            }
            int outOfBound=0;
          for(int i=0;i<it_begin->kfvdp_.size();i++)
          {
            auto it_befin=dkfs_.begin();

            if(std::fabs(it_begin->kfvdp_[i].T_world_cam().block<3,3>(0,0).trace())<0.01) 
            it_begin->kfvdp_[i].updatePose(it_begin->kfT_w_c);

            it_begin->kfvdp_[i].update_p_world();
            
            //p_world
            Eigen::Vector3d p_worldcur=it_begin->kfvdp_[i].p_world();
            //p_back to different world
            for(std::deque<KeyFrame>::const_iterator it_befin=dkfs_.begin();it_befin!=dkfs_.end();it_befin++)
            {
            Eigen::Vector3d p_ob=it_befin->kfT_w_c.block<3,3>(0,0).transpose() * p_worldcur
                                  - it_befin->kfT_w_c.block<3,1>(0,3);
             //back to cam frame 2d                              
            Eigen::Vector2d x_ob;
            camSysPtr_->cam_left_ptr_->world2Cam(p_ob,x_ob);
             
             if(x_ob.x()>0&&x_ob.y()>0&&x_ob.x()<camSysPtr_->cam_left_ptr_->width_&&p_ob.y()<camSysPtr_->cam_left_ptr_->height_)
             {
              it_begin->nieba_[&it_begin->kfvdp_[i]].push_back(it_befin->kfid_);
             }else outOfBound ++;
            
            }
          } 
#ifdef ESVO_CORE_LOOPCLOSING_LOG
          std::cout<<"numbers of point is out of bound "<<outOfBound<<std::endl;
          std::cout<<"check numbers "<<it_begin->kfid_<<"\n";
          std::cout<<"relativepose== \n "<<it_begin->kfT_w_c<<std::endl;
          std::cout<<"vdp.size()=="<<it_begin->kfvdp_.size()<<std::endl;
          //  it_begin->ImageLog();
          std::cout<<"check timestamp of each keyframe "<<it_begin->ts_.toNSec()<<std::endl;
          std::cout<<"observation size=="<<it_begin->nieba_.size()<<std::endl;
#endif
          it_begin++;
        }
      }
      
        // ROS_ERROR("we are simulating bundleAdjustment");
        //find tcw of last frame   
        // op_.POptimization(optest);
        op_.banbenvdp(dkfs_);
        // op_.BundleAdjustment(dkfs_);
      }
  }

}// esvo_core
