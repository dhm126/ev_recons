#include <esvo_core/optimization/optimizer.h>

namespace esvo_core{

optimizer::optimizer(
    const CameraSystem::Ptr & camsysPtr
):camSysPtr_(camsysPtr)
{
    fx_=camSysPtr_->cam_left_ptr_->P_(0,0);
    fy_=camSysPtr_->cam_left_ptr_->P_(1,1);
    cx_=camSysPtr_->cam_left_ptr_->P_(0,2);
    cy_=camSysPtr_->cam_left_ptr_->P_(1,2);
}
/*
bundleadjustment without covisibility connection
*/   
void optimizer::localBA( KeyFrame & kf){
    if(kf.kfvdp_.empty()) return ;
    g2o::SparseOptimizer  optimiser;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser.setAlgorithm(solver);
    ROS_INFO("1");

    int psize=kf.kfvdp_.size();
    for(auto p=kf.kfvdp_.begin(),end=kf.kfvdp_.end();p!=end;p++){
    size_t id=p-kf.kfvdp_.begin();//0~ psize-1
    g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();

        vSE3->setEstimate(convert2SE3Quart(p->T_world_cam()));//add quaternion
        vSE3->setId(id+1);
        vSE3->setFixed(id==0);
        optimiser.addVertex(vSE3);    
        
        g2o::VertexSBAPointXYZ *vPoint=new g2o::VertexSBAPointXYZ();
        p->update_p_world(); 
        Eigen::Vector3d p_3d= p->p_world();
        
        vPoint->setEstimate(p_3d);
        vPoint->setId(id+psize+1);
        
        vPoint->setMarginalized(true);
        optimiser.addVertex(vPoint);
        
        
            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;
            
            obs=Eigen::Vector2d(p->col(),p->row());
            
            //todo mesurement    
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id+1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id+psize+1)));
            
            e->setMeasurement(obs);

            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser.addEdge(e);
            delete vSE3;
            delete e ;
            delete vPoint;

    }
    optimiser.initializeOptimization();
    optimiser.optimize(5);
    
    for(int  i=psize+1;i<2*psize;i++)
    {
        g2o::VertexSBAPointXYZ *vPoint =  dynamic_cast<g2o::VertexSBAPointXYZ*>(optimiser.vertex(i)); 
        kf.kfvdp_[i-psize-1].setP_world(Eigen::Vector3d(vPoint->estimate().x(),vPoint->estimate().y(),vPoint->estimate().z()));
         delete vPoint;
    }

}
void optimizer::BundleAdjustment(const std::deque<KeyFrame> &vkf){
    g2o::SparseOptimizer  optimiser;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser.setAlgorithm(solver);
    
    int id=vkf.size()+1;
    //set vertex for every pose 
    for (int i=0;i<vkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        vSE3->setEstimate(convert2SE3Quart(vkf[i].kfT_w_c));//add quaternion 
        
        vSE3->setId(vkf[i].kfid_);
        vSE3->setFixed(vkf[0].kfid_==0);
        optimiser.addVertex(vSE3);
    
    //add vertex map points     
        for (int j=0;j<vkf[i].vkfdps_.size();j++)//smp points vector 3d
        {
        g2o::VertexSBAPointXYZ *vPoint=new g2o::VertexSBAPointXYZ();
         
        Eigen::Vector3d p_3d=vkf[i].getRotationMatrix() * vkf[i].kfvdp_[j].p_cam()+ vkf[i].getTranslation();  
        
        
        vPoint->setEstimate(p_3d);
        //map points id 
        id++;

        vPoint->setId(id);
        
        vPoint->setMarginalized(true);
        optimiser.addVertex(vPoint);

            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;
            
            obs=Eigen::Vector2d(vkf[i].kfvdp_[j].col(),vkf[i].kfvdp_[j].row());
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(i)));
            // optimiser.vertices().find(i)->second;

            e->setMeasurement(obs);

            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser.addEdge(e);
        
        }
    }   
    optimiser.initializeOptimization();
    optimiser.setVerbose(true);
    optimiser.optimize(50);
    // resultLog(optimiser,vkf,false);
}

//only for test 
void optimizer::POptimization( std::deque<KeyFrame*> pvkf, std::vector<DepthPoint*> vdps ){
    
    g2o::SparseOptimizer  optimiser;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);
    optimiser.setAlgorithm(solver);
    
    //set vertex for every pose 
    for (int i=0;i<pvkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        vSE3->setEstimate(convert2SE3Quart(pvkf[i]->kfT_w_c));//add quaternion 
        
        vSE3->setId(pvkf[i]->kfid_);
        
        vSE3->setFixed(pvkf[0]->kfid_==0);
   
        optimiser.addVertex(vSE3);
    }
    //add vertex map points     
        int id=pvkf.size()+1;
        // int kf_size=pvkf.front()->vkfdps_.size();
        for (int j=0;j<vdps.size();j++)//smp points vector 3d
        {
        g2o::VertexSBAPointXYZ *vPoint=new g2o::VertexSBAPointXYZ();
        Eigen::Matrix4d T_world_mps;
        for (size_t i = 0; i < pvkf.size()-1; i++)
        {

            // if(id<kfscontain.at(pvkf[i+1])&&id>kfscontain.at(pvkf[i]))
            // {
            //     T_world_mps=pvkf[i]->kfT_w_c;
            //     break;
            // }   
        }
    
        Eigen::Vector3d p_3d= T_world_mps.block<3,3>(0,0) * (vdps[j]->p_cam())+T_world_mps.block<3,1>(0,3);  
        // pvkf[i]->getRotationMatrix() * pvkf[i]->vkfdps_[j].second +pvkf[i]->getTranslation();
        
        vPoint->setEstimate(p_3d);
        //map points id 
        id++;

        vPoint->setId(id);
        
        vPoint->setMarginalized(true);
        optimiser.addVertex(vPoint);
        }
        
    //     {   
    //     g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
    //     Eigen::Vector2d obs;

    //     obs=Eigen::Vector2d(pvkf[i]->vkfdps_[j].first.first,pvkf[i]->vkfdps_[j].first.second);
    //     uv.push_back(coord(pvkf[i]->vkfdps_[j].first.first,pvkf[i]->vkfdps_[j].first.second));
    //         //todo mesurement 
            
    //     e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id)));
    //     e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(i)));
    //         // optimiser.vertices().find(i)->second;

    //     e->setMeasurement(obs);

    //         // const float 
    //     Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
    //     e->setInformation(info);

    //         e->fx=226.38018519795807;
    //         e->fy=226.15002947047415;
    //         e->cx=173.6470807871759;
    //         e->cy=133.73271487507847;

    //         optimiser.addEdge(e);    
    //     }
       

    
    // optimiser.initializeOptimization();
    // //optimiser.setVerbose(true);
    // optimiser.optimize(5);

    int idx=pvkf.size()+1;
    for(int i=0;i< pvkf.size();i++){
    g2o::VertexSE3Expmap *vSE3_r =  dynamic_cast<g2o::VertexSE3Expmap*>(optimiser.vertex(i)); 
    
    Eigen::Matrix3d rotation_op= vSE3_r->estimate().rotation().toRotationMatrix();
   
    Eigen::Vector3d pose_op= Eigen::Vector3d(vSE3_r->estimate().translation().x(),
                                            vSE3_r->estimate().translation().y(),
                                            vSE3_r->estimate().translation().z()) ;
    Eigen::Matrix4d T_op;
    T_op.block<3,3>(0,0)=rotation_op;T_op.block<3,1>(0,3)=pose_op;
    pvkf[i]->kfT_w_c= T_op;//push_back(pose_op);

    // for(int j=0;j<kfscontain[j];j++){
    //     g2o::VertexSBAPointXYZ *v_point_r = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimiser.vertex(idx)); 
    //     Eigen::Vector3d t_op=Eigen::Vector3d(v_point_r->estimate().x(),v_point_r->estimate().y(),v_point_r->estimate().z()); 
        
    //     pvkf[i]->vkfdps_[j]=std::make_pair(uv[idx-pvkf.size()-1],t_op);

    // }
    }

}
/*
using vdp for BA
considering covisibility connections 
final covisibility ?=essentialMap
*/
void optimizer::banbenvdp(std::deque<KeyFrame> & vkf){
    assert(!vkf.empty());
    g2o::SparseOptimizer  optimiser;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser.setAlgorithm(solver);
    
    int id=vkf.size()+1;
    //set vertex for every pose 关键帧节点 
    for (int i=0;i<vkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        // vSE3->setEstimate(g2o::SE3Quat());
        vSE3->setEstimate(convert2SE3Quart(vkf[i].kfT_w_c));//add quaternion 
        
        vSE3->setId(vkf[i].kfid_);
         vSE3->setFixed(i==0||i==vkf.size()-1);// 
        // vSE3->setFixed(true);
        
        optimiser.addVertex(vSE3);
        
        
    }   
        //add vertex map points 关键帧id

        for(int i=0;i<vkf.size();i++)
        { //深度点节点
        for (int j=0;j<vkf[i].kfvdp_.size();j++)//当前关键帧中深度点id
        {
        
        g2o::VertexSBAPointXYZ *vPoint=new g2o::VertexSBAPointXYZ();
        
        Eigen::Vector3d p_3d=vkf[i].kfvdp_[j].p_world();
        
        vPoint->setEstimate(p_3d);
        //map points id 
        id++;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimiser.addVertex(vPoint);
        
        
        //no observation only consider pose current depthpoints constraint
        
            if(vkf[i].nieba_[&vkf[i].kfvdp_[j]].empty()) //没有共识关系
            {
            
            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;
            
            obs<<vkf[i].kfvdp_[j].col(),vkf[i].kfvdp_[j].row();
            /*
            测试用代码 为了证明 Z* [u,v,1]=KTp 且本文中所使用的K P基本一直
            */
            // Eigen::Vector2d p_ver;
            // camSysPtr_->cam_left_ptr_->world2Cam(vkf[i].kfvdp_[j].p_cam(),p_ver);
            
            // Eigen::Vector3d p_plane;
            // double invDepth=vkf[i].kfvdp_[j].invDepth();
            // p_plane<<obs.x(),obs.y(),1.0;
            // Eigen::Vector3d p_cam_true;
            // camSysPtr_->cam_left_ptr_->cam2World(obs,invDepth,p_cam_true);
            // Eigen::Vector3d p_cam= 1/invDepth * camSysPtr_->cam_left_ptr_->K_.block<3,3>(0,0).inverse()* p_plane;
            // std::cout<<"K算得p_cam"<<p_cam<<" 原来的p_cam"<<vkf[i].kfvdp_[j].p_cam()<<"\t"<<"使用ESVO计算的p_cam"<<p_cam_true;
            
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(i)));//points id 
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id)));//关键帧id 
            
            e->setMeasurement(obs);
            g2o::RobustKernelHuber * rk=new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(std::sqrt(5.93));//cpy orbslam2 
            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser.addEdge(e);
            
            }
            else {//当前深度点有covislibility关系
            for(auto kit=vkf[i].nieba_[&vkf[i].kfvdp_[j]].begin();kit!=vkf[i].nieba_[&vkf[i].kfvdp_[j]].end();kit++)//vkf[i].nieba_[&vkf[i].kfvdp_[j]].size()
            {
            //kit 共识帧的号
            int observation_id=*kit;
            // if(observation_id <= 0) continue;
            
            // int observation_id=vkf[i].nieba_[&vkf[i].kfvdp_[j]][k];
            //std::cout<<"check covframe  id "<<observation_id<<"\n";

            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;
            
            // 地图点在共识帧uv 坐标
            Eigen::Vector3d p_ref_cam=vkf[i].kfvdp_[j].p_cam();
            Eigen::Matrix4d T_ref_cov=vkf[*kit].kfT_w_c.inverse() * vkf[i].kfT_w_c;
            
            Eigen::Vector3d p_cam= T_ref_cov.block<3,3>(0,0) * p_ref_cam +T_ref_cov.block<3,1>(0,3); ;
            Eigen::Vector2d p_ob;
            camSysPtr_->cam_left_ptr_->world2Cam(p_cam,p_ob);
            
            obs<<int(p_ob.x()),int(p_ob.y());
            
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(id)));//depthpoints 0 
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser.vertex(observation_id)));//keyframe 1 

            e->setMeasurement(obs);
            
            g2o::RobustKernelHuber * rk=new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(std::sqrt(5.93));//cpy orbslam2 5.99
            
            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);
            
            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser.addEdge(e);
            
            // delete e; 
                } 
            }
            }
        }
        std::cout<<"edge 的数量 "<<optimiser.edges().size()<<std::endl;
    std::cout<<"\n--- 设立节点完成 ---\n"<<std::endl;
    resultLog(optimiser,vkf,true,"/tmp/nonOp.txt");
    optimiser.initializeOptimization();
    optimiser.setVerbose(true);
    optimiser.optimize(5);
    
    resultLog(optimiser,vkf,true,"/tmp/opTest.txt");
}   

void optimizer::ggICP(std::vector<KeyFrame > & kfs){
    if(kfs.size()!=2) {
        ROS_ERROR("please set appropiate size for icp problem");
        return ;
    }

    int SminPoints=std::min(kfs[0].kfvdp_.size(),kfs[1].kfvdp_.size());
    g2o::SparseOptimizer  optimiser;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver_;

    linearSolver_=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver_);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser.setAlgorithm(solver);

    for(int i=0;i<2;i++){
    g2o::VertexSE3 *vc=new g2o::VertexSE3() ;
    vc->setEstimate(convert2SE3Quart(kfs[i].kfT_w_c));
    vc->setId(i);
    vc->setFixed(i==0);
    optimiser.addVertex(vc);
    }
    
    for(int i=0;i<SminPoints;i++){

        g2o::VertexSE3* vp0=dynamic_cast<g2o::VertexSE3*>(optimiser.vertices().find(0)->second);
        g2o::VertexSE3* vp1=dynamic_cast<g2o::VertexSE3*>(optimiser.vertices().find(1)->second);
    
        g2o::Edge_V_V_GICP * es=new g2o::Edge_V_V_GICP;
        es->setVertex(0,vp0);
        es->setVertex(1,vp1);

        g2o::EdgeGICP meas;
        //meas:mesurement 需要求当前点云的法向量 故作罢
    }

}
/**@
 * @ brief 寻找深度点和关键帧中的共识关系 
 * @ 
*/
void optimizer::getCovConnections(std::deque<KeyFrame> & vkf){
    // size_t rows=camSysPtr_->cam_left_ptr_->height_;
    // size_t cows=camSysPtr_->cam_left_ptr_->width_;

    // for(auto kit=vkf.begin();kit!=vkf.end();kit++){
    //     for(auto dit=kit->kfvdp_.begin();dit!=kit->kfvdp_.end();dit++){
    //         for(auto cit=vkf.begin();cit!=vkf.end();cit++){
    //         if(cit==kit ) continue;
    //         Eigen::Vector3d x_cam;
    //         dit->update_p_world();
    //         x_cam=dit->p_world();
    //         Eigen::Vector2d x_ob;
    //         Eigen::Vector3d x_world=cit->kfT_w_c.block<3,3>(0,0).transpose()*(x_cam- cit->kfT_w_c.block<1,3>(0,3));
    //         camSysPtr_->cam_left_ptr_->world2Cam(x_world,x_ob);//world2cam 只有相机坐标系
    //         if(x_ob.x()>0 &&x_ob.y() > 0&& x_ob.x()<cows&&x_ob.y()<rows){
            
    //         }
            
            
    //         }
    //     }
    // }
} 
optimizer::~optimizer(){
}

g2o::SE3Quat optimizer::convert2SE3Quart(const Eigen::Matrix4d & T_c_w)
{
 Eigen::Matrix<double,3,3> R;

    R <<T_c_w(0,0),T_c_w(0,1),T_c_w(0,2),
    T_c_w(1,0),T_c_w(1,1),T_c_w(1,2),
    T_c_w(2,0),T_c_w(2,1),T_c_w(2,2);

    Eigen::Matrix<double,3,1> t(T_c_w(0,3), T_c_w(1,3), T_c_w(2,3));

    return g2o::SE3Quat(R,t);
}

void optimizer::resultLog(g2o::SparseOptimizer  &optimizor,const std::deque<KeyFrame> &vkf,bool chech,std::string  ame){
if (!chech) return ;


// std::string savae_pcd_path="/tmp/cloud_tmp.pcd";
// pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());

LOG(INFO) << "Saving trajectory to " << ame << " ......";

  std::ofstream  f;
  f.open(ame.c_str(), std::ofstream::out);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << ame << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;
    int id=0;
    while (id<vkf.size())
    {
    g2o::VertexSE3Expmap *vSE3_r =  dynamic_cast<g2o::VertexSE3Expmap*>(optimizor.vertex(id)); 

    Eigen::Quaterniond qd= vSE3_r->estimate().rotation();

    Eigen::Vector3d landmark= Eigen::Vector3d( vSE3_r->estimate().translation().x(),
                                               vSE3_r->estimate().translation().y(), 
                                               vSE3_r->estimate().translation().z());
    
    // for(int i=0;i<vkf[id].kfvdp_.size();i++){
    
    // // vkf[id].kfvdp_[i].update_p_world();
    // DepthPoint * dp_new= new DepthPoint(vkf[id].kfvdp_[i]);
    // dp_new->update_p_world();
    // //std::cout<<"??"<<dp_new->p_world().x()<<"\t"<<dp_new->p_world().y()<<"\t"<<dp_new->p_world().z()<<std::endl;
    // pcl::PointXYZ pt(dp_new->p_world().x(),dp_new->p_world().y(),dp_new->p_world().z());
    // cloud_tmp->push_back(pt);
    // delete dp_new;
    // }
    f << vkf[id].ts_.toSec() << " " << std::setprecision(9) << landmark.transpose() << " "
      << qd.x() << " " << qd.y() << " " << qd.z() << " " << qd.w() << std::endl;
    
    id++;
    }
    // bool binary_mode =false;
    // if(pcl::io::savePCDFile(savae_pcd_path,*cloud_tmp,binary_mode)==-1)
    // {   
    //     ROS_INFO("save pcl pcd file failed");
    // }
    f.close();
    LOG(INFO) << "Saving after OP trajectory to " << ame << ". Done !!!!!!.";
}
}//esvo_core
