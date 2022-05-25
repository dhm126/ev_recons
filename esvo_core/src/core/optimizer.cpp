#include <esvo_core/optimization/optimizer.h>

namespace esvo_core{

optimizer::optimizer(
    const CameraSystem::Ptr & camsysPtr
):camSysPtr_(camsysPtr)
{
    fx_=camSysPtr_->cam_left_ptr_->K_(0,0);
    fy_=camSysPtr_->cam_left_ptr_->K_(1,1);
    cx_=camSysPtr_->cam_left_ptr_->K_(0,2);
    cy_=camSysPtr_->cam_left_ptr_->K_(1,2);
}
/*
bundleadjustment without covisibility connection
*/   
void optimizer::BundleAdjustment(const std::deque<KeyFrame> &vkf){
    g2o::SparseOptimizer  optimiser_;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver_;

    linearSolver_=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver_);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser_.setAlgorithm(solver);
    
    int id=vkf.size()+1;
    //set vertex for every pose 
    for (int i=0;i<vkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        vSE3->setEstimate(convert2SE3Quart(vkf[i].kfT_w_c));//add quaternion 
        
        vSE3->setId(vkf[i].kfid_);
        vSE3->setFixed(vkf[0].kfid_==0);
        optimiser_.addVertex(vSE3);
    
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
        optimiser_.addVertex(vPoint);

            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;
            
            obs=Eigen::Vector2d(vkf[i].kfvdp_[j].col(),vkf[i].kfvdp_[j].row());
            
            //todo mesurement 
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(id)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(i)));
            // optimiser_.vertices().find(i)->second;

            e->setMeasurement(obs);

            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser_.addEdge(e);
        
        }
    }   
    optimiser_.initializeOptimization();
    optimiser_.setVerbose(true);
    optimiser_.optimize(10);
    // resultLog(optimiser_,vkf,false);
}


void optimizer::POptimization( std::deque<KeyFrame*> pvkf, std::vector<DepthPoint*> vdps ){
    
    g2o::SparseOptimizer  optimiser_;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver_;
    linearSolver_=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver_);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);
    optimiser_.setAlgorithm(solver);
    
    //set vertex for every pose 
    for (int i=0;i<pvkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        vSE3->setEstimate(convert2SE3Quart(pvkf[i]->kfT_w_c));//add quaternion 
        
        vSE3->setId(pvkf[i]->kfid_);
        
        vSE3->setFixed(pvkf[0]->kfid_==0);
   
        optimiser_.addVertex(vSE3);

       
        
   
   
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
        optimiser_.addVertex(vPoint);
        }
        
    //     {   
    //     g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
    //     Eigen::Vector2d obs;

    //     obs=Eigen::Vector2d(pvkf[i]->vkfdps_[j].first.first,pvkf[i]->vkfdps_[j].first.second);
    //     uv.push_back(coord(pvkf[i]->vkfdps_[j].first.first,pvkf[i]->vkfdps_[j].first.second));
    //         //todo mesurement 
            
    //     e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(id)));
    //     e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(i)));
    //         // optimiser_.vertices().find(i)->second;

    //     e->setMeasurement(obs);

    //         // const float 
    //     Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
    //     e->setInformation(info);

    //         e->fx=226.38018519795807;
    //         e->fy=226.15002947047415;
    //         e->cx=173.6470807871759;
    //         e->cy=133.73271487507847;

    //         optimiser_.addEdge(e);    
    //     }
       

    
    // optimiser_.initializeOptimization();
    // //optimiser_.setVerbose(true);
    // optimiser_.optimize(5);

    int idx=pvkf.size()+1;
    for(int i=0;i< pvkf.size();i++){
    g2o::VertexSE3Expmap *vSE3_r =  dynamic_cast<g2o::VertexSE3Expmap*>(optimiser_.vertex(i)); 
    
    Eigen::Matrix3d rotation_op= vSE3_r->estimate().rotation().toRotationMatrix();
   
    Eigen::Vector3d pose_op= Eigen::Vector3d(vSE3_r->estimate().translation().x(),
                                            vSE3_r->estimate().translation().y(),
                                            vSE3_r->estimate().translation().z()) ;
    Eigen::Matrix4d T_op;
    T_op.block<3,3>(0,0)=rotation_op;T_op.block<3,1>(0,3)=pose_op;
    pvkf[i]->kfT_w_c= T_op;//push_back(pose_op);

    // for(int j=0;j<kfscontain[j];j++){
    //     g2o::VertexSBAPointXYZ *v_point_r = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimiser_.vertex(idx)); 
    //     Eigen::Vector3d t_op=Eigen::Vector3d(v_point_r->estimate().x(),v_point_r->estimate().y(),v_point_r->estimate().z()); 
        
    //     pvkf[i]->vkfdps_[j]=std::make_pair(uv[idx-pvkf.size()-1],t_op);

    // }
    }

}

void optimizer::banbenvdp(std::deque<KeyFrame> & vkf){
    assert(!vkf.empty());
    g2o::SparseOptimizer  optimiser_;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver_;

    linearSolver_=new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *pSolver=new g2o::BlockSolver_6_3(linearSolver_);
    g2o::OptimizationAlgorithmLevenberg *solver =new g2o::OptimizationAlgorithmLevenberg(pSolver);

    optimiser_.setAlgorithm(solver);
    
    int id=vkf.size()+1;
    //set vertex for every pose 
    for (int i=0;i<vkf.size();i++){
        g2o::VertexSE3Expmap *vSE3=new g2o::VertexSE3Expmap();
        vSE3->setEstimate(convert2SE3Quart(vkf[i].kfT_w_c));//add quaternion 
        
        vSE3->setId(vkf[i].kfid_);
        vSE3->setFixed(vkf[0].kfid_==0);
        optimiser_.addVertex(vSE3);
        // ROS_INFO("vertice node1");
        //add vertex map points 
    }   
        
        for(int i=0;i<vkf.size();i++)
        { 
        for (int j=0;j<vkf[i].kfvdp_.size();j++)//smp points vector 3d
        {
        g2o::VertexSBAPointXYZ *vPoint=new g2o::VertexSBAPointXYZ();
        
        Eigen::Vector3d p_3d=vkf[i].kfvdp_[j].p_world();

        // Eigen::Vector3d p_3d=  vkf[i].getRotationMatrix() * vkf[i].vkfdps_[j].second +vkf[i].getTranslation();
        
        vPoint->setEstimate(p_3d);
        //map points id 
        id++;

        vPoint->setId(id);
        
        vPoint->setMarginalized(true);
        optimiser_.addVertex(vPoint);
        // ROS_INFO("vertice node2");
        // int map_count;

        // std::cout<<"observation.size()=="<<vkf[i].nieba_[&vkf[i].kfvdp_[j]].size()<<"\t ";
            if(!vkf[i].nieba_[&vkf[i].kfvdp_[j]].size()) 
            {
            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;

            obs<<vkf[i].kfvdp_[j].col(),vkf[i].kfvdp_[j].row();
            // obs=Eigen::Vector2d(vkf[i].vkfdps_[j].first.first,vkf[i].vkfdps_[j].first.second);
            
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(id)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(i)));

            e->setMeasurement(obs);

            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser_.addEdge(e);        
            continue;//j++;
            }
            
            for(int k=0;k<vkf[i].nieba_[&vkf[i].kfvdp_[j]].size();k++)
            {
            // std::cout<<"observation size =="<<vkf[i].nieba_[&vkf[i].kfvdp_[j]].size()<<"\t";
            //assert(vkf[i].nieba_[&vkf[i].kfvdp_[j]].size()!=0);
            //  for(int k=0;k<vkf[i].nieba_[&vkf[i].kfvdp_[j]].size();k++){    
        
            int observation_id=vkf[i].nieba_[&vkf[i].kfvdp_[j]][k];
            // int edge_kf_id=vkf[i].kfvdp_[j].GetObservation()[k];
            // int observation_id=vkf[i].nieba_.at(& vkf[i].kfvdp_[j])[k];
            g2o::EdgeSE3ProjectXYZ *e= new g2o::EdgeSE3ProjectXYZ();
            Eigen::Vector2d obs;

            obs<<vkf[i].kfvdp_[j].col(),vkf[i].kfvdp_[j].row();
            // obs=Eigen::Vector2d(vkf[i].vkfdps_[j].first.first,vkf[i].vkfdps_[j].first.second);
            
            
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(id)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimiser_.vertex(observation_id)));

            e->setMeasurement(obs);

            // const float 
            Eigen::Matrix2d info=Eigen::Matrix2d::Identity();
            e->setInformation(info);

            e->fx=fx_;
            e->fy=fy_;
            e->cx=cx_;
            e->cy=cy_;

            optimiser_.addEdge(e);
            }
        
            }
        }
    optimiser_.initializeOptimization();
    optimiser_.setVerbose(true);
    optimiser_.optimize(10);
    resultLog(optimiser_,vkf,true);
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

void optimizer::resultLog(g2o::SparseOptimizer  &optimizor,const std::deque<KeyFrame> &vkf,bool chech){
if (!chech) return ;
std::string resultDir="/tmp/opTest.txt";

LOG(INFO) << "Saving trajectory to " << resultDir << " ......";

  std::ofstream  f;
  f.open(resultDir.c_str(), std::ofstream::out);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << resultDir << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;
    int id=0;
    while (id<vkf.size())
    {
    g2o::VertexSE3Expmap *vSE3_r =  dynamic_cast<g2o::VertexSE3Expmap*>(optimizor.vertex(id)); 

    Eigen::Quaterniond qd= vSE3_r->estimate().rotation();

    Eigen::Vector3d landmark= Eigen::Vector3d(vSE3_r->estimate().translation().x(),
                                              vSE3_r->estimate().translation().y(), 
                                              vSE3_r->estimate().translation().z());

    f << vkf[id].ts_.toSec() << " " << std::setprecision(9) << landmark.transpose() << " "
      << qd.x() << " " << qd.y() << " " << qd.z() << " " << qd.w() << std::endl;
    
    id++;
    }

    f.close();
    LOG(INFO) << "Saving after OP trajectory to " << resultDir << ". Done !!!!!!.";
}
}//esvo_core
