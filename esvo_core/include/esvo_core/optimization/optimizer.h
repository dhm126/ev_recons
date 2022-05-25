#ifndef ESVO_CORE_OPTIMIZER_H
#define ESVO_CORE_OPTIMIZER_H
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <deque>
#include <vector>
#include <map>
#include <esvo_core/tools/utils.h>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/KeyFrame.h>
#include <esvo_core/container/DepthPoint.h>

namespace esvo_core{
using namespace tools;
// using namespace core;
using namespace container;
class optimizer
{ 
    
public:
optimizer(
const CameraSystem::Ptr &camsysPtr 
);
virtual ~optimizer();

private:
    CameraSystem::Ptr camSysPtr_;
    bool bResultlog_;
    double fx_,fy_,cx_,cy_;
    
 public:
    void BundleAdjustment(const std::deque <KeyFrame> &vkf);
    void POptimization( std::deque<KeyFrame*> pvkf,std::vector< DepthPoint*> vdps);
    void resultLog(g2o::SparseOptimizer  &optimizor,const std::deque<KeyFrame> &vkf,bool chech);
    g2o::SE3Quat convert2SE3Quart(const Eigen::Matrix4d & T_c_w);
    void banbenvdp(std::deque<KeyFrame> &vkf);
    
    
};
//global parameters for optimzization 
}//namespace esvo_core 
#endif