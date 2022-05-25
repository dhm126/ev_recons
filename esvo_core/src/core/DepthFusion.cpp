#include <esvo_core/core/DepthFusion.h>
#include <thread>
#include <functional>

namespace esvo_core
{
namespace core
{
DepthFusion::DepthFusion(
  CameraSystem::Ptr &camSysPtr,
  std::shared_ptr<DepthProblemConfig> & dpConfigPtr):
  camSysPtr_(camSysPtr),
  dpConfigPtr_(dpConfigPtr){}

DepthFusion::~DepthFusion() {}
//根据T分布得到概率分布后的深度点 invdepth 尺度 var 自由度 
//T_prior作dp_prior的投影投到像素平面上去
bool
DepthFusion::propagate_one_point(
  DepthPoint &dp_prior,//from 2d world_frame
  DepthPoint &dp_prop,//to 2d  frame_frame
  Eigen::Matrix<double, 4, 4> &T_prop_prior)//经过Other矩阵T 传播到current cam position
{
  Eigen::Vector3d p_prop = T_prop_prior.block<3, 3>(0, 0) * dp_prior.p_cam() +
                           T_prop_prior.block<3, 1>(0, 3);//3X3 X3x1 +t 3x1  RP+t= 

  Eigen::Vector2d x_prop;//相机坐标系 z=1 T_prop frame point
  camSysPtr_->cam_left_ptr_->world2Cam(p_prop, x_prop);//left camera 事件时间戳的像素平面  
  if (!boundaryCheck(x_prop(0), x_prop(1),//check out of boundry
                     camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
    return false;

  // create a depth point with propagated attributes.
  size_t row = std::floor(x_prop(1));//x_prop至少为一个float
  size_t col = std::floor(x_prop(0));
  dp_prop = DepthPoint(row, col);
  dp_prop.update_x(x_prop);

  // compute the new inverse depth 在变幻后的深度分布
  double invDepth = 1.0 / p_prop(2);//1/z

  // compute the jacobian
  double denominator = T_prop_prior.block<1,2>(2,0) * dp_prior.p_cam().head(2) + T_prop_prior(2, 3);//1x2 x 2x1 +(row,col)
  denominator /= dp_prior.p_cam()(2);//1/z *de
  denominator += T_prop_prior(2,2);//
  double J = T_prop_prior(2,2) / pow(denominator, 2);//1/J

  // propagation
  double variance, scale2, nu;
  if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
  {
    variance = J * J * dp_prior.variance();
    dp_prop.update(invDepth, variance);
  }
  else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
  {
    scale2 = J * J * dp_prior.scaleSquared();//s 经过仿射变换重新秋在变换后的深度分布
    nu = dp_prior.nu();//dof
    variance = nu / (nu - 2) * scale2;//var
    dp_prop.update_studentT(invDepth, scale2, variance, nu);// T dist
  }
  else
    exit(-1);

  dp_prop.update_p_cam(p_prop);//
  dp_prop.residual() = dp_prior.residual();//
  dp_prop.age() = dp_prior.age();
  return true;
}

int
DepthFusion::update(
  std::vector<DepthPoint> &dp_obs,//dqv_depthPoint deque<vector> 当前 dp
  DepthFrame::Ptr &df,//depthFrame result pub
  int fusion_radius)// 传播范围
{
  int numFusion = 0;
  //像素到世界坐标系
  //当前相机对世界坐标系
  Eigen::Matrix<double, 4, 4> T_frame_world = df->T_world_frame_.inverse().getTransformationMatrix();//df为当前TS时间戳建立的dp点集合
  
  for (size_t i = 0; i < dp_obs.size(); i++)
  {
    Eigen::Matrix<double, 4, 4> T_frame_obs = T_frame_world * dp_obs[i].T_world_cam();//T   传播坐标系到dpobs坐标系 
    DepthPoint dp_prop;
    if (!propagate_one_point(dp_obs[i], dp_prop, T_frame_obs))//传播frame to obs
      continue;
    numFusion += fusion(dp_prop, df->dMap_, fusion_radius);//dp_prop 和dp_prior融合 存放到df中
  }
  return numFusion;
}

int
DepthFusion::fusion(//传播的x_prop，一般为非整数坐标：取整之后丧失了进度
  DepthPoint &dp_prop,
  DepthMap::Ptr &dm,//grid fushion_radius为0时为周围2x2 gird。反之为(fr*2+1)^2 grid，但值计算当前值的-101三个位置
  int fusion_radius)
{
  int numFusion = 0;
  // get neighbour pixels involved in fusion
  std::vector<std::pair<size_t, size_t> > vpCoordinate;// pair: <row, col>
  if(fusion_radius == 0)
  {
    const size_t patchSize = 4;
    vpCoordinate.reserve(patchSize);//2x2
    size_t row_topleft = dp_prop.row();// 像素坐标系下坐标 
    size_t col_topleft = dp_prop.col();//
    for(int dy = 0; dy <= 1; dy++)//往右下两像素位置查找
      for(int dx = 0; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_topleft + dy, col_topleft + dx));//vpcoordiante dp_prop.x dp_prop.y dp.y+1 dp.x+1
  }
  else
  {
    const size_t patchSize = (2*fusion_radius+1) * (2*fusion_radius+1);//如果是3X3的话fusion_radius应该是1
    vpCoordinate.reserve(patchSize);//预留sqr(2n+1) 个深度点坐标
    size_t row_centre = dp_prop.row();
    size_t col_centre = dp_prop.col();
    for(int dy = -1; dy <= 1; dy++)
      for(int dx = -1; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_centre + dy, col_centre + dx));// row center + -1/0/1
  }
  // fusion vpcoor.size=patch_size
  for(size_t i = 0; i < vpCoordinate.size(); i++)//每个dp共计4个可以改变的位置（0，0）（0，1）（1，0）（1，1）
  {
    size_t row = vpCoordinate[i].first;
    size_t col = vpCoordinate[i].second;
    if(!boundaryCheck(col, row, camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
      continue;

    // case 1: non-occupied  将传播的深度点直接占位
    
    if (!dm->exists(row, col))//
    {
      DepthPoint dp_new(row, col);
      if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        dp_new.update(dp_prop.invDepth(), dp_prop.variance());//赋  逆深度 方差
      else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      {
        //T分布概率模型 赋值
        dp_new.update_studentT(dp_prop.invDepth(), dp_prop.scaleSquared(), dp_prop.variance(), dp_prop.nu());
      }
      else
        exit(-1);

      dp_new.residual() = dp_prop.residual();
      dp_new.age() = dp_prop.age();
      Eigen::Vector3d p_cam;
      camSysPtr_->cam_left_ptr_->cam2World(dp_new.x(), dp_prop.invDepth(), p_cam);//将dp_new像素坐标转化为世界坐标下点p_cam
      dp_new.update_p_cam(p_cam);

      dm->set(row, col, dp_new);//更新depthmap
    }
    else// case 2: occupied 检查是否能够融合前深度点和传播的深度点
    {
      bool bCompatibility = false;
      if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        bCompatibility = chiSquareTest(dp_prop.invDepth(), dm->at(row, col).invDepth(),
                                       dp_prop.variance(), dm->at(row, col).variance());
      else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      {//d1 d2 var1 var2
        bCompatibility = studentTCompatibleTest(
          dp_prop.invDepth(), dm->at(row, col).invDepth(),dp_prop.variance(), dm->at(row, col).variance());
      }
      else
        exit(-1);

      // case 2.1 compatible 深度点可以兼容
      if (bCompatibility)
      {
        if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
          dm->get(row, col).update(dp_prop.invDepth(), dp_prop.variance());
        else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
          dm->get(row, col).update_studentT(dp_prop.invDepth(), dp_prop.scaleSquared(), dp_prop.variance(), dp_prop.nu());
        else
          exit(-1);

        dm->get(row, col).age()++;//融合到之前传播的dp——prop上
        dm->get(row, col).residual() = min(dm->get(row, col).residual(), dp_prop.residual());
        Eigen::Vector3d p_update;
        camSysPtr_->cam_left_ptr_->cam2World(dm->get(row, col).x(), dp_prop.invDepth(), p_update);
        dm->get(row, col).update_p_cam(p_update);
        numFusion++;
      }
      else // case 2.2 not compatible 
      {
        // consider occlusion (the pixel is already assigned with a point that is closer to the camera) 旧点把新点遮住了
        if (dm->at(row, col).invDepth() - 2 * sqrt(dm->at(row, col).variance()) > dp_prop.invDepth())
          continue;
        if (dp_prop.variance() < dm->at(row, col).variance()
            && dp_prop.residual() < dm->at(row, col).residual()) //&& other requirement? such as cost?
        {
          dm->get(row, col) = dp_prop;
        }
      }
    }
  }
  return numFusion;
}

bool
DepthFusion::boundaryCheck(
  double xcoor,
  double ycoor,
  size_t width,
  size_t height)
{
  if (xcoor < 0 || xcoor >= width || ycoor < 0 || ycoor >= height)
    return false;
  else
    return true;
}

bool
DepthFusion::chiSquareTest(
  double invD1, double invD2,
  double var1, double var2)
{
  double delta_d_squared = std::pow(invD1 - invD2, 2);
  double compatibility = delta_d_squared / var1 + delta_d_squared / var2;
  if (compatibility < 5.99)
    return true;
  else
    return false;
}

bool
DepthFusion::studentTCompatibleTest(
  double invD1, double invD2,
  double var1, double var2)
{
  double stdvar1 = sqrt(var1);
  double stdvar2 = sqrt(var2);
  double diff = fabs(invD1 - invD2);//float abs
  if(diff < 2*stdvar1 || diff < 2*stdvar2)
    return true;
  return false;
}

void
DepthFusion::naive_propagation(
  std::vector<DepthPoint> &dp_obs,
  DepthFrame::Ptr &df)
{
  Eigen::Matrix<double, 4, 4> T_frame_world = df->T_world_frame_.inverse().getTransformationMatrix();
  for (size_t i = 0; i < dp_obs.size(); i++)
  {
    Eigen::Matrix<double, 4, 4> T_frame_obs = T_frame_world * dp_obs[i].T_world_cam();
    DepthPoint dp_prop;
    if (!naive_propagate_one_point(dp_obs[i], dp_prop, T_frame_obs))
      continue;

    // determine the four neighbouring pixels
    std::vector<std::pair<size_t, size_t> > vpCoordinate;
    const size_t patchSize = 4;
    vpCoordinate.reserve(patchSize);
    size_t row_topleft = dp_prop.row();
    size_t col_topleft = dp_prop.col();
    for(int dy = 0; dy <= 1; dy++)
      for(int dx = 0; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_topleft + dy, col_topleft + dx));

    // naive propagation
    for(size_t i = 0; i < vpCoordinate.size(); i++)
    {
      size_t row = vpCoordinate[i].first;
      size_t col = vpCoordinate[i].second;
      if(!boundaryCheck(col, row, camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
        continue;

      // case 1: non-occupied
      if (!df->dMap_->exists(row, col))
      {
        DepthPoint dp_new(row, col);
        dp_new.update(dp_prop.invDepth(), dp_prop.variance());
        dp_new.residual() = dp_prop.residual();
        dp_new.age() = dp_prop.age();
        Eigen::Vector3d p_cam;
        camSysPtr_->cam_left_ptr_->cam2World(dp_new.x(), dp_prop.invDepth(), p_cam);
        dp_new.update_p_cam(p_cam);
        df->dMap_->set(row, col, dp_new);
      }
      else// case 2: occupied
      {
        if(df->dMap_->at(row, col).invDepth() > dp_prop.invDepth()) // dp_prop is further
          continue;
        else
        {
          if ( dp_prop.residual() < df->dMap_->at(row, col).residual())
            df->dMap_->get(row, col) = dp_prop;
        }
      }
    }
  }
}

bool
DepthFusion::naive_propagate_one_point(
  DepthPoint &dp_prior,
  DepthPoint &dp_prop,
  Eigen::Matrix<double, 4, 4> &T_prop_prior)//传播的公共时间t
{
  Eigen::Vector3d p_prop = T_prop_prior.block<3, 3>(0, 0) * dp_prior.p_cam() +
                           T_prop_prior.block<3, 1>(0, 3);

  Eigen::Vector2d x_prop;
  camSysPtr_->cam_left_ptr_->world2Cam(p_prop, x_prop);
  if (!boundaryCheck(x_prop(0), x_prop(1),
                     camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
    return false;

  // create a depth point with propagated attributes.
  size_t row = std::floor(x_prop(1));
  size_t col = std::floor(x_prop(0));
  dp_prop = DepthPoint(row, col);
  dp_prop.update_x(x_prop);

  // compute the new inverse depth
  double invDepth = 1.0 / p_prop(2);

  // compute the jacobian
  double denominator = T_prop_prior.block<1,2>(2,0) * dp_prior.p_cam().head(2) + T_prop_prior(2, 3);
  denominator /= dp_prior.p_cam()(2);
  denominator += T_prop_prior(2,2);
  double J = T_prop_prior(2,2) / pow(denominator, 2);

  // propagation
  double variance = J * J * dp_prior.variance();
  dp_prop.update(invDepth, variance);
  dp_prop.update_p_cam(p_prop);
  dp_prop.residual() = dp_prior.residual();
  dp_prop.age() = dp_prior.age();
  return true;
}

}// namespace core
}// namespace esvo_core