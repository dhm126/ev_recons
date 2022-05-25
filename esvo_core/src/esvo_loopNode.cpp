#include <esvo_core/loopdetector.h>
using namespace esvo_core;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Loopclosing");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dl loopdetector(nh, nh_private);
  ros::spin();
  return 0;
}

