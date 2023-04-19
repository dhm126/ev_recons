#include <esvo_core/EventProvider.h>
using namespace esvo_core;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "event_provider");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //dl (nh, nh_private);
  eventProvider ep(nh,nh_private);
  
  ros::spin();
  return 0;
}