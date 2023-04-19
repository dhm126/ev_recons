#ifndef ESVO_CORE_EVENTPROVIDER
#define ESVO_CORE_EVENTPROVIDER
#include <ros/ros.h>
#include <string>
#include <hdf5/serial/hdf5.h>
#include <hdf5/serial/H5Cpp.h>
#include <vector>
#include <thread>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <esvo_core/tools/params_helper.h>
using namespace std;
using namespace H5;

namespace esvo_core{
class eventProvider{
public:
eventProvider(ros::NodeHandle &nh,ros::NodeHandle &pnh);
~eventProvider();
void EventReadingloop();

void readH5Datasets(string file_name ,string data_name,
                    vector<double > &data,
                    hsize_t hslab_offset, hsize_t hslab_count);
void readH5Datasets(string file_name ,string data_name,
                    vector<double > &data,
                    hsize_t hslab_offset, hsize_t &hslab_count,double time_limit);
void TimeSlicer(string file_name ,string data_name,
                    vector<double > &data,
                    hsize_t &hslab_offset);

void publishEvents(dvs_msgs::EventArrayPtr & msg);
private:
ros::NodeHandle nh_,nhp_;
ros::Publisher events_left_pub_;
ros::Publisher events_right_pub_;

string h5f_name_;
size_t sensor_height_;
size_t sensor_width_;


};
}

#endif