#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <opencv2/features2d.hpp>
#include <DBoW3/DBoW3.h>
#include <opencv2/calib3d.hpp>

#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h> 
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <esvo_core/tools/fsolver.h>

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/tools/params_helper.h>
#include <geometry_msgs/PoseStamped.h>
#include <esvo_core/tools/utils.h>
#include <esvo_core/tools/fsolver.h>

#include <map>
#include <mutex>


namespace esvo_core{

using namespace esvo_core;
using namespace tools;
// class FSolver;
class dl{
public:
dl(const ros::NodeHandle& nh,const ros::NodeHandle &nhp); 
~dl();//shut down publisher

private:
ros::NodeHandle nh_,pnh_;

DBoW3::Vocabulary my_voc_;
DBoW3::Database my_database_;
std::string  voc_file;
DBoW3::BowVector last_bowvec_;
double maxScore_;
std::ofstream voc_score;
int close_match;

container::CameraSystem::Ptr camSysPtr_;
std::string calibFile_;
cv::Mat camera_Matrix_;
bool bkfIns_;
// cv::Mat m_images_old;
std::vector<cv::Mat > m_image_descriptors;
// DBoW3::BowVector last_bowvec_;
std::vector<std::vector<cv::KeyPoint>> m_image_keys;

std::map<int,ros::Time > mSudaro_;
//callback function 
public:
//void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg);
void accumulatedEventCallback(const sensor_msgs::ImageConstPtr &msg);//
void KeyframeInsertionCallback(const std_msgs::BoolConstPtr & msg);
// void eventsCallback(const cv::Mat &eventMap);
//subscriber

ros::Subscriber event_sub_;
ros::Subscriber acumulatedEvents_sub_;
ros::Subscriber KeyFrame_sub_;
//publiser
ros::Publisher obs_pub_,pose_pub_;
image_transport::Publisher image_pub;

std::vector<dvs_msgs::Event> events_;

// image_transport::Publisher ;
private:
enum DetectionStatus{
LOOPDETECTED,
CLOSE_FRAMES,//not too much pictures 
DATABASE_NO_RESULT,//query no result in database
DATABASE_LOW_SCORE,//removescores
NO_GROUP,//cannot compute islands
NO_TEMPORAL_CONSISTENCY,//check conssitency in islands 
NO_GEOMETRY_CONSISTENCY,//gemometry check via flann for query and match
MATCH_TOO_CLOSE//matching result too close maybe a bug

};

struct island{


DBoW3::EntryId first_id;
DBoW3::EntryId last_id;

double score;//compute island

DBoW3::EntryId best_entry;

double best_score;

island(){}

island(DBoW3::EntryId f,DBoW3::EntryId l):first_id(f),last_id(l){}

island(DBoW3::EntryId f,DBoW3::EntryId l,double s):first_id(f),last_id(l),score(s){}

inline bool operator < (const island &b) const
    {
      return this->score < b.score;
    }
    
    /**
     * Says whether this score is greater than the score of another island
     * @param b
     * @return true iff this score > b.score
     */
    inline bool operator > (const island &b) const
    {
      return this->score > b.score;
    }
    
    /** 
     * Returns true iff a > b
     * This function is used to sort in descending order
     * @param a
     * @param b
     * @return a > b
     */
    static inline bool gt(const island &a, const island &b)
    {
      return a.score > b.score;
    }
        
    /**
     * Returns true iff entry ids of a are less then those of b.
     * Assumes there is no overlap between the islands
     * @param a
     * @param b
     * @return a.first < b.first
     */
    static inline bool ltId(const island &a, const island &b)
    {
      return a.first_id < b.first_id;
    }
    
    /**
     * Returns the length of the island
     * @return length of island
     */
    inline int length() const { return last_id - first_id + 1; }
    
    /**
     * Returns a printable version of the island
     * @return printable island
     */
    std::string toString() const
    {
      std::stringstream ss;
      ss << "[" << first_id<< "-" << last_id<< ": " << score << " | best: <"
        << best_entry << ": " << best_score << "> ] ";
      return ss.str();
    }
};

struct tTemporalWindow
  {
    /// Island matched in the last query
    island last_matched_island;
    /// Last query id
    DBoW3::EntryId last_query_id;
    /// Number of consistent entries in the window
    int nentries;
    
    /**
     * Creates an empty temporal window
     */
    tTemporalWindow(): nentries(0) {}
  };
/**
 @param match 经过loopdetect的结果 id
 @param status loopdetect 状态结果
 @param query 当前所遍历到的图像进口id
*/
struct DetectionResult{

  DetectionStatus status;
  
  DBoW3::EntryId match;
  DBoW3::EntryId query;

 inline bool detection() const {
   return status==LOOPDETECTED;
 }
};

struct parameters {

int dislocal ;

//void parameters();
};

public:
std::vector<cv::Mat> images_;
int NUM_IMAGE;
int image_total_length;

tTemporalWindow window_;

private:

// bool dataTransfer();
void publishPose( Eigen::Matrix4d &tcw,const ros::Time &t_lc);
void addFrametoLd();

//void detecting();//std::promise<void> detecting_promise);

void extract(const cv::Mat &img , std::vector<cv::KeyPoint> &kps,  cv::Mat &desc);

bool loopdetect(const std::vector<cv::KeyPoint> &kps ,const cv::Mat &descriptors,DetectionResult &match);

void removeLowsocres(DBoW3::QueryResults & q,double minScore);

void updateTemporalWindow(const island & ile,DBoW3::EntryId ids);

void computeIslands(DBoW3::QueryResults & qr,std::vector<island> & islands);

double calculateIslandScore(const DBoW3::QueryResults &wqr,int first,int last);

int  getConsistentEntries(void);

void SetFlann( cv::FlannBasedMatcher &flann, const cv::Mat & descriptors);

bool isGeometricallyConsistent_Flann  (const DBoW3::EntryId old_entry,  const std::vector<cv::KeyPoint> &keys, const cv::Mat  & descriptors,  cv::FlannBasedMatcher &flann_structure);

void getMatches_neighratio(
  const std::vector<cv::Mat> &A, const std::vector<unsigned int> &i_A,
  const std::vector<cv::Mat> &B, const std::vector<unsigned int> &i_B,
  std::vector<unsigned int> &i_match_A, std::vector<unsigned int> &i_match_B);

double distance(const cv::Mat &a, const cv::Mat &b);

Eigen::Matrix4d calculateRelativePose(const std::vector<cv::KeyPoint> &kps1,const std::vector <cv::KeyPoint> &kps2,const int  &e_id, const int &q_id );

void publishMap(const cv::Mat & image);

};//loop detector loop

}//namesapce esvo_core