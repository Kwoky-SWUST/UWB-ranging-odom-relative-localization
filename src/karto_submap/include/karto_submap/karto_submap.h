#ifndef KARTO_SUBMAP_H_
#define KARTO_SUBMAP_H_

#include "ros/ros.h"
#include "tf/tf.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include "open_karto/Mapper.h"

#include <string>
#include <map>
#include <vector>

#include "nav_msgs/Odometry.h"

// 从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
public:
    SlamKarto();
    ~SlamKarto();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
private:
    void InitParams();
    bool getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t);
    karto::LaserRangeFinder *getLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
    bool addScan(karto::LaserRangeFinder *laser,
                 const sensor_msgs::LaserScan::ConstPtr &scan,
                 karto::Pose2 &karto_pose);
    bool updateMap();
    bool myHasMovedEnough(karto::LocalizedRangeScan *pScan, karto::LocalizedRangeScan *pLastScan);
    bool myProcess(karto::LocalizedRangeScan *pScan);

    // ROS handles
    ros::NodeHandle node_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    karto::Pose2 odomPose_;
    double odomTime_;
    ros::Publisher sst_;

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
  

    karto::Mapper *mapper_;
    karto::Dataset *dataset_;
    std::map<std::string, karto::LaserRangeFinder *> lasers_;

    // Internal state
    bool got_map_;

    std::queue< karto::LocalizedRangeScan * > q_rangeScan_;
    ros::Duration map_update_interval_;

    karto::LocalizedRangeScan * last_rangScan_;
    double minimum_time_interval_;
    double minimum_travel_distance_;
    double minimum_travel_heading_;
    int max_queue_size_;
    int now_to_last_size_;
    int throttle_scans_;
    double resolution_;
    double use_scan_range_;

    std::string pub_submap_name_;
    std::string sub_odom_name_;
    std::string sub_scan_name_;
    bool use_real_param_;
};

#endif
