/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "karto_submap/karto_submap.h"

SlamKarto::SlamKarto() : got_map_(false)
{
  ROS_INFO_STREAM("\033[1;32m----> Karto build submap started.\033[0m");

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();
  last_rangScan_ = NULL;
  // 参数初始化
  InitParams();

  // Set up advertisements and subscriptions
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>(pub_submap_name_, 1, true);
  scan_sub_ = node_.subscribe(sub_scan_name_, 1, &SlamKarto::laserCallback, this);
  odom_sub_ = node_.subscribe(sub_odom_name_, 1, &SlamKarto::OdomCallBack, this);
}
void SlamKarto::OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double robot_time_stamps = msg->header.stamp.toSec();

  // to calculate the moving distance
  double orientation_x = msg->pose.pose.orientation.x;
  double orientation_y = msg->pose.pose.orientation.y;
  double orientation_z = msg->pose.pose.orientation.z;
  double orientation_w = msg->pose.pose.orientation.w;

  tf::Quaternion q_odom(orientation_x, orientation_y, orientation_z, orientation_w);
  double roll_odom, pitch_odom, yaw_odom; // theta_x, theta_y, theta_z
  tf::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

  if(use_real_param_)
    odomPose_ = karto::Pose2(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  else
    odomPose_ = karto::Pose2(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_odom);
  odomTime_ = msg->header.stamp.toSec();
  //  std::cout << "odom: " << odomPose_ << std::endl;
}

SlamKarto::~SlamKarto()
{
  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
}

// ros的参数初始化
void SlamKarto::InitParams()
{
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "robot_0/odom";
  if (!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "robot_0/map";
  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "robot_0/base_link";
  if (!private_nh_.getParam("pub_submap_name", pub_submap_name_))
    pub_submap_name_ = "robot_0/map";
  if (!private_nh_.getParam("sub_odom_name", sub_odom_name_))
    sub_odom_name_ = "robot_0/odom";
  if (!private_nh_.getParam("sub_scan_name", sub_scan_name_))
    sub_scan_name_ = "robot_0/scan";

  // mapper内部参数

  if (!private_nh_.getParam("minimum_time_interval", minimum_time_interval_))
    minimum_time_interval_ = 3600;
  mapper_->setParamMinimumTimeInterval(minimum_time_interval_);

  if (!private_nh_.getParam("minimum_travel_distance", minimum_travel_distance_))
    minimum_travel_distance_ = 0.2;
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance_);

  if (!private_nh_.getParam("minimum_travel_heading", minimum_travel_heading_))
    minimum_travel_heading_ = 0.2;
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading_);

  bool use_scan_matching;
  if (private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);

  bool do_loop_closing;
  if (private_nh_.getParam("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  int scan_buffer_size;
  if (private_nh_.getParam("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);
  
  double scan_buffer_maximum_scan_distance;
  if (private_nh_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  // 外部参数
  double tmp;
  if (!private_nh_.getParam("map_update_interval", tmp))
    tmp = 1;
  map_update_interval_.fromSec(tmp);

  private_nh_.param("use_scan_range", use_scan_range_, 12.0);

  if (!private_nh_.getParam("resolution", resolution_))
  {
    resolution_ = 0.05;
  }
  if (!private_nh_.getParam("max_queue_size", max_queue_size_))
  {
    max_queue_size_ = 10;
  }

  double now_to_last_dist;
  if (!private_nh_.getParam("now_to_last_dist", now_to_last_dist))
  {
    now_to_last_dist = 1.0;
  }
  now_to_last_size_ = now_to_last_dist / minimum_travel_distance_;

  if (!private_nh_.getParam("use_real_param", use_real_param_))
  {
    use_real_param_ = true;
  }
}

void SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{

  static ros::Time last_map_update(0, 0);
  ros::Time begin = ros::Time::now();
  // Check whether we know about this laser yet
  karto::LaserRangeFinder *laser = getLaser(scan);

  karto::Pose2 odom_pose;
  if (addScan(laser, scan, odom_pose) || !got_map_)
  {
    if (!got_map_ ||
        (scan->header.stamp - last_map_update) > map_update_interval_)
    {
     
      if (updateMap())
      {
        ros::Time end = ros::Time::now();
        std::cout << "submap time cost:" << end.toSec() - begin.toSec() << std::endl;
        last_map_update = scan->header.stamp;
        got_map_ = true;
        lasers_.clear();
        mapper_->Reset();
        dataset_->Clear();
      }
    }
  }
}

karto::LaserRangeFinder *SlamKarto::getLaser(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    karto::LaserRangeFinder *laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(scan->header.frame_id));
    laser->SetOffsetPose(karto::Pose2(0,
                                      0,
                                      0));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    laser->SetRangeThreshold(use_scan_range_);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool SlamKarto::addScan(karto::LaserRangeFinder *laser,
                        const sensor_msgs::LaserScan::ConstPtr &scan,
                        karto::Pose2 &karto_pose)
{
  if (!getOdomPose(karto_pose, scan->header.stamp))
    return false;
  // std::cout << "odom_pose: " << karto_pose << std::endl;

  // Create a vector of doubles for karto
  std::vector<kt_double> readings;
  // int cnt=0;
  for (std::vector<float>::const_iterator it = scan->ranges.begin();
       it != scan->ranges.end();
       ++it)
  {
    // if( cnt > 710)
    //   readings.push_back(0);
    // else
    if(*it > use_scan_range_)
      readings.push_back(use_scan_range_);
    else
      readings.push_back(*it);
    // cnt++;
   
  }

  // create localized range scan
  karto::LocalizedRangeScan *range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings);
  if (range_scan == NULL)
  {
    std::cout << laser->GetName() << readings.size() << std::endl;
  }
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  kt_bool processed;
  if (processed = myProcess(range_scan))
  {
    q_rangeScan_.push(range_scan);
    for (int i = 0; i < q_rangeScan_.size(); i++)
    {
      karto::LocalizedRangeScan *tmpScan = q_rangeScan_.front();
      if(i < max_queue_size_)
      {
        mapper_->Process(tmpScan);
      }
      q_rangeScan_.pop();
      q_rangeScan_.push(tmpScan);
    }

    if (q_rangeScan_.size() > max_queue_size_+now_to_last_size_)
    {
      q_rangeScan_.pop();
    }
  }
  else
  {
    delete range_scan;
  }
  return processed;
}

bool SlamKarto::getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t)
{
  if (fabs(t.toSec() - odomTime_) > 1.0)
    return false;
    
  karto_pose = odomPose_;
  return true;
}

bool SlamKarto::updateMap()
{
  karto::OccupancyGrid *occ_grid =
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if (!occ_grid)
  {
    // std::cout << "updateMap flase" << std::endl;
    return false;
  }

  if (!got_map_)
  {
    map_.map.info.resolution = resolution_;
    // std::cout << "resolution_" << resolution_ << std::endl;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();

  kt_int32s height = occ_grid->GetHeight();
  // std::cout << "width" << width << "\theight" << height << std::endl;
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();
  // std::cout << "offset" << offset.GetX() << "\toffset" << offset.GetY() << std::endl;
  if (map_.map.info.width != (unsigned int)width ||
      map_.map.info.height != (unsigned int)height ||
      map_.map.info.origin.position.x != offset.GetX() ||
      map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++)
  {
    for (kt_int32s x = 0; x < width; x++)
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
      case karto::GridStates_Unknown:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] =100;
        break;
      case karto::GridStates_Occupied:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        break;
      case karto::GridStates_Free:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
        break;
      default:
        ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
        break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;
  
  // if(q_rangeScan_.size() >= max_queue_size_)
  sst_.publish(map_.map);
  delete occ_grid;

  return true;
}

bool SlamKarto::myHasMovedEnough(karto::LocalizedRangeScan *pScan, karto::LocalizedRangeScan *pLastScan)
{
  // test if first scan
  if (pLastScan == NULL)
  {
    return true;
  }

  // test if enough time has passed
  double timeInterval = pScan->GetTime() - pLastScan->GetTime();
  if (timeInterval >= minimum_time_interval_)
  {
    return true;
  }

  karto::Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  karto::Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  // test if we have turned enough
  double deltaHeading = karto::math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
  if (fabs(deltaHeading) >= minimum_travel_heading_)
  {
    return true;
  }

  // test if we have moved enough
  double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
  if (squaredTravelDistance >= karto::math::Square(minimum_travel_distance_) - karto::KT_TOLERANCE)
  {
    return true;
  }

  return false;
}

bool SlamKarto::myProcess(karto::LocalizedRangeScan *pScan)
{
  if (pScan != NULL)
  {
    karto::LaserRangeFinder *pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    // get last scan
    karto::LocalizedRangeScan *pLastScan = last_rangScan_;

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!myHasMovedEnough(pScan, pLastScan))
    {
      return false;
    }
    last_rangScan_ = pScan;
    return true;
  }
  
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "karto_submap_node");

  SlamKarto kn;

  ros::spin();

  return 0;
}
