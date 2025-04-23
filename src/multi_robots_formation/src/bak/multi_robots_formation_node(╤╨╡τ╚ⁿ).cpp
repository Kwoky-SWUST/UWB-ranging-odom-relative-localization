#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include "vfhdriver.h"
#include "vfh_algorithm.h"
#include "gnuplot.h"
#include "text/to_string.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen> 
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include <g2o/core/factory.h>
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam2d/parameter_se2_offset.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <pthread.h>

using namespace g2o;
ros::Publisher robot2_pub;
ros::Publisher robot3_pub;
double detect_R = 0.8;
const double INITIAL1_X = 0;
const double INITIAL1_Y = 0;
const double INITIAL1_YAW = 0;
const double INITIAL2_X = -1;
const double INITIAL2_Y = 1;
const double INITIAL2_YAW = 0;
const double INITIAL3_X = -1;
const double INITIAL3_Y = -1;
const double INITIAL3_YAW = 0;


#define FORMATION_ROBOT2
#define FORMATION_ROBOT3
#define BIZHANG_ENABLE
// #define SIMULATION_ENABLE

class RobotPose{
public:
  RobotPose() {}
  double x;
  double y;
  double yaw;
  double timestamp;
};


RobotPose obstacle1_pose;
RobotPose obstacle2_pose;
RobotPose obstacle3_pose;

std::map<std::string, std::vector<RobotPose> > m_v_global_pose;

double carmen_normalize_theta(double theta)
{
  int multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;
  multiplier = (int)(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

GnuplotInterface * plot_global_pose = new GnuplotInterface();
void plotGlobalPose(int minX=-8, int maxX=15, int minY=-8, int maxY=20)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'Robot Global Track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
    for(it_amcl=m_v_global_pose.begin(); it_amcl!=m_v_global_pose.end(); it_amcl++)
    {
        std::string line_colour; 
        std::string line_type;
        if(it_amcl->first == "robot1")
        {
            line_colour = "#FF0000";
            line_type = "1";
        }  
        else if(it_amcl->first == "robot2")
        {
            line_colour = "#0000FF";
            line_type = "3";
        } 
        else if(it_amcl->first == "robot3")
        {
            line_colour = "#000000";
            line_type = "5";
        }
        else
        {
            continue;
        } 
        cmd+="'-' u 1:2 w lp lw 0.5 pt "+ line_type +" ps 0.5 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"',";
    }
    cmd += "\n";
    for(it_amcl=m_v_global_pose.begin(); it_amcl!=m_v_global_pose.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	
	plot_global_pose->commandStr( cmd );
}
g2o::SE3Quat computePoseToG2oSE3(RobotPose pose)
{
    Eigen::Vector3d _trans(pose.x, pose.y,0);
    Eigen::AngleAxisd roll = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond _q = roll * pitch * yaw;
    g2o::SE3Quat _se3(_q, _trans);
    return _se3;
}
RobotPose computeG2oSE3ToPose(g2o::SE3Quat se3)
{
    Eigen::Vector3d _trans = se3.translation();
    Eigen::Quaterniond _q = se3.rotation();
    Eigen::Matrix3d temp_mat=_q.toRotationMatrix();
	  Eigen::Vector3d euler = temp_mat.eulerAngles(0,1,2); 
    RobotPose pose;
    pose.x = _trans.x();
    pose.y = _trans.y();
    pose.yaw = euler[2];
    return pose;
}
double computeTheta(double theta, double rotation1)
{
  // theta = theta%(2*M_PI);
  if(theta > M_PI)
  {
    theta = theta - 2*M_PI;
  }
  // rotation1 = rotation1%(2*M_PI);
  if(rotation1 > M_PI)
  {
    rotation1 = rotation1 - 2*M_PI;
  }
  double w = theta - rotation1;
  return w;
}

std::vector<double> v_laserScan1;
geometry_msgs::Twist currentVel1;
std::vector<double> v_laserScan2;
geometry_msgs::Twist currentVel2;
std::vector<double> v_laserScan3;
geometry_msgs::Twist currentVel3;

geometry_msgs::Twist vel1_msg;
geometry_msgs::Twist vel2_msg;

vfh::VfhDriver *vfhDriver1 = new vfh::VfhDriver("/home/dzy/formation_ws/src/multi_robots_formation/config/robot2VFHConfig.txt");
vfh::VfhDriver *vfhDriver2 = new vfh::VfhDriver("/home/dzy/formation_ws/src/multi_robots_formation/config/robot2VFHConfig.txt");

std::map<std::string, RobotPose> m_rel_pose;
void robot1PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(v_laserScan2.size() == 0 || v_laserScan3.size() == 0) return;
  double timestamp = msg->header.stamp.toSec();
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  msg->pose.pose.position.z;
  m_v_global_pose["robot1"].push_back(pose);


  g2o::SE3Quat robot1_se3 = computePoseToG2oSE3(pose);

#ifdef FORMATION_ROBOT2
  // robot2 control
  if(m_v_global_pose.find("robot2") != m_v_global_pose.end())
  {
    g2o::SE3Quat robot2_se3 = computePoseToG2oSE3(m_v_global_pose["robot2"].back());
    // 1. relative positioning 
    RobotPose rel_12;
    g2o::SE3Quat rel_12_se3;
   
    rel_12_se3 = robot1_se3.inverse() * robot2_se3;
    rel_12 = computeG2oSE3ToPose(rel_12_se3);

    std::cout << "odom: " << rel_12.x << "\t" <<  rel_12.y << "\t" << rel_12.yaw <<  std::endl;
  
    // 2. set a goal;
    RobotPose goal1; 
    goal1.x = INITIAL2_X;
    goal1.y = INITIAL2_Y;
    goal1.yaw = INITIAL2_YAW;
  
    // 3. compute the diff of goal to rel
    g2o::SE3Quat diff1_se3 = rel_12_se3.inverse() * computePoseToG2oSE3(goal1);
    RobotPose diff1 = computeG2oSE3ToPose(diff1_se3);

    // 4. publish robot2/cmd_vel -> diff1 and robot3/cmd_vel -> diff2
    if(fabs(timestamp - m_v_global_pose["robot2"].back().timestamp) < 2)
    {
      double d = sqrt(diff1.x*diff1.x + diff1.y*diff1.y);
  
      double theta = atan2(diff1.y, diff1.x);  
      // 1.避障控制
      double r = sqrt(obstacle2_pose.x*obstacle2_pose.x + obstacle2_pose.y*obstacle2_pose.y);
#ifdef BIZHANG_ENABLE
      if(r < detect_R) // 避障 
      {
        // std::cout << "obstacle: ";
        geometry_msgs::Twist goal;
        goal.linear.x = d;
        goal.angular.z = theta;
        vel1_msg = vfhDriver1->approachGoalCommand(0.1, goal, currentVel2, v_laserScan2);
        double PI = d/0.4;
        if(PI > 1) PI = 1;
        vel1_msg.linear.x  = PI*vel1_msg.linear.x;
        vel1_msg.angular.z = PI*vel1_msg.angular.z;
      }
      else 
#endif
      {
      // 2.编队控制一致性保持
        vel1_msg.linear.x =  0.5 * d + 0.5*currentVel1.linear.x;
        vel1_msg.angular.z = 4 * theta; 
        if(vel1_msg.angular.z > 1) vel1_msg.angular.z = 1;
        if(vel1_msg.angular.z < -1) vel1_msg.angular.z = -1;
      } 
      if(d < 0.1)
      {
        vel1_msg.linear.x = 0; 
        // if(fabs(diff1.yaw) > 0.2)
        //   vel1_msg.angular.z = diff1.yaw;
        // else
          vel1_msg.angular.z = 0;
      }
    }
    else
    {
      vel1_msg.linear.x = 0;
      vel1_msg.angular.z = 0;
    }
    
    if(vel1_msg.linear.x > 0.2) vel1_msg.linear.x = 0.2;
    if(vel1_msg.angular.z > 1) vel1_msg.angular.z = 1;
    if(vel1_msg.angular.z < -1) vel1_msg.angular.z = -1;
    robot2_pub.publish(vel1_msg);
  }
#endif

#ifdef FORMATION_ROBOT3
  // robot3 control
  if(m_v_global_pose.find("robot3") != m_v_global_pose.end())
  {
    g2o::SE3Quat robot3_se3 = computePoseToG2oSE3(m_v_global_pose["robot3"].back());
    // 1. relative positioning 
    RobotPose rel_13;
    g2o::SE3Quat rel_13_se3;
  
    rel_13_se3 = robot1_se3.inverse() * robot3_se3;
    rel_13 = computeG2oSE3ToPose(rel_13_se3);
    
    std::cout << "----------------uwb: " << rel_13.x << "\t" <<  rel_13.y << "\t" << rel_13.yaw <<  std::endl;
  
    // 2. set a goal;
    RobotPose goal1; 
    goal1.x = INITIAL3_X;
    goal1.y = INITIAL3_Y;
    goal1.yaw = INITIAL3_YAW;
  
    // 3. compute the diff of goal to rel
    g2o::SE3Quat diff1_se3 = rel_13_se3.inverse() * computePoseToG2oSE3(goal1);
    RobotPose diff1 = computeG2oSE3ToPose(diff1_se3);

    // 4. publish robot2/cmd_vel -> diff1 and robot3/cmd_vel -> diff2
    if(fabs(timestamp - m_v_global_pose["robot3"].back().timestamp) < 2)
    {
      double d = sqrt(diff1.x*diff1.x + diff1.y*diff1.y);
      double theta = atan2(diff1.y, diff1.x);  
      // 1.避障控制
      double r = sqrt(obstacle3_pose.x*obstacle3_pose.x + obstacle3_pose.y*obstacle3_pose.y);
#ifdef BIZHANG_ENABLE
      if(r < detect_R) // 避障 
      {
        // std::cout << "obstacle: ";
        geometry_msgs::Twist goal;
        goal.linear.x = d;
        goal.angular.z = theta;
        vel2_msg = vfhDriver2->approachGoalCommand(0.1, goal, currentVel3, v_laserScan3);
        double PI = d/0.4;
        if(PI > 1) PI = 1;

        vel2_msg.linear.x  = PI*vel2_msg.linear.x;
        vel2_msg.angular.z = PI*vel2_msg.angular.z;
      }
      else 
#endif
      {
      // 2.编队控制一致性保持
        vel2_msg.linear.x =  0.5 * d + 0.5*currentVel1.linear.x;
        vel2_msg.angular.z = 4 * theta; 
        
        if(vel2_msg.angular.z > 1) vel2_msg.angular.z = 1;
        if(vel2_msg.angular.z < -1) vel2_msg.angular.z = -1;
      }
      if(d < 0.1)
      {
        vel2_msg.linear.x = 0;
        vel2_msg.angular.z = 0;
        // if(fabs(diff1.yaw) > 0.2 )
        //   vel2_msg.angular.z = diff1.yaw;
        // else
          vel2_msg.angular.z = 0;
      }
    }
    else
    {
      vel2_msg.linear.x = 0;
      vel2_msg.angular.z = 0;
    }

    if(vel2_msg.linear.x > 0.2) vel2_msg.linear.x = 0.2;
    if(vel2_msg.angular.z > 1) vel2_msg.angular.z = 1;
    if(vel2_msg.angular.z < -1) vel2_msg.angular.z = -1;
    robot3_pub.publish(vel2_msg);
  }
#endif
}

void *plot_tid(void *args)
{
  while(1)
  {
      plotGlobalPose();
      sleep(1);
  }
}
void robot2PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double timestamp = msg->header.stamp.toSec();
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  msg->pose.pose.position.z;
  m_v_global_pose["robot2"].push_back(pose);
}
void robot3PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
 double timestamp = msg->header.stamp.toSec();
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  msg->pose.pose.position.z;
  m_v_global_pose["robot3"].push_back(pose);
}

void robot1ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   v_laserScan1.clear();
   const double LIDAR_ERR = 0.05;
   const double LIDAR_MAX = 30;
   double min_dis = 3;
   double min_ang = 0;
   RobotPose min_point;
  //  std::cout << msg->ranges.size() << std::endl;
   for(int i=0; i<msg->ranges.size(); i++)
   {
      if(i%2 == 0)
        v_laserScan1.push_back(msg->ranges[i]);
      if(i>=180 && i<540)
      {
        if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
        {
          if(min_dis > msg->ranges[i])
          {
            min_dis = msg->ranges[i];
            min_ang = i;
          }
        }
      }
   }
   v_laserScan1.push_back(msg->ranges.back());
  //  std::cout << v_laserScan2.size();
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*2*M_PI/720);
     min_point.y=min_dis*sin(min_ang*2*M_PI/720);
     min_point.yaw=min_ang*2*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
  //  std::cout << msg->ranges.size() << std::endl;
   obstacle1_pose = min_point;
} 
void robot2ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   v_laserScan2.clear();
   const double LIDAR_ERR = 0.10;
   const double LIDAR_MAX = 30;
   double min_dis = 3;
   double min_ang = 0;
   RobotPose min_point;
  //  std::cout << msg->ranges.size() << std::endl;
   for(int i=0; i<msg->ranges.size(); i++)
   {
     if(i%2 == 0)
      v_laserScan2.push_back(msg->ranges[i]);
       if(i>=0 && i<720)
       {
         if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
         {
           if(min_dis > msg->ranges[i])
           {
             min_dis = msg->ranges[i];
             min_ang = i;
           }
         }
       }
   }
   v_laserScan2.push_back(msg->ranges.back());
  //  std::cout << v_laserScan2.size();
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*2*M_PI/720);
     min_point.y=min_dis*sin(min_ang*2*M_PI/720);
     min_point.yaw=min_ang*2*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
  //  std::cout << msg->ranges.size() << std::endl;
   obstacle2_pose = min_point;
} 
void robot3ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  v_laserScan3.clear();
   const double LIDAR_ERR = 0.10;
   const double LIDAR_MAX = 30;
   double min_dis = 3;
   double min_ang = 0;
   RobotPose min_point;
   for(int i=0; i<msg->ranges.size(); i++)
   {
      if(i%2 == 0)
      v_laserScan3.push_back(msg->ranges[i]);
       if(i>=0 && i<720 )
       {
         if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
         {
           if(min_dis > msg->ranges[i])
           {
             min_dis = msg->ranges[i];
             min_ang = i;
           }
         }
       }
   }
   v_laserScan3.push_back(msg->ranges.back());
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*2*M_PI/720);
     min_point.y=min_dis*sin(min_ang*2*M_PI/720);
     min_point.yaw=min_ang*2*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
   obstacle3_pose = min_point;
} 
void robot1CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  currentVel1.linear.x = msg->linear.x;
  currentVel1.angular.z = msg->angular.z;
}
void robot2CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  currentVel2.linear.x = msg->linear.x;
  currentVel2.angular.z = msg->angular.z;
}
void robot3CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  currentVel3.linear.x = msg->linear.x;
  currentVel3.angular.z = msg->angular.z;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_robots_formation_node");
  ros::NodeHandle n;
 
  ros::Subscriber sub_robot1_pose = n.subscribe("/robot1/est_odom", 1000, robot1PoseCallback);
	ros::Subscriber sub_robot2_pose = n.subscribe("/robot2/est_odom", 1000, robot2PoseCallback);
	ros::Subscriber sub_robot3_pose = n.subscribe("/robot3/est_odom", 1000, robot3PoseCallback);
  
  // ros::Subscriber sub_robot1_laser = n.subscribe("/robot1/scan", 1000, robot1ScanCallback);
  ros::Subscriber sub_robot2_laser = n.subscribe("/robot2/scan", 1000, robot2ScanCallback);
  ros::Subscriber sub_robot3_laser = n.subscribe("/robot3/scan", 1000, robot3ScanCallback);

#ifdef SIMULATION_ENABLE
  ros::Subscriber sub_robot1_vel = n.subscribe("/robot1/cmd_vel", 1000, robot1CmdVelCallback);
  ros::Subscriber sub_robot2_vel = n.subscribe("/robot2/cmd_vel", 1000, robot2CmdVelCallback);
  ros::Subscriber sub_robot3_vel = n.subscribe("/robot3/cmd_vel", 1000, robot3CmdVelCallback);
  robot2_pub = n.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1);
  robot3_pub = n.advertise<geometry_msgs::Twist>("robot3/cmd_vel", 1);
#else
  ros::Subscriber sub_robot1_vel = n.subscribe("/robot1/cmd_vel_mux/input/teleop", 1000, robot1CmdVelCallback);
  ros::Subscriber sub_robot2_vel = n.subscribe("/robot2/cmd_vel_mux/input/teleop", 1000, robot2CmdVelCallback);
  ros::Subscriber sub_robot3_vel = n.subscribe("/robot3/cmd_vel_mux/input/teleop", 1000, robot3CmdVelCallback);
  robot2_pub = n.advertise<geometry_msgs::Twist>("robot2/cmd_vel_mux/input/teleop", 1);
  robot3_pub = n.advertise<geometry_msgs::Twist>("robot3/cmd_vel_mux/input/teleop", 1);
#endif
  // pthread_t tid;
	// int ret = pthread_create(&tid, NULL, plot_tid, NULL);
  ros::spin();
  return 0;
}
