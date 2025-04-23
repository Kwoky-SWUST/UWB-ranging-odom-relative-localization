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

using namespace g2o;
int GAP = 1;
double detect_R2 = 1.5;
double detect_R3 = 1.5;
class RobotPose{
public:
  RobotPose() {}
  double x;
  double y;
  double yaw;
  double timestamp;
};
const double INITIAL1_X = 0;
const double INITIAL1_Y = 0;
const double INITIAL1_YAW = 0;
const double INITIAL2_X = -1.0;
const double INITIAL2_Y = 1.0;
const double INITIAL2_YAW = 0;
const double INITIAL3_X = -1.0;
const double INITIAL3_Y = -1.0;
const double INITIAL3_YAW = 0;
const double ROBOT_RAD = 0.2;
double L1 = 0.1;
double L2 = 0.1;
RobotPose robot1_previous_odom;
RobotPose robot2_previous_odom;
RobotPose robot3_previous_odom;
RobotPose robot1_current_odom;
RobotPose robot2_current_odom;
RobotPose robot3_current_odom;

RobotPose obstacle1_pose;
RobotPose obstacle2_pose;
RobotPose obstacle3_pose;

double Dse = 0;
double Lr = 0;
double Ll = 0;
double Lr2 = 0;
double Ll2 = 0;
double Lr3 = 0;
double Ll3 = 0;
int zai_flag2 = 0;
int zai_flag3 = 0;
int guo2_flag = 0;
int guo3_flag = 0;
int state_flag = 0;


std::map<std::string, std::vector<RobotPose> > m_v_global_pose;
std::string ex_file("/home/dzy/Desktop/大论文实验数据/exp3/ex_error.txt");
std::string ey_file("/home/dzy/Desktop/大论文实验数据/exp3/ey_error.txt");
std::string et_file("/home/dzy/Desktop/大论文实验数据/exp3/et_error.txt");
std::string all_pose_file("/home/dzy/Desktop/大论文实验数据/exp3/all_pose.txt");
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

int search_gap_point(double gap, std::vector<RobotPose> path, RobotPose& point)
{
  int path_size = path.size();
  double c_x = path.back().x;
  double c_y = path.back().y;
  int cnt = 0;
  int i;
  for(i=path_size-1; i>0; i--)
  {
    cnt++;
    double delta_d = sqrt( pow((path[i].x - c_x),2) + pow((path[i].y - c_y),2 ));
    if(delta_d >=gap)
    {
      point = path[i];
      std::cout << delta_d << std::endl;
      return 1;
    }
  }
  return 0;
}

void robot1PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(m_v_global_pose.find("robot2") == m_v_global_pose.end() || m_v_global_pose.find("robot3") == m_v_global_pose.end()) 
    return;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot1/odom"));
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double timestamp = msg->header.stamp.toSec();
  RobotPose pose;
  pose.timestamp = timestamp;
  static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
		robot1_current_odom.x = INITIAL1_X;
		robot1_current_odom.y = INITIAL1_Y;
		robot1_current_odom.yaw =  INITIAL1_YAW;
		pose.x=robot1_current_odom.x;
		pose.y=robot1_current_odom.y;
		pose.yaw=robot1_current_odom.yaw; 	
    robot1_previous_odom.x=msg->pose.pose.position.x;
		robot1_previous_odom.y=msg->pose.pose.position.y;
		robot1_previous_odom.yaw=yaw;
	}
	else
	{
		double previous_x = robot1_previous_odom.x;
		double previous_y = robot1_previous_odom.y;
		double previous_theta = robot1_previous_odom.yaw;

		double current_x = msg->pose.pose.position.x;
		double current_y = msg->pose.pose.position.y ;
		double current_theta = yaw;

		double actual_delta_x=current_x-previous_x;
		double actual_delta_y=current_y-previous_y;
    
    double sigma_trans=sqrt(actual_delta_x*actual_delta_x+actual_delta_y*actual_delta_y );
    double actual_delta_theta=carmen_normalize_theta(current_theta-previous_theta);

		robot1_previous_odom.x=msg->pose.pose.position.x;
		robot1_previous_odom.y=msg->pose.pose.position.y;
		robot1_previous_odom.yaw=yaw;

		robot1_current_odom.x = robot1_current_odom.x + sigma_trans*cos(robot1_current_odom.yaw);
		robot1_current_odom.y = robot1_current_odom.y + sigma_trans*sin(robot1_current_odom.yaw);
		robot1_current_odom.yaw = robot1_current_odom.yaw + actual_delta_theta;
		robot1_current_odom.yaw = carmen_normalize_theta(robot1_current_odom.yaw);

		pose.x = robot1_current_odom.x;
		pose.y = robot1_current_odom.y;
		pose.yaw = robot1_current_odom.yaw;
	}
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yaw;
  m_v_global_pose["robot1"].push_back(pose);
  
  // 1.formation control: 基于领航跟随法的机器人可变队形编队控制. 彭滔，陈延政，刘成军
  // 获得一个基于领航者的坐标系
  // plotGlobalPose();
  RobotPose robot1_pose = pose;
  // robot1_pose.yaw = 0;
  RobotPose robot2_pose = m_v_global_pose["robot2"].back();
  // robot2_pose.yaw = 0;
  RobotPose robot3_pose = m_v_global_pose["robot3"].back();
  // robot3_pose.yaw = 0;
  g2o::SE3Quat robot1_se3 = computePoseToG2oSE3(robot1_pose);
  g2o::SE3Quat robot2_se3 = computePoseToG2oSE3(robot2_pose);
  g2o::SE3Quat robot3_se3 = computePoseToG2oSE3(robot3_pose);
  // 1. relative positioning 
  RobotPose rel_12;
  RobotPose rel_13;
  g2o::SE3Quat rel_12_se3;
  g2o::SE3Quat rel_13_se3;
  rel_12_se3 = robot1_se3.inverse() * robot2_se3;
  rel_12 = computeG2oSE3ToPose(rel_12_se3);
  rel_13_se3 = robot1_se3.inverse() * robot3_se3;
  rel_13 = computeG2oSE3ToPose(rel_13_se3);

  // 根据期望队形生成编队所需虚拟位姿
  // 2. set a goal;
  RobotPose goal1; 
  RobotPose goal2;

  static RobotPose ru_point;
  if((Ll < 1.0 && Lr < 1.0) && state_flag == 0)
  {
    zai_flag2 = 1;
    zai_flag3 = 1;
    state_flag = 1;
    ru_point = robot1_pose;  
  }

  if(state_flag == 1)
  {
    double delta_d = sqrt(pow(robot1_pose.x - ru_point.x, 2) + pow(robot1_pose.y - ru_point.y, 2));
    if(delta_d > 1.2*GAP)
    {
      guo2_flag = 1;
    }

    if((Ll >= 1.0 && Lr >= 1.0) &&
       (Ll2 >= 1.0 && Lr2 >= 1.0) &&
       (Ll3 >= 1.0 && Lr3 >= 1.0)&&
        guo2_flag == 1
     )
    {
      detect_R2 = detect_R3 = 1.5;
      guo2_flag = 0;
      zai_flag2 = 0;
      state_flag = 0;
      zai_flag3 = 0;
    }
  }

  if(zai_flag2 == 1)
  {
    detect_R2 = ROBOT_RAD;
    RobotPose point;
    if(search_gap_point(GAP, m_v_global_pose["robot1"], point))
    {
      std::cout << point.x  << "\t" << point.y << std::endl;
      g2o::SE3Quat goal1_se3 = computePoseToG2oSE3(robot1_pose).inverse()*computePoseToG2oSE3(point);
      goal1 = computeG2oSE3ToPose(goal1_se3);
    }else{
      goal1.x = INITIAL2_X;
      goal1.y = INITIAL2_Y;
      goal1.yaw = INITIAL2_YAW;
    }
  }
  else
  {
    goal1.x = INITIAL2_X;
    goal1.y = INITIAL2_Y;
    goal1.yaw = INITIAL2_YAW;
  }
  if(zai_flag3 == 1)
  {
    detect_R3 = ROBOT_RAD;
    RobotPose point;
    if(search_gap_point(2*GAP, m_v_global_pose["robot1"], point))
    {
      std::cout << point.x  << "\t" << point.y << std::endl;
      g2o::SE3Quat goal2_se3 = computePoseToG2oSE3(robot1_pose).inverse()*computePoseToG2oSE3(point);
      goal2 = computeG2oSE3ToPose(goal2_se3);
    }else{
      goal2.x = INITIAL3_X;
      goal2.y = INITIAL3_Y;
      goal2.yaw = INITIAL3_YAW;
    }
  }
  else
  {
    goal2.x = INITIAL3_X;
    goal2.y = INITIAL3_Y;
    goal2.yaw = INITIAL3_YAW;
  }
  // // {
  //   static int flag = 1;
  // //   static RobotPose start_point;
  // //   if(flag)
  // //   {
  // //     flag = 0;
  // //     start_point = robot1_pose;
  // //   }
  // //   goal1.x = -0.6;
  // //   goal1.y = 0;
  // //   goal1.yaw = 0;
  // //   goal2.x = -1.2;
  // //   goal2.y = 0;
  // //   goal2.yaw = 0;
  // //   std::cout << goal1.x << "\t" << goal1.y << "\t" <<  goal2.x << "\t" << goal2.y << std::endl;
  // // }
  // // else
  // {
    
   
  //   goal2.x = INITIAL3_X;
  //   goal2.y = INITIAL3_Y;
  //   goal2.yaw = INITIAL3_YAW;
  //   // std::cout << "dui2" << std::endl;
  // }
  // 3. 机器人2误差方程
  double xe = goal1.x - rel_12.x;
  double ye = goal1.y - rel_12.y;
  double te = goal1.yaw - rel_12.yaw;

  double e1 = xe*cos(rel_12.yaw) + ye*sin(rel_12.yaw);
  double e2 = -xe*sin(rel_12.yaw) + ye*cos(rel_12.yaw);
  double et = te;
  
  double vr = currentVel1.linear.x;
  double wr = currentVel1.angular.z;
  double v = currentVel2.linear.x;
  double w = currentVel2.angular.z;
  
  static double old_vr = 0;
  static double old_wr = 0;

  double dvr = vr - old_vr;
  double dwr = vr - old_wr;

  old_vr = vr;
  old_wr = wr;
  // 4. 机器人3
  double xe3 = goal2.x - rel_13.x;
  double ye3 = goal2.y - rel_13.y;
  double te3 = goal2.yaw - rel_13.yaw;

  double e13 = xe3*cos(rel_13.yaw) + ye3*sin(rel_13.yaw);
  double e23 = -xe3*sin(rel_13.yaw) + ye3*cos(rel_13.yaw);
  double et3 = te3;
  
  double v3 = currentVel3.linear.x;
  double w3 = currentVel3.angular.z;


  double k1 = 0.1;
  double k2 = 4;
  double k3 = 1;
  
 if(fabs(timestamp - m_v_global_pose["robot2"].back().timestamp) < 1)
 {
    double d = sqrt(e1*e1 + e2*e2);
    double theta = atan2(e2, e1);  
    // 1.避障控制
    double r = sqrt(obstacle2_pose.x*obstacle2_pose.x + obstacle2_pose.y*obstacle2_pose.y);
    RobotPose robot21;
    RobotPose robot23;
    robot21 = computeG2oSE3ToPose(computePoseToG2oSE3(robot2_pose).inverse() * computePoseToG2oSE3(robot1_pose));
    robot23 = computeG2oSE3ToPose(computePoseToG2oSE3(robot2_pose).inverse() * computePoseToG2oSE3(robot3_pose));
    
    if(r < detect_R2 && (fabs(robot21.x-obstacle2_pose.x) > L1 || fabs(robot21.y-obstacle2_pose.y) > L2) && (fabs(robot23.x-obstacle2_pose.x) > L1 || fabs(robot23.y-obstacle2_pose.y) > L2)) // 避障 
    {
      geometry_msgs::Twist goal;
      goal.linear.x = d;
      goal.angular.z = theta;
      vel1_msg = vfhDriver1->approachGoalCommand(0.1, goal, currentVel2, v_laserScan2);
    }
    else
    {
      v = vr*cos(et) + k2*e1; //  + ;
      w = wr + (1/k1)*vr*e2  + k3*sin(et);
                              
      vel1_msg.linear.x = v;
      vel1_msg.angular.z = w;
    }
    if(vel1_msg.linear.x > 0.5) vel1_msg.linear.x = 0.5;
    if(vel1_msg.linear.x < 0) vel1_msg.linear.x = 0;
    if(vel1_msg.angular.z > 1.0) vel1_msg.angular.z = 1.0;
    if(vel1_msg.angular.z < -1.0) vel1_msg.angular.z = -1.0;
 }
 else
 {
  vel1_msg.linear.x = 0;
  vel1_msg.angular.z = 0;
 }

  g2o::SE3Quat diff2_se3 = rel_13_se3.inverse() * computePoseToG2oSE3(goal2);
  RobotPose diff2 = computeG2oSE3ToPose(diff2_se3);
 if(fabs(timestamp - m_v_global_pose["robot3"].back().timestamp) < 1)
 {
    double d = sqrt(e13*e13 + e23*e23);
    double theta = atan2(e23, e13);  
    double d1 = sqrt(diff2.x*diff2.x + diff2.y*diff2.y);
    double theta1 = atan2(diff2.y, diff2.x);  

    // 1.避障控制
    double r = sqrt(obstacle3_pose.x*obstacle3_pose.x + obstacle3_pose.y*obstacle3_pose.y);
    RobotPose robot31;
    RobotPose robot32;
    robot31 = computeG2oSE3ToPose(computePoseToG2oSE3(robot3_pose).inverse() * computePoseToG2oSE3(robot1_pose));
    robot32 = computeG2oSE3ToPose(computePoseToG2oSE3(robot3_pose).inverse() * computePoseToG2oSE3(robot2_pose));
    
    if(r < detect_R3 && (fabs(robot31.x-obstacle3_pose.x) > L1 || fabs(robot31.y-obstacle3_pose.y) > L2) && (fabs(robot32.x-obstacle3_pose.x) > L1 || fabs(robot32.y-obstacle3_pose.y) > L2)) // 避障 
    {
      geometry_msgs::Twist goal;
      goal.linear.x = d;
      goal.angular.z = theta;
      vel2_msg = vfhDriver2->approachGoalCommand(0.1, goal, currentVel3, v_laserScan3);
    }
    else
    {
      v3 = vr*cos(et3) + k2*e13; //  + ;
      w3 = wr + (1/k1)*vr*e23  + k3*sin(et3);
                     
      vel2_msg.linear.x = v3;
      vel2_msg.angular.z = w3;
    }
    if(vel2_msg.linear.x > 0.5) vel2_msg.linear.x = 0.5;
    if(vel2_msg.linear.x < 0) vel2_msg.linear.x = 0;
    if(vel2_msg.angular.z > 1.0) vel2_msg.angular.z = 1.0;
    if(vel2_msg.angular.z < -1.0) vel2_msg.angular.z = -1.0;
  
 }
 else
 {
  vel2_msg.linear.x = 0;
  vel2_msg.angular.z = 0;
 }
 std::ofstream file1(ex_file, std::ios::app);
 file1 << "robot2_ex\t" << e1 << "\trobot3_ex\t" << e13 << "\n";
 file1.close();

 std::ofstream file2(ey_file, std::ios::app);
 file2 << "robot2_ey\t" << e2 << "\trobot3_ey\t" << e23 << "\n";
 file2.close();

 std::ofstream file3(et_file, std::ios::app);
 file3 << "robot2_et\t" << et << "\trobot3_et\t" << et3 << "\n";
 file3.close();

 std::ofstream file4(all_pose_file, std::ios::app);
 file4 << "robot1\t" << robot1_pose.x << "\t" << robot1_pose.y << "\t" << robot1_pose.yaw << "\t"
       << "robot2\t" << robot2_pose.x << "\t" << robot2_pose.y << "\t" << robot2_pose.yaw << "\t"
       << "robot3\t" << robot3_pose.x << "\t" << robot3_pose.y << "\t" << robot3_pose.yaw << "\n";
 file4.close();
}

void robot2PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot2/odom"));
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double timestamp = msg->header.stamp.toSec();
 RobotPose pose;
  pose.timestamp = timestamp;
  static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
		robot2_current_odom.x = INITIAL2_X;
		robot2_current_odom.y = INITIAL2_Y;
		robot2_current_odom.yaw =  INITIAL2_YAW;
		pose.x=robot2_current_odom.x;
		pose.y=robot2_current_odom.y;
		pose.yaw=robot2_current_odom.yaw; 	
    robot2_previous_odom.x=msg->pose.pose.position.x;
		robot2_previous_odom.y=msg->pose.pose.position.y;
		robot2_previous_odom.yaw=yaw;
	}
	else
	{
		double previous_x = robot2_previous_odom.x;
		double previous_y = robot2_previous_odom.y;
		double previous_theta = robot2_previous_odom.yaw;

		double current_x = msg->pose.pose.position.x;
		double current_y = msg->pose.pose.position.y ;
		double current_theta = yaw;

		double actual_delta_x=current_x-previous_x;
		double actual_delta_y=current_y-previous_y;
    
    double sigma_trans=sqrt(actual_delta_x*actual_delta_x+actual_delta_y*actual_delta_y );
    double actual_delta_theta=carmen_normalize_theta(current_theta-previous_theta);

		robot2_previous_odom.x=msg->pose.pose.position.x;
		robot2_previous_odom.y=msg->pose.pose.position.y;
		robot2_previous_odom.yaw=yaw;

		robot2_current_odom.x = robot2_current_odom.x + sigma_trans*cos(robot2_current_odom.yaw);
		robot2_current_odom.y = robot2_current_odom.y + sigma_trans*sin(robot2_current_odom.yaw);
		robot2_current_odom.yaw = robot2_current_odom.yaw + actual_delta_theta;
		robot2_current_odom.yaw = carmen_normalize_theta(robot2_current_odom.yaw);

		pose.x = robot2_current_odom.x;
		pose.y = robot2_current_odom.y;
		pose.yaw = robot2_current_odom.yaw;
	}
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yaw;
  m_v_global_pose["robot2"].push_back(pose);
}
void robot3PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot3/odom"));
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double timestamp = msg->header.stamp.toSec();
   RobotPose pose;
  pose.timestamp = timestamp;
  static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
		robot3_current_odom.x = INITIAL3_X;
		robot3_current_odom.y = INITIAL3_Y;
		robot3_current_odom.yaw =  INITIAL3_YAW;
		pose.x=robot3_current_odom.x;
		pose.y=robot3_current_odom.y;
		pose.yaw=robot3_current_odom.yaw; 	
    robot3_previous_odom.x=msg->pose.pose.position.x;
		robot3_previous_odom.y=msg->pose.pose.position.y;
		robot3_previous_odom.yaw=yaw;
	}
	else
	{
		double previous_x = robot3_previous_odom.x;
		double previous_y = robot3_previous_odom.y;
		double previous_theta = robot3_previous_odom.yaw;

		double current_x = msg->pose.pose.position.x;
		double current_y = msg->pose.pose.position.y ;
		double current_theta = yaw;

		double actual_delta_x=current_x-previous_x;
		double actual_delta_y=current_y-previous_y;
    
    double sigma_trans=sqrt(actual_delta_x*actual_delta_x+actual_delta_y*actual_delta_y );
    double actual_delta_theta=carmen_normalize_theta(current_theta-previous_theta);

		robot3_previous_odom.x=msg->pose.pose.position.x;
		robot3_previous_odom.y=msg->pose.pose.position.y;
		robot3_previous_odom.yaw=yaw;

		robot3_current_odom.x = robot3_current_odom.x + sigma_trans*cos(robot3_current_odom.yaw);
		robot3_current_odom.y = robot3_current_odom.y + sigma_trans*sin(robot3_current_odom.yaw);
		robot3_current_odom.yaw = robot3_current_odom.yaw + actual_delta_theta;
		robot3_current_odom.yaw = carmen_normalize_theta(robot3_current_odom.yaw);

		pose.x = robot3_current_odom.x;
		pose.y = robot3_current_odom.y;
		pose.yaw = robot3_current_odom.yaw;
	}
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yaw;
  m_v_global_pose["robot3"].push_back(pose);
}

void robot1ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   const double LIDAR_ERR = 0.10;
   const double LIDAR_MAX = 30;
   double min_dis = 3;
   double min_ang = 0;
   double temp_lr = 0;
   double temp_ll = 0;
   RobotPose min_point;
   for(int i=0; i<msg->ranges.size(); i++)
   {
      if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
      {
        if(min_dis > msg->ranges[i])
        {
          min_dis = msg->ranges[i];
          min_ang = i;
        }
      }
      if(i == 0 )
      {
        temp_lr = msg->ranges[i]*cos(i*M_PI/720);
        if(isinf(temp_lr))
          temp_lr = 100;
      }
      if(i == 719 )
      {
         temp_ll = msg->ranges[i]*cos(i*M_PI/720);
         if(isinf(temp_ll))
          temp_ll = 100;
      }
   }
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*M_PI/720);
     min_point.y=min_dis*sin(min_ang*M_PI/720);
     min_point.yaw=min_ang*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
   obstacle1_pose = min_point;
   Lr = temp_lr;
   Ll = temp_ll;
} 

void robot2ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   v_laserScan2.clear();
   const double LIDAR_ERR = 0.10;
   const double LIDAR_MAX = 30;
   double min_dis = 3;
   double min_ang = 0;
   RobotPose min_point;
   for(int i=0; i<msg->ranges.size(); i++)
   {
    if(i%2 == 0)
      v_laserScan2.push_back(msg->ranges[i]);
    if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
    {
      if(min_dis > msg->ranges[i])
      {
        min_dis = msg->ranges[i];
        min_ang = i;
      }
    }
     if(i == 0 )
      {
        Lr2 = msg->ranges[i]*cos(i*M_PI/720);
         if(isinf(Lr2))
          Lr2 = 100;
      }
      if(i == 719 )
      {
         Ll2 = msg->ranges[i]*cos(i*M_PI/720); 
         if(isinf(Ll2))
          Ll2 = 100;
      }
   }
   v_laserScan2.push_back(msg->ranges.back());
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*M_PI/720);
     min_point.y=min_dis*sin(min_ang*M_PI/720);
     min_point.yaw=min_ang*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
   obstacle2_pose = min_point;
   RobotPose rotation;
   rotation.x = rotation.y = 0;
   rotation.yaw = -M_PI/2.0;
   obstacle2_pose = computeG2oSE3ToPose(computePoseToG2oSE3(rotation)*computePoseToG2oSE3(obstacle2_pose)); 
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
    if(msg->ranges[i] >= LIDAR_ERR && msg->ranges[i]<=LIDAR_MAX)
    {
      if(min_dis > msg->ranges[i])
      {
        min_dis = msg->ranges[i];
        min_ang = i;
      }
    }
      if(i == 0 )
      {
        Lr3 = msg->ranges[i]*cos(i*M_PI/720);
        if(isinf(Lr3))
          Lr3 = 100;
      }
      if(i == 719 )
      {
        Ll3 = msg->ranges[i]*cos(i*M_PI/720);
         if(isinf(Ll3))
          Ll3 = 100;
      }
         
   }
   v_laserScan3.push_back(msg->ranges.back());
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*M_PI/720);
     min_point.y=min_dis*sin(min_ang*M_PI/720);
     min_point.yaw=min_ang*M_PI/720;
   }
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   }
   obstacle3_pose = min_point;
   RobotPose rotation;
   rotation.x = rotation.y = 0;
   rotation.yaw = -M_PI/2.0;
   obstacle3_pose = computeG2oSE3ToPose(computePoseToG2oSE3(rotation)*computePoseToG2oSE3(obstacle3_pose)); 
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

void robot2RelCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  RobotPose rel_pose;
  rel_pose.x = msg->x;
  rel_pose.y = msg->y;
  rel_pose.yaw = msg->z;
  m_rel_pose["robot2"] = rel_pose;
}
void robot3RelCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  RobotPose rel_pose;
  rel_pose.x = msg->x;
  rel_pose.y = msg->y;
  rel_pose.yaw = msg->z;
   m_rel_pose["robot3"] = rel_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_robots_formation_node");
  ros::NodeHandle n;
  std::ofstream file1(ex_file, std::ios::out);
  std::ofstream file2(ey_file, std::ios::out);
  std::ofstream file3(et_file, std::ios::out);
  std::ofstream file4(all_pose_file, std::ios::out);
  ros::Subscriber sub_robot1_pose = n.subscribe("/robot1/odom", 1000, robot1PoseCallback);
	ros::Subscriber sub_robot2_pose = n.subscribe("/robot2/odom", 1000, robot2PoseCallback);
	ros::Subscriber sub_robot3_pose = n.subscribe("/robot3/odom", 1000, robot3PoseCallback);
  
  ros::Subscriber sub_robot1_laser = n.subscribe("/robot1/scan", 1000, robot1ScanCallback);
  ros::Subscriber sub_robot2_laser = n.subscribe("/robot2/scan", 1000, robot2ScanCallback);
  ros::Subscriber sub_robot3_laser = n.subscribe("/robot3/scan", 1000, robot3ScanCallback);
  
  ros::Subscriber sub_robot1_vel = n.subscribe("/robot1/cmd_vel", 1000, robot1CmdVelCallback);
  ros::Subscriber sub_robot2_vel = n.subscribe("/robot2/cmd_vel", 1000, robot2CmdVelCallback);
  ros::Subscriber sub_robot3_vel = n.subscribe("/robot3/cmd_vel", 1000, robot3CmdVelCallback);
  
  ros::Subscriber sub_robot2_rel = n.subscribe("/robot2/relative_pose", 1000, robot2RelCallback);
  ros::Subscriber sub_robot3_rel = n.subscribe("/robot3/relative_pose", 1000, robot3RelCallback);

  ros::Publisher robot2_pub = n.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 10);
  ros::Publisher robot3_pub = n.advertise<geometry_msgs::Twist>("robot3/cmd_vel", 10);

  ros::Rate sleep_rate(20);
  while(n.ok())
  {
    ros::spinOnce();
    robot2_pub.publish(vel1_msg);
    robot3_pub.publish(vel2_msg);
    sleep_rate.sleep();
  }
  ros::spin();
  return 0;
}
