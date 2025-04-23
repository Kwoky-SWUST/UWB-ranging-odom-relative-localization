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
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
// #define SIMULATION_ENABLE

using namespace g2o;
ros::Publisher robot1_p_pub; 
ros::Publisher robot2_p_pub;
ros::Publisher robot3_p_pub;
ros::Publisher robot2_pub;
ros::Publisher robot3_pub;
ros::Publisher robot1_submap;
double detect_R = 1.5;
double GAP = 0.6;
bool change_flag2_ = false;
bool change_flag3_ = false;
const double pre_d_ = 0.3;
const double after_d_ = 0.8;
double detect_R2 = 1.5;
double detect_R3 = 1.5;
const double INITIAL1_X = 0;
const double INITIAL1_Y = 0;
const double INITIAL1_YAW = 0;
const double INITIAL2_X = -0.6;
const double INITIAL2_Y = 0.6;
const double INITIAL2_YAW = 0;
const double INITIAL3_X = -0.6;
const double INITIAL3_Y = -0.6;
const double INITIAL3_YAW = 0;
const double ROBOT_RAD = 0.3;
double L1 = 0.4;
double L2 = 0.4;


// bresensam 算法找直线
/*结构体点*/
template <class T>
struct point{
	inline point():x(0),y(0) {}
	inline point(T _x, T _y):x(_x),y(_y){}
	T x, y;
};

typedef point<int> IntPoint;

typedef struct
{
    int num_points;
    std::vector<IntPoint> points;
} GridLineTraversalLine;

struct GridLineTraversal
{
    inline static void gridLine(IntPoint start, IntPoint end, GridLineTraversalLine *line);
    inline static void gridLineCore(IntPoint start, IntPoint end, GridLineTraversalLine *line);
};

void GridLineTraversal::gridLineCore(IntPoint start, IntPoint end, GridLineTraversalLine *line)
{
    int dx, dy;             // 横纵坐标间距
    int incr1, incr2;       // P_m增量
    int d;                  // P_m
    int x, y, xend, yend;   // 直线增长的首末端点坐标
    int xdirflag, ydirflag; // 横纵坐标增长方向
    int cnt = 0;            // 直线过点的点的序号

    dx = abs(end.x - start.x);
    dy = abs(end.y - start.y);

    // 斜率绝对值小于等于1的情况，优先增加x
    if (dy <= dx)
    {
        d = 2 * dy - dx;       // 初始点P_m0值
        incr1 = 2 * dy;        // 情况（1）
        incr2 = 2 * (dy - dx); // 情况（2）

        // 将增长起点设置为横坐标小的点处，将 x的增长方向 设置为 向右侧增长
        if (start.x > end.x)
        {
            // 起点横坐标比终点横坐标大，ydirflag = -1（负号可以理解为增长方向与直线始终点方向相反）
            x = end.x; 
            y = end.y;
            ydirflag = (-1);
            xend = start.x; // 设置增长终点横坐标
        }
        else
        {
            x = start.x;
            y = start.y;
            ydirflag = 1;
            xend = end.x;
        }

        //加入起点坐标
        line->points.push_back(IntPoint(x, y));
        // line->points[cnt].x = x;
        // line->points[cnt].y = y;
        cnt++;

        // 向 右上 方向增长
        if (((end.y - start.y) * ydirflag) > 0)
        {
            // start.y > end.y
            while (x < xend)
            {
                x++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    y++;
                    d += incr2; // 纵坐标向正方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
        // 向 右下 方向增长
        else
        {
            while (x < xend)
            {
                x++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    y--;
                    d += incr2; // 纵坐标向负方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
    }
    // 斜率绝对值大于1的情况，优先增加y
    else
    {
        // dy > dx，当斜率k的绝对值|k|>1时，在y方向进行单位步进
        d = 2 * dx - dy; 
        incr1 = 2 * dx;  
        incr2 = 2 * (dx - dy);

        // 将增长起点设置为纵坐标小的点处，将 y的增长方向 设置为 向上侧增长
        if (start.y > end.y)
        {
            y = end.y; // 取最小的纵坐标作为起点
            x = end.x;
            yend = start.y;
            xdirflag = (-1); 
        }
        else
        {
            y = start.y;
            x = start.x;
            yend = end.y;
            xdirflag = 1;
        }
        // 添加起点
        line->points.push_back(IntPoint(x, y));
        // line->points[cnt].x = x;
        // line->points[cnt].y = y;
        cnt++;
        // 向 上右 增长
        if (((end.x - start.x) * xdirflag) > 0)
        {
            while (y < yend)
            {
                y++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    x++;
                    d += incr2; // 横坐标向正方向增长
                }
                // 添加新的点
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
        // 向 上左 增长
        else
        {
            while (y < yend)
            {
                y++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    x--;
                    d += incr2; //横坐标向负方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                // 记录添加所有点的数目
                cnt++;
            }
        }
    }
    line->num_points = cnt;
}

// 最终在外面被使用的bresenham画线算法
void GridLineTraversal::gridLine(IntPoint start, IntPoint end, GridLineTraversalLine *line)
{
    int i, j;
    int half;
    IntPoint v;
    gridLineCore(start, end, line);
    if (start.x != line->points[0].x ||
        start.y != line->points[0].y)
    {
        half = line->num_points / 2;
        for (i = 0, j = line->num_points - 1; i < half; i++, j--)
        {
            v = line->points[i];
            line->points[i] = line->points[j];
            line->points[j] = v;
        }
    }
}
//----------------------------------------------------------------------------------------------


class RobotPose{
public:
  RobotPose() {}
  double x;
  double y;
  double yaw;
  double timestamp;
};
class SubMap{
  public:
  SubMap(){}
  double timestamp;
  double resolution;
  unsigned int width;
  unsigned int height;
  RobotPose origin;
  std::vector<signed char> data;;
};
SubMap subMap_;

RobotPose robot1_previous_odom;
RobotPose robot2_previous_odom;
RobotPose robot3_previous_odom;
RobotPose robot1_current_odom;
RobotPose robot2_current_odom;
RobotPose robot3_current_odom;

RobotPose obstacle1_pose;
RobotPose obstacle2_pose;
RobotPose obstacle3_pose;

std::string ex_file("/home/dzy/Desktop/大论文实验数据/exp3/ex_error.txt");
std::string ey_file("/home/dzy/Desktop/大论文实验数据/exp3/ey_error.txt");
std::string et_file("/home/dzy/Desktop/大论文实验数据/exp3/et_error.txt");
std::string all_pose_file("/home/dzy/Desktop/大论文实验数据/exp3/all_pose.txt");
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
bool search_gap_point(double gap, std::vector<RobotPose> path, RobotPose& point)
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
      return true;
    }
  }
  return false;
}
nav_msgs::OccupancyGrid af_submap;


bool getFormationScore(g2o::SE3Quat leader_se3, RobotPose relative_goal, double pre_d,double &score)
{
  RobotPose line_ep_pose;
  line_ep_pose.y = relative_goal.y;
  line_ep_pose.x = relative_goal.x + after_d_;
  line_ep_pose.yaw = 0;
  RobotPose global_line_ep = computeG2oSE3ToPose(leader_se3*computePoseToG2oSE3(line_ep_pose));
  RobotPose line_sp_pose;
  line_sp_pose.y = relative_goal.y;
  line_sp_pose.x = relative_goal.x - pre_d;
  line_sp_pose.yaw = 0;
  RobotPose global_line_sp = computeG2oSE3ToPose(leader_se3*computePoseToG2oSE3(line_sp_pose));

  RobotPose map_line_ep = computeG2oSE3ToPose(computePoseToG2oSE3(subMap_.origin).inverse() * computePoseToG2oSE3(global_line_ep));
  RobotPose map_line_sp = computeG2oSE3ToPose(computePoseToG2oSE3(subMap_.origin).inverse() * computePoseToG2oSE3(global_line_sp));

  IntPoint sp;
  sp.x = map_line_sp.x/subMap_.resolution;
  sp.y = map_line_sp.y/subMap_.resolution;
  IntPoint ep;
  ep.x = map_line_ep.x/subMap_.resolution;
  ep.y = map_line_ep.y/subMap_.resolution;

  // std::cout <<"pre:" << sp.x << "\t" << sp.y << "\t" << ep.x << "\t" << ep.y << std::endl;
  if(sp.x < 6)
    sp.x = 6;
  if(sp.y < 0)
    sp.y = 6;
  if(ep.x < 6)
    ep.x = 6;
  if(ep.y < 6)
    ep.y = 6;

  if(sp.x >= subMap_.width-6)
    sp.x = subMap_.width-6;
  if(sp.y >= subMap_.height-6)
    sp.y = subMap_.height-6;
  if(ep.x >= subMap_.width-6)
    ep.x = subMap_.width-6;
  if(ep.y >= subMap_.height-6)
    ep.y = subMap_.height-6;

  if(sp.x == 6 && sp.y == 6 && ep.x == 6 && ep.y == 6)
  {
    std::cout << "submap too small...." << std::endl;
    return false;
  }
  // std::cout << sp.x << "\t" << sp.y << "\t" << ep.x << "\t" << ep.y << std::endl;
  GridLineTraversalLine line;
  GridLineTraversal::gridLine(sp, ep, &line);

  double inflate_r = ROBOT_RAD/subMap_.resolution;

  // std::cout << "line:" << line.num_points <<  "\t" << line.points.size() << "\n point:\t" << std::endl;
  char tmp_map[subMap_.width*subMap_.height] = {0};
  for(int i=0; i<line.points.size(); i++){
    // std::cout << line.points[i].x << "\t" << line.points[i].y << std::endl;
    tmp_map[MAP_IDX(subMap_.width, line.points[i].x, line.points[i].y)] = 255;
  }
  cv::Mat cv_map(subMap_.height, subMap_.width, CV_8UC1, tmp_map);
  cv::bitwise_not(cv_map, cv_map); // 图像取反
 
  int erodeSize = inflate_r;

  cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                              cv::Size( 2*erodeSize + 1, 2*erodeSize+1 ),
                              cv::Point( erodeSize, erodeSize ) );
 
  cv::erode(cv_map, cv_map, element);

  int total_cnt = 0;  
  int goal_score_cnt = 0;  
  for(int i=0; i<subMap_.height*subMap_.width; i++)
  {
     if((int)cv_map.data[i] == 0)
     {
       total_cnt++;
       if(subMap_.data[i] == 100)
       {
          goal_score_cnt++;
       }
       af_submap.data[i] = -1;
       
     }
  }
  score = (double)(total_cnt-goal_score_cnt) / (double)(total_cnt);
  // std::cout << "get score:" << score << std::endl;
  return true;
}

geometry_msgs::Twist currentVel1;
std::vector<double> v_laserScan2;
geometry_msgs::Twist currentVel2;
std::vector<double> v_laserScan3;
geometry_msgs::Twist currentVel3;

geometry_msgs::Twist vel2_msg;

geometry_msgs::Twist vel3_msg;

vfh::VfhDriver *vfhDriver1 = new vfh::VfhDriver("/home/dzy/formation_ws/src/multi_robots_formation/config/robot2VFHConfig.txt");
vfh::VfhDriver *vfhDriver2 = new vfh::VfhDriver("/home/dzy/formation_ws/src/multi_robots_formation/config/robot2VFHConfig.txt");
std::map<std::string, RobotPose> m_rel_pose;
void robot1PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(m_v_global_pose.find("robot3") == m_v_global_pose.end() || m_v_global_pose.find("robot2") == m_v_global_pose.end()) 
    return;
    // std::cout << "en3" << std::endl;
  double timestamp = msg->header.stamp.toSec();
   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  yaw;
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

  
  RobotPose relative_goal2;
  relative_goal2.x = INITIAL2_X;
  relative_goal2.y = INITIAL2_Y;
  relative_goal2.yaw = INITIAL2_YAW;
  static RobotPose goal2;
  RobotPose relative_goal3;
  relative_goal3.x = INITIAL3_X;
  relative_goal3.y = INITIAL3_Y;
  relative_goal3.yaw = INITIAL3_YAW;
  static RobotPose goal3;
  if(fabs(pose.timestamp- subMap_.timestamp) < 1.0 )
  {
    ros::Time begin = ros::Time::now();
    double goal2_score = 0;
    if(getFormationScore(robot1_se3, relative_goal2, pre_d_ ,goal2_score))
    {
      std::cout << "goal2_score:" << goal2_score << std::endl;
      if(goal2_score < 1)
      {
        RobotPose point;
        if(search_gap_point(GAP, m_v_global_pose["robot1"], point))
        {
          g2o::SE3Quat goal2_se3 = computePoseToG2oSE3(robot1_pose).inverse()*computePoseToG2oSE3(point);
          goal2 = computeG2oSE3ToPose(goal2_se3); 
          change_flag2_ = true;
        }
        else
        {
          change_flag2_ = false;
          std::cout << "odom path too short, search error!! will use VFH+ avoidance..." << std::endl;
        }
      }
      else
        change_flag2_ = false;
    }
    else
    {
      change_flag2_ = false;
      std::cout << "score cmpute error!! will use VFH+ avoidance..." << std::endl;
    }
    double goal3_score = 0;
    if(getFormationScore(robot1_se3, relative_goal3, 2*pre_d_ ,goal3_score))
    {
      std::cout << "goal3_score:" << goal3_score << std::endl;
      if(goal3_score < 1)
      {
        RobotPose point;
        if(search_gap_point(2*GAP, m_v_global_pose["robot1"], point))
        {
          g2o::SE3Quat goal3_se3 = computePoseToG2oSE3(robot1_pose).inverse()*computePoseToG2oSE3(point);
          goal3 = computeG2oSE3ToPose(goal3_se3); 
          change_flag3_ = true;
        }
        else
        {
          change_flag3_ = false;
          std::cout << "odom path too short, search error!! will use VFH+ avoidance..." << std::endl;
        }
      }
      else
        change_flag3_ = false;
    }
    else
    {
      change_flag3_ = false;
      std::cout << "score cmpute error!! will use VFH+ avoidance..." << std::endl;
    }
    ros::Time end = ros::Time::now();
    std::cout << "search formation time cost:" << end.toSec() - begin.toSec() << std::endl; 
    robot1_submap.publish(af_submap);
  }
  if(!change_flag2_)
  {
    goal2.x = INITIAL2_X;
    goal2.y = INITIAL2_Y;
    goal2.yaw = INITIAL2_YAW;
  }
  if(!change_flag3_)
  {
    goal3.x = INITIAL3_X;
    goal3.y = INITIAL3_Y;
    goal3.yaw = INITIAL3_YAW;
  }

  // 3. 机器人2误差方程
  double xe = goal2.x - rel_12.x;
  double ye = goal2.y - rel_12.y;
  double te = goal2.yaw - rel_12.yaw;

  double e1 = xe*cos(rel_12.yaw) + ye*sin(rel_12.yaw);
  double e2 = -xe*sin(rel_12.yaw) + ye*cos(rel_12.yaw);
  double et = te;
  
  double vr = currentVel1.linear.x;
  double wr = currentVel1.angular.z;
  double v = currentVel2.linear.x;
  double w = currentVel2.angular.z;

  // 4. 机器人3
  double xe3 = goal3.x - rel_13.x;
  double ye3 = goal3.y - rel_13.y;
  double te3 = goal3.yaw - rel_13.yaw;

  double e13 = xe3*cos(rel_13.yaw) + ye3*sin(rel_13.yaw);
  double e23 = -xe3*sin(rel_13.yaw) + ye3*cos(rel_13.yaw);
  double et3 = te3;
  
  double v3 = currentVel3.linear.x;
  double w3 = currentVel3.angular.z;


  double k1 = 0.1;
  double k2 = 3;
  double k3 = 1;
  
 if(fabs(timestamp - m_v_global_pose["robot2"].back().timestamp) < 1)
 {
    double d = sqrt(e1*e1 + e2*e2);
    double theta = atan2(e2, e1);  
    // std::cout << diff1.x << "\t"<< diff1.y << "\t" << theta  << std::endl;
    //  1.避障控制
    // double r = sqrt(obstacle2_pose.x*obstacle2_pose.x + obstacle2_pose.y*obstacle2_pose.y);
    // RobotPose robot21;
    // RobotPose robot23;
    // robot21 = computeG2oSE3ToPose(computePoseToG2oSE3(robot2_pose).inverse() * computePoseToG2oSE3(robot1_pose));
    // robot23 = computeG2oSE3ToPose(computePoseToG2oSE3(robot2_pose).inverse() * computePoseToG2oSE3(robot3_pose));
    
    // if(r < detect_R2 && (fabs(robot21.x-obstacle2_pose.x) > L1 || fabs(robot21.y-obstacle2_pose.y) > L2) && (fabs(robot23.x-obstacle2_pose.x) > L1 || fabs(robot23.y-obstacle2_pose.y) > L2)) // 避障 
    // {
    //   geometry_msgs::Twist goal;
    //   goal.linear.x = d;
    //   goal.angular.z = theta;
    //   vel1_msg = vfhDriver1->approachGoalCommand(0.1, goal, currentVel2, v_laserScan2);
    // }
    // else
    {
      // double k = 3;
      v = vr*cos(et) + k2*e1; //  + ;
      w = wr + (1/k1)*vr*e2  + k3*sin(et);

      // std::cout << "e1:" << e1 << "\te2:" << e2 << "\tet:" << et << "\nv:" << v << "\tw:" << w << std::endl; 
                                 
      vel2_msg.linear.x = v;
      vel2_msg.angular.z = w;
    }
    if(vel2_msg.linear.x > 0.4) vel2_msg.linear.x = 0.4;
    if(vel2_msg.linear.x < 0) vel2_msg.linear.x = 0;
    if(vel2_msg.angular.z > 1.0) vel2_msg.angular.z = 1.0;
    if(vel2_msg.angular.z < -1.0) vel2_msg.angular.z = -1.0;
 }
 else
 {
  vel2_msg.linear.x = 0;
  vel2_msg.angular.z = 0;
 }

 if(fabs(timestamp - m_v_global_pose["robot3"].back().timestamp) < 1)
 {
    double d = sqrt(e13*e13 + e23*e23);
    double theta = atan2(e23, e13);  ;  
    // std::cout << diff1.x << "\t"<< diff1.y << "\t" << theta  << std::endl;
    // 1.避障控制
    // // 1.避障控制
    // double r = sqrt(obstacle3_pose.x*obstacle3_pose.x + obstacle3_pose.y*obstacle3_pose.y);
    // RobotPose robot31;
    // RobotPose robot32;
    // robot31 = computeG2oSE3ToPose(computePoseToG2oSE3(robot3_pose).inverse() * computePoseToG2oSE3(robot1_pose));
    // robot32 = computeG2oSE3ToPose(computePoseToG2oSE3(robot3_pose).inverse() * computePoseToG2oSE3(robot2_pose));
    
    // if(r < detect_R3 && (fabs(robot31.x-obstacle3_pose.x) > L1 || fabs(robot31.y-obstacle3_pose.y) > L2) && (fabs(robot32.x-obstacle3_pose.x) > L1 || fabs(robot32.y-obstacle3_pose.y) > L2)) // 避障 
    // {
    //   geometry_msgs::Twist goal;
    //   goal.linear.x = d;
    //   goal.angular.z = theta;
    //   vel2_msg = vfhDriver2->approachGoalCommand(0.1, goal, currentVel3, v_laserScan3);
    // }
    // else
    {
      // double k = 3;
      v3 = vr*cos(et3) + k2*e13; //  + ;
      w3 = wr + (1/k1)*vr*e23  + k3*sin(et3);

      // std::cout << "e1:" << e13 << "\te2:" << e23 << "\tet:" << et3 << "\nv:" << v3 << "\tw:" << w3 << std::endl; 
                              
      vel3_msg.linear.x = v3;
      vel3_msg.angular.z = w3;
    }
    if(vel3_msg.linear.x > 0.4) vel3_msg.linear.x = 0.4;
    if(vel3_msg.linear.x < 0) vel3_msg.linear.x = 0;
    if(vel3_msg.angular.z > 1.0) vel3_msg.angular.z = 1.0;
    if(vel3_msg.angular.z < -1.0) vel3_msg.angular.z = -1.0;
 }
 else
 {
  vel3_msg.linear.x = 0;
  vel3_msg.angular.z = 0;
 }
//  std::ofstream file1(ex_file, std::ios::app);
//  file1 << "robot2_ex\t" << e1 << "\trobot3_ex\t" << e13 << "\n";
//  file1.close();

//  std::ofstream file2(ey_file, std::ios::app);
//  file2 << "robot2_ey\t" << e2 << "\trobot3_ey\t" << e23 << "\n";
//  file2.close();

//  std::ofstream file3(et_file, std::ios::app);
//  file3 << "robot2_et\t" << et << "\trobot3_et\t" << et3 << "\n";
//  file3.close();
// std::ofstream file4(all_pose_file, std::ios::app);
//  file4 << "robot1\t" << robot1_pose.x << "\t" << robot1_pose.y << "\t" << robot1_pose.yaw << "\t"
//        << "robot2\t" << robot2_pose.x << "\t" << robot2_pose.y << "\t" << robot2_pose.yaw << "\t"
//        << "robot3\t" << robot3_pose.x << "\t" << robot3_pose.y << "\t" << robot3_pose.yaw << "\n";
//  file4.close();
 robot2_pub.publish(vel2_msg);
 robot3_pub.publish(vel3_msg);
}
void robot2PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double timestamp = msg->header.stamp.toSec();
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  yaw;
  m_v_global_pose["robot2"].push_back(pose);
}
void robot3PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double timestamp = msg->header.stamp.toSec();
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x =  msg->pose.pose.position.x;
  pose.y =  msg->pose.pose.position.y;
  pose.yaw =  yaw;
  m_v_global_pose["robot3"].push_back(pose);
}

void robot1ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   const double LIDAR_ERR = 0.15;
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
      if(i == 540 )
      {
        temp_lr = msg->ranges[i]*sin(i*2*M_PI/720);
         if(isinf(temp_lr))
          temp_lr = 100;
      }
      if(i == 60 )
      {
         temp_ll = msg->ranges[i]*sin(i*2*M_PI/720); 
       if(isinf(temp_ll))
          temp_ll = 100;
      }
   }
   if(min_dis < 3)
   {
     min_point.x=min_dis*cos(min_ang*2*M_PI/720);
     min_point.y=min_dis*sin(min_ang*2*M_PI/720);
     min_point.yaw=min_ang*M_PI/720;
   }
  
   else
   {
      min_point.x = -1000;
      min_point.y = -1000;
   } 
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


void robot1MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  af_submap = *msg;
  subMap_.timestamp = msg->header.stamp.toSec();
  subMap_.resolution = msg->info.resolution;
  subMap_.width = msg->info.width;
  subMap_.height = msg->info.height;

  tf::Quaternion q(msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w);
  double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  subMap_.origin.x = msg->info.origin.position.x;
  subMap_.origin.y = msg->info.origin.position.y;
  subMap_.origin.yaw = yaw;
  subMap_.data = msg->data;

  // std::cout << msg->header.stamp.toSec() << std::endl;
  // std::cout << subMap_.data.size() << "\n"
  //           << subMap_.data[0].size() << "\n"
  //           << subMap_.width << "\n"
  //           << subMap_.height << "\n"
  //           << subMap_.origin.x << "\n"
  //           << subMap_.origin.y << "\n"
  //           << subMap_.origin.yaw << "\n";
}

void robot1AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  static nav_msgs::Path path;
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = msg->header.frame_id;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = msg->header.stamp;
  pose.header.frame_id = msg->header.frame_id;
  pose.pose = msg->pose.pose;
  path.poses.push_back(pose);
  robot1_p_pub.publish(path);
}
void robot2AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  static nav_msgs::Path path;
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = msg->header.frame_id;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = msg->header.stamp;
  pose.header.frame_id = msg->header.frame_id;
  pose.pose = msg->pose.pose;
  path.poses.push_back(pose);
  robot2_p_pub.publish(path);
}
void robot3AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  static nav_msgs::Path path;
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = msg->header.frame_id;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = msg->header.stamp;
  pose.header.frame_id = msg->header.frame_id;
  pose.pose = msg->pose.pose;
  path.poses.push_back(pose);
  robot3_p_pub.publish(path);
}

int main(int argc, char** argv)
{
  std::ofstream file1(ex_file, std::ios::out);
  std::ofstream file2(ey_file, std::ios::out);
  std::ofstream file3(et_file, std::ios::out);
  std::ofstream file4(all_pose_file, std::ios::out);

  ros::init(argc, argv, "multi_robots_formation_node");
  ros::NodeHandle n;
  
  ros::Subscriber sub_robot1_laser = n.subscribe("/robot1/scan", 1000, robot1ScanCallback);
  ros::Subscriber sub_robot2_laser = n.subscribe("/robot2/scan", 1000, robot2ScanCallback);
  ros::Subscriber sub_robot3_laser = n.subscribe("/robot3/scan", 1000, robot3ScanCallback);

#ifdef SIMULATION_ENABLE
  ros::Subscriber sub_robot1_vel = n.subscribe("/robot1/cmd_vel", 1000, robot1CmdVelCallback);
  ros::Subscriber sub_robot2_vel = n.subscribe("/robot2/cmd_vel", 1000, robot2CmdVelCallback);
  ros::Subscriber sub_robot3_vel = n.subscribe("/robot3/cmd_vel", 1000, robot3CmdVelCallback);
  robot2_pub = n.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1);
  robot3_pub = n.advertise<geometry_msgs::Twist>("/robot3/cmd_vel", 1);
  robot1_submap = n.advertise<nav_msgs::OccupancyGrid>("/robot1/af_submap", 1);
  ros::Subscriber sub_robot1_pose = n.subscribe("/robot1/odom", 1000, robot1PoseCallback);
	ros::Subscriber sub_robot2_pose = n.subscribe("/robot2/odom", 1000, robot2PoseCallback);
	ros::Subscriber sub_robot3_pose = n.subscribe("/robot3/odom", 1000, robot3PoseCallback);
  ros::Subscriber sub_robot1_map = n.subscribe("/robot1/submap", 1, robot1MapCallback);
#else
  ros::Subscriber sub_robot1_vel = n.subscribe("/robot1/cmd_vel_mux/input/teleop", 1000, robot1CmdVelCallback);
  ros::Subscriber sub_robot2_vel = n.subscribe("/robot2/cmd_vel_mux/input/teleop", 1000, robot2CmdVelCallback);
  ros::Subscriber sub_robot3_vel = n.subscribe("/robot3/cmd_vel_mux/input/teleop", 1000, robot3CmdVelCallback);
  robot2_pub = n.advertise<geometry_msgs::Twist>("/robot2/cmd_vel_mux/input/teleop", 1);
  robot3_pub = n.advertise<geometry_msgs::Twist>("/robot3/cmd_vel_mux/input/teleop", 1);
  robot1_submap = n.advertise<nav_msgs::OccupancyGrid>("/robot1/af_submap", 1);

  ros::Subscriber sub_robot1_pose = n.subscribe("/robot1/est_odom", 1000, robot1PoseCallback);
	ros::Subscriber sub_robot2_pose = n.subscribe("/robot2/est_odom", 1000, robot2PoseCallback);
	ros::Subscriber sub_robot3_pose = n.subscribe("/robot3/est_odom", 1000, robot3PoseCallback);
  ros::Subscriber sub_robot1_map = n.subscribe("/robot1/submap", 1, robot1MapCallback);
#endif

  ros::Subscriber sub_amcl1 = n.subscribe("/robot1/amcl_pose", 1000, robot1AMCLCallback);
  ros::Subscriber sub_amcl2 = n.subscribe("/robot2/amcl_pose", 1000, robot2AMCLCallback);
  ros::Subscriber sub_amcl3 = n.subscribe("/robot3/amcl_pose", 1000, robot3AMCLCallback);
  
  // ros::Subscriber sub_amcl1 = n.subscribe("/robot1/est_odom", 1000, robot1OCallback);
  // ros::Subscriber sub_amcl2 = n.subscribe("/robot2/est_odom", 1000, robot2OCallback);
  // ros::Subscriber sub_amcl3 = n.subscribe("/robot3/est_odom", 1000, robot3OCallback);
  
  robot1_p_pub = n.advertise<nav_msgs::Path>("robot1/path", 10);
  robot2_p_pub = n.advertise<nav_msgs::Path>("robot2/path", 10);
  robot3_p_pub = n.advertise<nav_msgs::Path>("robot3/path", 10);
  ros::spin();
  return 0;
}
