#ifndef RELATIVE_LOCALIZATION_NODE_H_
#define RELATIVE_LOCALIZATION_NODE_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "nlink_parser/LinktrackNodeframe2.h"

#include <sstream>
#include <boost/circular_buffer.hpp>
#include <numeric>
#include <iostream>
#include <fstream>
#include <cmath>

#include "pf2_6d.h"
#include "gnuplot.h"
#include "text/to_string.h"


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
#include "g2o/solvers/csparse/linear_solver_csparse.h"
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen> 
#include <pthread.h>
#include <mutex>
#include <queue>

using namespace std;
using namespace Eigen;
using namespace g2o;




typedef Eigen::Matrix<double, 12, 1> Vector12d;
double dx=0;
double dy=0;
class RobotPose{
public:
	RobotPose(){};
public:
    double timestamp;
	double x;
	double y;
	double yaw;
	double accumulated_dist;
	double accumulated_orientation;
	double uwbCovariance;
};
class IndexNum{
public: 
    IndexNum(){index1 = index2 = index3 = 0;}
public:
    int index1;
    int index2;
    int index3;
    int index4;
};

class UWBConfigure{
public:
	UWBConfigure(){};
public:
	int uwb_id;
	RobotPose uwb_pose;
};

class UWBMessage{
    public:
        UWBMessage(double a=0, double b=0, double c=0, double d=0){
            timestamp = a; range = b; rxRssi = c; fpRssi = d;
        }
    public:
	double timestamp;
    double range;
    double rxRssi;
    double fpRssi;
        
};

class G2OMeasure{
public:
    G2OMeasure(){}
public:
    double distance;
    double tag_delta_x;
    double tag_delta_y;
    double anchor_delta_x;
    double anchor_delta_y;
    double tag_T_x;
    double tag_T_y;
    double tag_T_yaw;
    double anchor_T_x;
    double anchor_T_y;
    double anchor_T_yaw;
    double weight;
};

class CommonPose
{
public: 
	CommonPose(){};
public:
	double timeStamp;
	RobotPose tagPose;
	RobotPose anchorPose;
};

boost::circular_buffer<UWBMessage> v_robot12_uwb_measure(40000);
boost::circular_buffer<UWBMessage> v_robot13_uwb_measure(40000);
boost::circular_buffer<UWBMessage> v_robot21_uwb_measure(40000);
boost::circular_buffer<UWBMessage> v_robot23_uwb_measure(40000);
boost::circular_buffer<UWBMessage> v_robot31_uwb_measure(40000);
boost::circular_buffer<UWBMessage> v_robot32_uwb_measure(40000);

std::map<std::string, std::vector<RobotPose> > m_v_amcl;
std::map<std::string, std::vector<RobotPose> > m_v_odom;


std::map<std::string, std::vector<CommonPose> > m_v_com_odom;
std::map<std::string, vector<CommonPose> > m_v_com_amcl;

RobotPose robot1_previous_odom;
RobotPose robot2_previous_odom;
RobotPose robot3_previous_odom;
RobotPose robot1_current_odom;
RobotPose robot2_current_odom;
RobotPose robot3_current_odom;
double robot1_acc_distance=-1;
double robot1_acc_angle=-1;
double robot2_acc_distance=-1;
double robot2_acc_angle=-1;
double robot3_acc_distance=-1;
double robot3_acc_angle=-1;

double firstTimestamp = 0;
std::map<std::string, RobotPose> m_old_odom;
std::map<std::string, RobotPose> m_old_rel;
std::map<std::string, CommonPose> m_old_odom_pose;
std::map<std::string, int> m_flag1;
std::map<std::string, std::vector<UWBMessage> > m_v_win_uwb;
std::map<std::string, int > m_flag;
std::map<std::string, IndexNum> m_index_record;
// RobotPose old_rel;
// RobotPose old_odom;
// CommonPose old_odom_pose;
std::vector<double> v_error1;
std::vector<double> v_error2;
std::vector<double> v_error3;
std::map<std::string, RobotPose> m_est_rel;

std::map<std::string, double> m_old_odom_anchor_acc_dist;
std::map<std::string, double> m_old_odom_tag_acc_dist;
std::map<std::string, double> m_old_odom_anchor_acc_angle;
std::map<std::string, double> m_old_odom_tag_acc_angle;

class ObjectConfig
{
public: ObjectConfig(){}
std::map <std::string, std::vector<RobotPose> > g2o_estimate;
};
std::map< std::string, std::vector<RobotPose> > m_v_robot_pose_g2o;
std::map< std::string, ObjectConfig > m_v_rel_pose_g2o;
std::map< std::string, std::vector<RobotPose> > m_v_odom_pose_g2o;

RobotPose est_current_odom1 ;
RobotPose est_current_odom2 ;
RobotPose est_current_odom3 ;

RobotPose est_current_odom1_g2o;
RobotPose est_current_odom2_g2o;
RobotPose est_current_odom3_g2o;

int get_odom1_flag=0;
int get_odom2_flag=0;
int get_odom3_flag=0;

std::map< std::string, std::vector<RobotPose> > m_v_est_globoal_pose;


std::map<std::string, std::vector<RobotPose> > m_v_amcl_drone;

std::vector<double> v_rel_yaw_error;
std::vector<double> v_rel_position_error;
std::vector<double> v_rel_uwb_error;
std::map<std::string, std::vector<RobotPose>> m_v_dis_opt_global_pose;
std::vector<double> v_dis_dist_std;
std::vector<double> v_dis_rxfx_std;
std::vector<double> v_dis_rxfx;
std::vector<double> v_dis_fx;
std::vector<double> v_dis_op_uwb;
std::vector<double> v_dis_raw_uwb;
std::vector<double> v_dis_time;
std::vector<double> v_odom_yaw_error;
std::vector<double> v_odom_position_error;
std::map<std::string, RobotPose> m_goal;

std::mutex m_mtx_pose_est; 

std::mutex m_mtx_odom_est;



#endif