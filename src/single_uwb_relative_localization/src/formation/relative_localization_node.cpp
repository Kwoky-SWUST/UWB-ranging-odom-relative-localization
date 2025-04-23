#include "relative_localization_node.h"
#define INF 0x7fffffff
int TIME_WINDOW_SIZE = 40;
int TIME_GRAP = 3;
const int NUM_POINTS = 20;
const int LAMDA = 20;
const double UPDATE_DIST = 0.20;  // m
const double UPDATE_ANGLE = 0.20; // rad
const double UPDATE_TIME = 0.5;  // s
const double FINAL_TIME_WINDOW_SIZE = 60;
const double time_limit = 0.5;

const double INITIAL1_X = 0;
const double INITIAL1_Y = 0;
const double INITIAL1_YAW = 0;
const double INITIAL2_X = -0.6;
const double INITIAL2_Y = 0.6;
const double INITIAL2_YAW = 0;
const double INITIAL3_X = -0.6;
const double INITIAL3_Y = -0.6;
const double INITIAL3_YAW = 0;

// const double ROTATION_X = 0.5;
// const double ROTATION_Y = -0.1;
// const double ROTATION_YAW = 0.05;

const double ROTATION_X = 0;
const double ROTATION_Y = 0;
const double ROTATION_YAW = 0;
// #define INITIAL_POSE_IS_AMCL
// #define SIMULATION_ENABLE
// #define SAVE_G2O_DATA

ros::Publisher robot1_pub;
ros::Publisher robot2_pub;
ros::Publisher robot3_pub;

//------------------------------------------------------------------------
std::string uwb_measure_name("/home/dzy/Desktop/uwb_measure.txt");
std::string amcl_measure_name("/home/dzy/Desktop/amcl_measure.txt");
std::string odom_measure_name("/home/dzy/Desktop/odom_measure.txt");
std::string test_file_name("/home/dzy/Desktop/test1.txt");
std::string com_trues_dist_file("/home/dzy/Desktop/trues_dist.txt");
std::string com_uwb_measure_file("/home/dzy/Desktop/uwb_dist.txt");
std::string com_odom_file("/home/dzy/Desktop/com_odom.txt");
std::string com_amcl_file("/home/dzy/Desktop/com_amcl.txt");
std::string gt_file_name("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/data/amcl_measure.txt");
// g2o
std::string drone_odom_file1("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/drone1_odom.txt");
std::string drone_odom_file2("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/drone2_odom.txt");
std::string drone_odom_file3("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/drone3_odom.txt");
std::string drone_AMCL_file1("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/gt_1_interpolation.txt");
std::string drone_AMCL_file2("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/gt_2_interpolation.txt");
std::string drone_AMCL_file3("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/gt_3_interpolation.txt");
std::string drone_est12_file("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/params_estimate_1_2.txt");
std::string drone_est13_file("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/params_estimate_1_3.txt");
std::string drone_est23_file("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/params_estimate_2_3.txt");
std::string drone_rel_pose_file("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/output/relative_pose.txt");
//-------------------------------------------------------------------------

Matrix3d covariance1;
Matrix3d covariance2;
Matrix3d covariance3;
const double alpha1 = 0.5;
const double alpha2 = 0.5;
const double alpha3 = 0.5;
const double alpha4 = 0.5;

double angle_diff(double a, double b)
{
	double d1, d2;
	a = carmen_normalize_theta(a);
	b = carmen_normalize_theta(b);
	d1 = a-b;
	d2 = 2*M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
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
    pose.yaw = carmen_normalize_theta(euler[2]);
    return pose;
}

// No Matrix Coordinate-Transformation  >> time cast: 7.5 times faster than Matrix method
// return value = T1.inverse() * T2;
inline RobotPose computeT1inverse_x_T2(RobotPose T1, RobotPose T2)
{
	RobotPose relative_pose;
	double deno_ = cos(T1.yaw) + tan(T1.yaw)*sin(T1.yaw);
	relative_pose.x = (T2.x-T1.x+tan(T1.yaw)*(T2.y - T1.y))
						/ (deno_);
	relative_pose.y = (T2.y - T1.y - tan(T1.yaw)*(T2.x - T1.x))
						/ (deno_); 
	relative_pose.yaw =carmen_normalize_theta(T2.yaw - T1.yaw);
	return relative_pose;
}
// return value = T1 * T2;
inline RobotPose computeT1_x_T2(RobotPose T1, RobotPose T2)
{
	RobotPose relative_pose;
	relative_pose.x = T1.x + T2.x*cos(T1.yaw) - T2.y*sin(T1.yaw);
	relative_pose.y = T1.y + T2.x*sin(T1.yaw) + T2.y*cos(T1.yaw); 
	relative_pose.yaw =carmen_normalize_theta(T1.yaw+T2.yaw);
	return relative_pose;
}


void computeMeanVariance(std::vector<double> v_error, double &mean, double &std_dev)
{

	double sum_error=0;
	double sum_variance_error=0;

	for(int i=0;i<v_error.size();i++)
	{
		sum_error=sum_error+v_error[i];
	}
	mean=sum_error/((double)(v_error.size()));

	for(int i=0;i<v_error.size();i++)
	{
		sum_variance_error=sum_variance_error+(v_error[i]-mean)*(v_error[i]-mean);
	}

	std_dev=sqrt(sum_variance_error/((double)(v_error.size())));
}

// define G2O inherit
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexParams()
	{
	}

	virtual bool read(std::istream& /*is*/)
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	virtual bool write(std::ostream& /*os*/) const
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	virtual void setToOriginImpl()
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
	}

	virtual void oplusImpl(const double* update)
	{
		Eigen::Vector3d::ConstMapType v(update);
		_estimate += v;
	}
};

class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, Vector12d, VertexParams>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgePointOnCurve()
	{
	}
	virtual bool read(std::istream& /*is*/)
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}
	virtual bool write(std::ostream& /*os*/) const
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	
	void computeError()
	{
		const VertexParams* params = static_cast<const VertexParams*>(vertex(0));
		const double& x = params->estimate()(0);
		const double& y = params->estimate()(1);
		const double& theta = params->estimate()(2);

		double delta_tag_x=measurement()(0);
		double delta_tag_y=measurement()(1);		
		double x_tag=measurement()(2);
		double y_tag=measurement()(3);
		double theta_tag=measurement()(4);

		double delta_peer_tag_x=measurement()(5);
		double delta_peer_tag_y=measurement()(6);
		double peer_tag_x=measurement()(7);
		double peer_tag_y=measurement()(8);
		double peer_tag_theta=measurement()(9);

		//measured distance
		double measued_d=measurement()(10);

		double weight = measurement()(11);
		//we add x y theta for the tag

		//estimation*odom

	
		Eigen::Vector3d tag_odom_trans(x_tag,y_tag,0);
		tf2::Quaternion tag_odom_quaternion;
		tag_odom_quaternion.setRPY( 0, 0, theta_tag);
		Eigen::Quaterniond tag_odom_q(tag_odom_quaternion.w(),tag_odom_quaternion.x(),tag_odom_quaternion.y(),tag_odom_quaternion.z());//w,x,y,z
		g2o::SE3Quat tag_odom_se3(tag_odom_q,tag_odom_trans);	


		Eigen::Vector3d estimation_trans(x,y,0);
		tf2::Quaternion estimation_quaternion;
		estimation_quaternion.setRPY( 0, 0, theta);
		Eigen::Quaterniond estimation_q(estimation_quaternion.w(),estimation_quaternion.x(),estimation_quaternion.y(),estimation_quaternion.z());//w,x,y,z
		g2o::SE3Quat estimation_se3(estimation_q,estimation_trans);	



		g2o::SE3Quat relative_estmation_odom=estimation_se3*tag_odom_se3;//relative transformtion T=A*B,

		Eigen::Quaterniond qd_relative_estimation_odom= relative_estmation_odom.rotation();
		Eigen::Vector3d trans_relative_estimation_odom=relative_estmation_odom.translation();		// x y	
		auto euler_estimation_odom = qd_relative_estimation_odom.toRotationMatrix().eulerAngles(0, 1, 2);
		double relative_yaw = euler_estimation_odom[2]; //angle 
		//we compute the relative odom measurements


		double x_predicted_tag=trans_relative_estimation_odom.x()+delta_tag_x*cos(relative_yaw)-delta_tag_y*sin(relative_yaw);
		double y_predicted_tag=trans_relative_estimation_odom.y()+delta_tag_x*sin(relative_yaw)+delta_tag_y*cos(relative_yaw);

		//assume the x, y, and theta are zero
		double x_predicted_peer_tag= peer_tag_x + delta_peer_tag_x*cos(peer_tag_theta)-delta_peer_tag_y*sin(peer_tag_theta);
		double y_predicted_peer_tag= peer_tag_y + delta_peer_tag_x*sin(peer_tag_theta)+delta_peer_tag_y*cos(peer_tag_theta);

		double fval = sqrt((x_predicted_tag-x_predicted_peer_tag)*(x_predicted_tag-x_predicted_peer_tag)+(y_predicted_tag-y_predicted_peer_tag)*(y_predicted_tag-y_predicted_peer_tag));

		_error(0) = weight * fabs(fval - measued_d);
	}
};

GnuplotInterface * plot_uwb_pose = new GnuplotInterface();
std::map<std::string, std::vector<RobotPose> > m_v_uwb_rel;
void plotUWBRel(int minX=-8, int maxX=15, int minY=-8, int maxY=20)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'UWB Positioning Track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
    for(it_amcl=m_v_amcl.begin(); it_amcl!=m_v_amcl.end(); it_amcl++)
    {
        std::string line_colour; 
        std::string line_type;
        std::string line_name;
        if(it_amcl->first == "robot1")
        {
            line_colour = "#C0C0C0";
            line_type = "1";
        }  
        else if(it_amcl->first == "robot2")
        {
            line_colour = "#C0C0C0";
            line_type = "3";
        } 
        else if(it_amcl->first == "robot3")
        {
            line_colour = "#C0C0C0";
            line_type = "5";
        }
        else
        {
            continue;
        } 
        cmd+="'-' u 1:2 w lp lw 2 pt "+ line_type +" ps 1 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"-gt',";
    }
	for(it_amcl=m_v_uwb_rel.begin(); it_amcl!=m_v_uwb_rel.end(); it_amcl++)
    {
        std::string line_colour; 
        std::string line_type;
        std::string line_name;
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
        cmd+="'-' u 1:2 w lp lw 2 pt "+ line_type +" ps 1 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"-uwb',";
    }
    cmd += "\n";
	for(it_amcl=m_v_amcl.begin(); it_amcl!=m_v_amcl.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
    for(it_amcl=m_v_uwb_rel.begin(); it_amcl!=m_v_uwb_rel.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	plot_uwb_pose->commandStr( cmd );
}
//------------------------------------------------------------------------
GnuplotInterface * plot_odom = new GnuplotInterface();
void plotODOM(int minX=-8, int maxX=15, int minY=-8, int maxY=20)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'ODOM Track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
    for(it_amcl=m_v_odom.begin(); it_amcl!=m_v_odom.end(); it_amcl++)
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
        cmd+="'-' u 1:2 w lp lw 2 pt "+ line_type +" ps 1 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"',";
    }
    cmd += "\n";
    for(it_amcl=m_v_odom.begin(); it_amcl!=m_v_odom.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	
	plot_odom->commandStr( cmd );
}

GnuplotInterface * plot_est_glo = new GnuplotInterface();
void plotEstGlo(int minX=-10, int maxX=15, int minY=-15, int maxY=15)
{std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title '基于图优化+里程计的定位轨迹'\n";
	cmd+="set size ratio -1\n";	
	cmd+="unset arrow\n";

	std::map <std::string, std::vector< RobotPose >>::iterator it;


	cmd+="set key top right height 0.1 width -0.5\n";


	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";

	int line_style=1;
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
	for(it_amcl=m_v_est_globoal_pose.begin(); it_amcl!=m_v_est_globoal_pose.end(); it_amcl++)
    {
        std::string line_colour; 
        std::string line_type;
		std::string line_name;
        if(it_amcl->first == "robot1")
        {

            line_colour = "#FF0000";
			line_name = "机器人1图优化+里程计定位轨迹";
            line_type = "1";
        }  
        else if(it_amcl->first == "robot2")
        {
            line_colour = "#0000FF";
			line_name = "机器人2图优化+里程计定位轨迹";
            line_type = "3";
        } 
        else if(it_amcl->first == "robot3")
        {
            line_colour = "#000000";
			line_name = "机器人3图优化+里程计定位轨迹";
            line_type = "5";
        }
        else
        {
            continue;
        } 
       	cmd+="'-' u 1:2 w lp lw 1 pt 7 ps 0.5 lc rgb '"+line_colour+"' ti '"+line_name+"',";
    }
  
    cmd += "\n"; 

    for(it_amcl=m_v_est_globoal_pose.begin(); it_amcl!=m_v_est_globoal_pose.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	
	plot_est_glo->commandStr( cmd );
}
//------------------------------------------------------------------------
GnuplotInterface * plot_amcl = new GnuplotInterface();
void plotAMCL(int minX=0, int maxX=10, int minY=0, int maxY=18)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'times'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'AMCL Track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
    for(it_amcl=m_v_amcl.begin(); it_amcl!=m_v_amcl.end(); it_amcl++)
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
        cmd+="'-' u 1:2 w lp lw 2 pt "+ line_type +" ps 1 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"',";
    }
    cmd += "\n";
    for(it_amcl=m_v_amcl.begin(); it_amcl!=m_v_amcl.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	
	plot_amcl->commandStr( cmd );
}
//------------------------------------------------------------------------
GnuplotInterface * plot_com_odom_amcl = new GnuplotInterface();
void plotComODOMAMCL(std::vector<CommonPose> v_com_amcl, std::vector<CommonPose> v_com_odom)
{
	int minX=0;
	int maxX=10;
	int minY=0;
	int maxY=18;
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'times'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'ODOM and AMCL track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
    cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#FF0000' ti 'AMCL2',";
    cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#0000FF' ti 'AMCL1',";
    cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#00FF00' ti 'ODOM2',";
    cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#000000' ti 'ODOM1',";
    cmd += "\n";

    for(int i=0;i<v_com_amcl.size();i=i+1)
    {
        // if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
        //     continue;
        cmd += toString( v_com_amcl[i].anchorPose.x ) + ' ' + toString( v_com_amcl[i].anchorPose.y ) + ' ' + toString( 0.5 ) + '\n';
    }
    cmd += "e\n";

    for(int i=0;i<v_com_amcl.size();i=i+1)
    {
        //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
        //     continue;
        cmd += toString( v_com_amcl[i].tagPose.x ) + ' ' + toString( v_com_amcl[i].tagPose.y ) + ' ' + toString( 0.5 ) + '\n';
    }
    cmd += "e\n";

    for(int i=0;i<v_com_odom.size();i=i+1)
    {
        //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
        //     continue;
        cmd += toString( v_com_odom[i].anchorPose.x ) + ' ' + toString( v_com_odom[i].anchorPose.y ) + ' ' + toString( 0.5 ) + '\n';
    }
    cmd += "e\n";

    for(int i=0;i<v_com_odom.size();i=i+1)
    {
        //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
        //     continue;
        cmd += toString( v_com_odom[i].tagPose.x ) + ' ' + toString( v_com_odom[i].tagPose.y ) + ' ' + toString( 0.5 ) + '\n';
    }
    cmd += "e\n";
	
	plot_com_odom_amcl->commandStr( cmd );
}
//------------------------------------------------------------------------
GnuplotInterface * plot_win = new GnuplotInterface();
std::map<std::string, std::vector<RobotPose> > m_v_win;
void plotWindow(int minX=-8, int maxX=15, int minY=-8, int maxY=20)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'odom window Track'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
   
    std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
    for(it_amcl=m_v_win.begin(); it_amcl!=m_v_win.end(); it_amcl++)
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
        cmd+="'-' u 1:2 w lp lw 2 pt "+ line_type +" ps 1 lc rgb '"+ line_colour +"' ti '"+it_amcl->first+"',";
    }
    cmd += "\n";
    for(it_amcl=m_v_win.begin(); it_amcl!=m_v_win.end(); it_amcl++)
    {
        std::vector<RobotPose> v_amcl=it_amcl->second;
        for(int i=0;i<v_amcl.size();i=i+1)
        {
            cmd += toString( v_amcl[i].x ) + ' ' + toString( v_amcl[i].y ) + ' ' + toString( 0.5 ) + '\n';
        }
        cmd += "e\n";
    }
	
	plot_win->commandStr( cmd );
}
GnuplotInterface * plot_uwb = new GnuplotInterface();
std::vector<double> v_op_uwb;
std::vector<double> v_trues_dist;
void plotUWBMeasure(int minX=0, int maxX=650, int minY=0, int maxY=20)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'time(s)'\n";
	cmd+="set ylabel 'distance(m)'\n";
	cmd+="set title 'Ranging Results'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+to_string(minX)+':'+to_string(maxX)+"]["+to_string(minY)+':'+to_string(maxY)+"] ";
	
   
    
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#FF0000' ti 'UWB Measure',";
    // cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#0000FF' ti 'KF Estimate',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#000000' ti 'Ground Trues',";
    cmd += "\n";
    boost::circular_buffer<UWBMessage> v_uwb = v_robot21_uwb_measure;
    for(int i=0;i<v_uwb.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_uwb[i].range ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";
    // for(int i=0;i<v_op_uwb.size();i=i+1)
    // {
    //     cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_op_uwb[i] ) + ' ' + to_string( 0.5 ) + '\n';
    // }
    // cmd += "e\n";
    for(int i=0;i<v_trues_dist.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_trues_dist[i]) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";
	
    plot_uwb->commandStr( cmd );
}
//---------------------------------------------------------------------
GnuplotInterface * plotrx_fx = new GnuplotInterface();
std::vector<double> v_fx_rx;
std::vector<double> v_nlos_fx_rx;
std::vector<double> v_fp_;
std::vector<double> v_rx_;
void plotRx_fx(int minX=0, int maxX=650, int minY=0, int maxY=25)
{
	std::string cmd;

	cmd+="set grid\n";
	cmd+="set xlabel 'time(s)'\n";
	cmd+="set ylabel 'signal(dB)'\n";
	cmd+="set title 'rxRSSI-fpRSSI'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";
	cmd+="set key top right height 0.5 width -2\n";
	cmd+="plot ["+to_string(minX)+':'+to_string(maxX)+"]["+to_string(minY)+':'+to_string(maxY)+"] ";
	
   
    
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#FF0000' ti 'rx-fp',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#0000FF' ti 'nlos',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#00FF00' ti 'nlos limit 10dB',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#000000' ti 'los limit 6dB',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#00F000' ti 'FP',";
    cmd+="'-' u 1:2 w l lw 2 pt 1 ps 1 lc rgb '#00000F' ti 'RX',";
    cmd += "\n";
    
    boost::circular_buffer<UWBMessage> v_uwb = v_robot12_uwb_measure;
    for(int i=0;i<v_fx_rx.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_fx_rx[i] ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";
    for(int i=0;i<v_nlos_fx_rx.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_nlos_fx_rx[i] ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";
   
    for(int i=0;i<v_uwb.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( 10) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";
	
    for(int i=0;i<v_uwb.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( 6 ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";

    for(int i=0;i<v_fp_.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_fp_[i] ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";

    for(int i=0;i<v_rx_.size();i=i+1)
    {
        cmd += to_string( v_uwb[i].timestamp) + ' ' + to_string( v_rx_[i] ) + ' ' + to_string( 0.5 ) + '\n';
    }
    cmd += "e\n";

    plotrx_fx->commandStr( cmd );
}
//------------------------------------------------------------------------
// --------------------------------------------------------------------------
GnuplotInterface * m_plot_g2o_track = new GnuplotInterface();
std::map<std::string, std::vector<RobotPose> > m_g2o_results;
double m_dBinningMinX=-10;
double m_dBinningMaxX=15;
double m_dBinningMinY=-15;
double m_dBinningMaxY=15;
// --------------------------------------------------------------------------
void plot_all_g2o_tracks()
// --------------------------------------------------------------------------
{

	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title '基于图优化的定位轨迹'\n";
	cmd+="set size ratio -1\n";	
	cmd+="unset arrow\n";

	std::map <std::string, std::vector< RobotPose >>::iterator it;


	cmd+="set key top right height 0.1 width -0.5\n";


	cmd+="plot ["+toString(m_dBinningMinX)+':'+toString(m_dBinningMaxX)+"]["+toString(m_dBinningMinY)+':'+toString(m_dBinningMaxY)+"] ";

	int line_style=1;
std::map <std::string, std::vector<RobotPose> >::iterator it_amcl;
	//set the line colors
	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	std::string line_color;
	// 	std::string line_name;
	// 	if(it_amcl->first=="robot1")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人1真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot2")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人2真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot3")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人3真实轨迹";
	// 	}			

	// 	//cmd+="'-' u 1:2 w lp lw 1 pt 6 ps 1 lc "+toString(line_style)+" ti '"+it->first+"',";
	// 	cmd+="'-' u 1:2 w l lw 2 pt 6 ps 1 lc rgb '"+line_color+"' ti '"+line_name+"',";
	// 	line_style=line_style+1;
	// }
	//set the line colors
	for(it = m_g2o_results.begin(); it != m_g2o_results.end(); ++it)
	{
		std::string line_color="#00FF00";	
		std::string line_name;
		if(it->first=="drone1")
		{
			line_color="#FF0000";
			line_name = "机器人1图优化轨迹";
		}			
		else if(it->first=="drone2")
		{
			line_color="#0000FF";
			line_name = "机器人2图优化轨迹";
		}			
		else if(it->first=="drone3")
		{
			line_color="#000000";
			line_name = "机器人3图优化轨迹";
		}			

		
		cmd+="'-' u 1:2 w lp lw 1 pt 7 ps 0.5 lc rgb '"+line_color+"' ti '"+line_name+"',";
		line_style=line_style+1;
	}
	
	cmd += "\n";

	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	//output the measurements
	// 	std::vector< RobotPose > trajectories=it_amcl->second;

	// 	for(int i=0;i<trajectories.size();i=i+1)
	// 	{
	// 		cmd += toString( trajectories[i].x ) + ' ' + toString( trajectories[i].y ) + ' ' + toString( 0.5 ) + '\n';
	// 	}
	// 	cmd += "e\n";
	// }
	for(it = m_g2o_results.begin(); it != m_g2o_results.end(); ++it)
	{

		//output the measurements
		std::vector< RobotPose > trajectories_low_frequency=it->second;

		for(int i=0;i<trajectories_low_frequency.size();i=i+1)
		{
		cmd += toString( trajectories_low_frequency[i].x ) + ' ' + toString( trajectories_low_frequency[i].y ) + toString( 0.5 ) + '\n';				

		}
		cmd += "e\n";
	}

	m_plot_g2o_track->commandStr( cmd );
}//---------------------------------------------------------------------------


GnuplotInterface * m_plot_odom_track = new GnuplotInterface();
void plot_all_odom_tracks()
// --------------------------------------------------------------------------
{

	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title '基于里程计的定位轨迹'\n";
	cmd+="set size ratio -1\n";	
	cmd+="unset arrow\n";

	std::map <std::string, std::vector< RobotPose >>::iterator it;


	cmd+="set key top right height 0.1 width -0.5\n";


	cmd+="plot ["+toString(m_dBinningMinX)+':'+toString(m_dBinningMaxX)+"]["+toString(m_dBinningMinY)+':'+toString(m_dBinningMaxY)+"] ";

	int line_style=1;
std::map <std::string, std::vector<RobotPose> >::iterator it_amcl;
	//set the line colors
	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	std::string line_color;
	// 	std::string line_name;
	// 	if(it_amcl->first=="robot1")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人1真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot2")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人2真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot3")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人3真实轨迹";
	// 	}			

	// 	//cmd+="'-' u 1:2 w lp lw 1 pt 6 ps 1 lc "+toString(line_style)+" ti '"+it->first+"',";
	// 	cmd+="'-' u 1:2 w l lw 2 pt 6 ps 1 lc rgb '"+line_color+"' ti '"+line_name+"',";
	// 	line_style=line_style+1;
	// }
	//set the line colors
	for(it = m_v_odom.begin(); it != m_v_odom.end(); ++it)
	{
		std::string line_color="#00FF00";	
		std::string line_name;
		if(it->first=="robot1")
		{
			line_color="#FF0000";
			line_name = "机器人1里程计轨迹";
		}			
		else if(it->first=="robot2")
		{
			line_color="#0000FF";
			line_name = "机器人2里程计轨迹";
		}			
		else if(it->first=="robot3")
		{
			line_color="#000000";
			line_name = "机器人3里程计轨迹";
		}			

		
		cmd+="'-' u 1:2 w lp lw 1 pt 7 ps 0.5 lc rgb '"+line_color+"' ti '"+line_name+"',";
		line_style=line_style+1;
	}
	
	cmd += "\n";

	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	//output the measurements
	// 	std::vector< RobotPose > trajectories=it_amcl->second;

	// 	for(int i=0;i<trajectories.size();i=i+1)
	// 	{
	// 		cmd += toString( trajectories[i].x ) + ' ' + toString( trajectories[i].y ) + ' ' + toString( 0.5 ) + '\n';
	// 	}
	// 	cmd += "e\n";
	// }
	for(it = m_v_odom.begin(); it != m_v_odom.end(); ++it)
	{

		//output the measurements
		std::vector< RobotPose > trajectories_low_frequency=it->second;

		for(int i=0;i<trajectories_low_frequency.size();i=i+1)
		{
		cmd += toString( trajectories_low_frequency[i].x ) + ' ' + toString( trajectories_low_frequency[i].y ) + toString( 0.5 ) + '\n';				

		}
		cmd += "e\n";
	}

	m_plot_odom_track->commandStr( cmd );
}//---------------------------------------------------------------------------


GnuplotInterface * m_plot_odomuwb_track = new GnuplotInterface();
void plot_all_odomuwb_tracks()
// --------------------------------------------------------------------------
{

	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title '图优化结合里程计的定位轨迹'\n";
	cmd+="set size ratio -1\n";	
	cmd+="unset arrow\n";

	std::map <std::string, std::vector< RobotPose >>::iterator it;


	cmd+="set key top right height 0.1 width -0.5\n";


	cmd+="plot ["+toString(m_dBinningMinX)+':'+toString(m_dBinningMaxX)+"]["+toString(m_dBinningMinY)+':'+toString(m_dBinningMaxY)+"] ";

	int line_style=1;
std::map <std::string, std::vector<RobotPose> >::iterator it_amcl;
	//set the line colors
	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	std::string line_color;
	// 	std::string line_name;
	// 	if(it_amcl->first=="robot1")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人1真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot2")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人2真实轨迹";
	// 	}			
	// 	else if(it_amcl->first=="robot3")
	// 	{
	// 		line_color="#7F7F7F";
	// 		line_name = "机器人3真实轨迹";
	// 	}			

	// 	//cmd+="'-' u 1:2 w lp lw 1 pt 6 ps 1 lc "+toString(line_style)+" ti '"+it->first+"',";
	// 	cmd+="'-' u 1:2 w l lw 2 pt 6 ps 1 lc rgb '"+line_color+"' ti '"+line_name+"',";
	// 	line_style=line_style+1;
	// }
	//set the line colors
	for(it = m_v_est_globoal_pose.begin(); it != m_v_est_globoal_pose.end(); ++it)
	{
		std::string line_color="#00FF00";	
		std::string line_name;
		if(it->first=="robot1")
		{
			line_color="#FF0000";
			line_name = "机器人1轨迹";
		}			
		else if(it->first=="robot2")
		{
			line_color="#0000FF";
			line_name = "机器人2轨迹";
		}			
		else if(it->first=="robot3")
		{
			line_color="#000000";
			line_name = "机器人3轨迹";
		}			

		
		cmd+="'-' u 1:2 w lp lw 1 pt 7 ps 0.5 lc rgb '"+line_color+"' ti '"+line_name+"',";
		line_style=line_style+1;
	}
	
	cmd += "\n";

	// for(it_amcl = m_v_amcl.begin(); it_amcl != m_v_amcl.end(); ++it_amcl)
	// {
	// 	//output the measurements
	// 	std::vector< RobotPose > trajectories=it_amcl->second;

	// 	for(int i=0;i<trajectories.size();i=i+1)
	// 	{
	// 		cmd += toString( trajectories[i].x ) + ' ' + toString( trajectories[i].y ) + ' ' + toString( 0.5 ) + '\n';
	// 	}
	// 	cmd += "e\n";
	// }
	for(it = m_v_est_globoal_pose.begin(); it != m_v_est_globoal_pose.end(); ++it)
	{

		//output the measurements
		std::vector< RobotPose > trajectories_low_frequency=it->second;

		for(int i=0;i<trajectories_low_frequency.size();i=i+1)
		{
		cmd += toString( trajectories_low_frequency[i].x ) + ' ' + toString( trajectories_low_frequency[i].y ) + toString( 0.5 ) + '\n';				

		}
		cmd += "e\n";
	}

	m_plot_odomuwb_track->commandStr( cmd );
}//---------------------------------------------------------------------------

std::map<std::string, std::vector<RobotPose> > m_v_amcl1;
void loadAMCLFile(std::string file_name)
{
	std::ifstream file(file_name.c_str(), std::ios::in);
    if(file.is_open() == false)
    {
        std::cerr << "Not open file: " << file_name << std::endl;
        exit(1);
    }
    std::istream *f_in = nullptr;
    f_in = new istream(file.rdbuf());
    if(f_in == nullptr)
    {
        std::cerr << file_name << " is empty..." << std::endl;
        exit(2);
    }
    std::string line;
    std::string invalid_word;
    std::istringstream iss_line;
    while(f_in->good())
    {
        iss_line.clear();
        getline(*f_in, line);
        
        if(line.length() <= 0 || line[0] == '#')
            continue;
        
        iss_line.str(line);

        std::string key;
		RobotPose amcl;
        iss_line >> key;
		iss_line >> amcl.timestamp;
		iss_line >> amcl.x;
		iss_line >> amcl.y;
		iss_line >> amcl.yaw;
		m_v_amcl1[key].push_back(amcl);
    }
    file.close();
}
// --------------------------------------------------------------------------
double getDistance(double e_x,double e_y,double e_theta,double tag_dx,double tag_dy,double tag_x, double tag_y, double tag_theta, 
		double anchor_dx,double anchor_dy, double anchor_x, double anchor_y, double anchor_theta)
// --------------------------------------------------------------------------
{

	//estimation*odom
	Eigen::Vector3d tag_odom_trans(tag_x,tag_y,0);
	tf2::Quaternion tag_odom_quaternion;
	tag_odom_quaternion.setRPY( 0, 0, tag_theta);
	Eigen::Quaterniond tag_odom_q(tag_odom_quaternion.w(),tag_odom_quaternion.x(),tag_odom_quaternion.y(),tag_odom_quaternion.z());//w,x,y,z
	g2o::SE3Quat tag_odom_se3(tag_odom_q,tag_odom_trans);	


	Eigen::Vector3d estimation_trans(e_x,e_y,0);
	tf2::Quaternion estimation_quaternion;
	estimation_quaternion.setRPY( 0, 0, e_theta);
	Eigen::Quaterniond estimation_q(estimation_quaternion.w(),estimation_quaternion.x(),estimation_quaternion.y(),estimation_quaternion.z());//w,x,y,z
	g2o::SE3Quat estimation_se3(estimation_q,estimation_trans);


	g2o::SE3Quat relative_estmation_odom=estimation_se3*tag_odom_se3;//relative transformtion T=A*B,

	Eigen::Quaterniond qd_relative_estimation_odom= relative_estmation_odom.rotation();
	Eigen::Vector3d trans_relative_estimation_odom=relative_estmation_odom.translation();		// x y	
	auto euler_estimation_odom = qd_relative_estimation_odom.toRotationMatrix().eulerAngles(0, 1, 2);
	double relative_yaw = euler_estimation_odom[2]; //angle 
	//we compute the relative odom measurements


	double x_predicted_tag=trans_relative_estimation_odom.x()+tag_dx*cos(relative_yaw)-tag_dy*sin(relative_yaw);
	double y_predicted_tag=trans_relative_estimation_odom.y()+tag_dx*sin(relative_yaw)+tag_dy*cos(relative_yaw);

	//assume the x, y, and theta are zero
	double x_predicted_anchor= anchor_x + anchor_dx*cos(anchor_theta)-anchor_dy*sin(anchor_theta);
	double y_predicted_anchor= anchor_y + anchor_dx*sin(anchor_theta)+anchor_dy*cos(anchor_theta);
	
	double fval = sqrt((x_predicted_tag-x_predicted_anchor)*(x_predicted_tag-x_predicted_anchor)+(y_predicted_tag-y_predicted_anchor)*(y_predicted_tag-y_predicted_anchor));
	return fval;

}
// kalman filters
boost::circular_buffer<double> v_win_fp_rx(10);
class KalmanFilter{
public:
	KalmanFilter(){
		Xbar= 0;
		Pbar = 0.5;
		Q=0.5;
		R=0.1;
		flag=1;
	}
	double Kalman_filters(double z, bool NLOS_flag, double d_pre);
private:
	double X_bar;
    double Xbar;
	double P_bar;
	double Pbar;
    double K;
	double Q;
	double R;
    double B; 
	double old_d_pre;
	int flag;
};
inline double KalmanFilter::Kalman_filters(double z, bool NLOS_flag, double d_pre)                                                                                                                                                                                                             
{  
    if(flag)
    {
        Xbar = d_pre;
        flag = 0;
    }
    else
    {
        // predict
        X_bar = Xbar + (d_pre - old_d_pre);
        P_bar = Pbar + Q;
        // update
        if(NLOS_flag)
            K = 0;
        else
            K = P_bar / (P_bar + R);
		// cout << Q << "\t" << R << "\t" << Xbar << "\t" << X_bar <<  "\t" << Pbar << "\t" << K << "\t" << z << "\t" << d_pre << "\t" << old_d_pre << "\n------" << endl;
        Xbar = X_bar + K*(z-X_bar);
	
        Pbar = P_bar - K*P_bar;
		
    }
    old_d_pre = d_pre;
    return Xbar;
}

KalmanFilter *kf12 = new KalmanFilter();
KalmanFilter *kf21 = new KalmanFilter();
KalmanFilter *kf13 = new KalmanFilter();
KalmanFilter *kf23 = new KalmanFilter();
KalmanFilter *kf31 = new KalmanFilter();
KalmanFilter *kf32 = new KalmanFilter();

RobotPose g2o_process(std::vector<G2OMeasure> g2o_measures, int numPoints, int maxIterations, RobotPose init_value)
{
	Vector12d* points = new Vector12d[numPoints];
	for (int j = 0; j < numPoints; ++j)
	{
		points[j](0)=g2o_measures[j].tag_delta_x;
		points[j](1)=g2o_measures[j].tag_delta_y;
		points[j](2)=g2o_measures[j].tag_T_x;
		points[j](3)=g2o_measures[j].tag_T_y;
		points[j](4)=g2o_measures[j].tag_T_yaw;

		points[j](5)=g2o_measures[j].anchor_delta_x;
		points[j](6)=g2o_measures[j].anchor_delta_y;
		points[j](7)=g2o_measures[j].anchor_T_x;
		points[j](8)=g2o_measures[j].anchor_T_y;
		points[j](9)=g2o_measures[j].anchor_T_yaw;			
		points[j](10)=g2o_measures[j].distance;
        points[j](11)=g2o_measures[j].weight;
	}
    
	// some handy typedefs
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
	typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

	// setup the solver
	g2o::SparseOptimizer optimizer;
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
			g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

	optimizer.setAlgorithm(solver);

	// build the optimization problem given the points
	// 1. add the parameter vertex
	VertexParams* params = new VertexParams();
	params->setId(0);
	params->setEstimate(Eigen::Vector3d(init_value.x,init_value.y,init_value.yaw)); // some initial value for the params
	optimizer.addVertex(params);

	// 2. add the points we measured to be on the curve
	for (int i = 0; i < numPoints; ++i) {
		EdgePointOnCurve* e = new EdgePointOnCurve;
		e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
		g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber();	
		rk->setDelta(0.2);
		e->setRobustKernel(rk);
		e->setVertex(0, params);
		e->setMeasurement(points[i]);
		optimizer.addEdge(e);
	}

	// perform the optimization
	optimizer.initializeOptimization();
	optimizer.setVerbose(false);
	optimizer.optimize(maxIterations);	

    RobotPose estimation;
	estimation.x = params->estimate()(0);
	estimation.y = params->estimate()(1);
	estimation.yaw = params->estimate()(2);
    return estimation;
}

std::vector<double> v_pose_error;
std::vector<double> v_yaw_error;
std::vector<double> v_max_error_time;
int valid_count=0;
std::vector<double> v_odom_pose_error;
std::vector<double> v_odom_yaw_error;

std::map<std::string, std::vector<RobotPose> > m_v_rel_uwb; 
RobotPose nonlinearOptimization_process(int total_num, std::string anchor_name, std::string tag_name, std::vector<CommonPose> v_com_odom, boost::circular_buffer<UWBMessage> v_robot_uwb_measure)
{  
	valid_count++;
	std::vector<G2OMeasure> v_g2o_measure;
	CommonPose now_odom = v_com_odom[total_num-1];
	ros::Time begin = ros::Time::now();
	for(int i=total_num-1; i>total_num-TIME_WINDOW_SIZE; i=i-TIME_GRAP)
	{
		CommonPose lag_odom = v_com_odom[i];
		G2OMeasure g2o_measure;
		// check node 
		RobotPose tag_odom = now_odom.tagPose;
		RobotPose lag_tag_odom = lag_odom.tagPose;  
		RobotPose anchor_odom = now_odom.anchorPose;
		RobotPose lag_anchor_odom = lag_odom.anchorPose;

		// 1) compute odom relative transform matrix
		// 1.1)) tag odom
		RobotPose relative_tag_pose = computeT1inverse_x_T2(tag_odom, lag_tag_odom);
		// 1.2)) anchor odom
		RobotPose relative_anchor_pose = computeT1inverse_x_T2(anchor_odom, lag_anchor_odom);
		// 1.3)) save data
		g2o_measure.tag_T_x = relative_tag_pose.x;
		g2o_measure.tag_T_y = relative_tag_pose.y;
		g2o_measure.tag_T_yaw = carmen_normalize_theta(relative_tag_pose.yaw);
		g2o_measure.anchor_T_x = relative_anchor_pose.x;
		g2o_measure.anchor_T_y = relative_anchor_pose.y;
		g2o_measure.anchor_T_yaw = carmen_normalize_theta(relative_anchor_pose.yaw);
		g2o_measure.weight = 1;
		// 2) save uwb delta pose and distance
		g2o_measure.anchor_delta_x = dx;
		g2o_measure.anchor_delta_y = dy;
		g2o_measure.tag_delta_x = dx;
		g2o_measure.tag_delta_y = dy;
		g2o_measure.distance = v_robot_uwb_measure[i].range;

		v_g2o_measure.push_back(g2o_measure);

		RobotPose anchor_ = lag_anchor_odom;
		RobotPose tag_ = lag_tag_odom;
		m_v_win[tag_name].push_back(anchor_);
		m_v_win[anchor_name].push_back(tag_);
	}
	ros::Time end = ros::Time::now();
	// plotWindow();
	m_v_win[tag_name].clear();
	m_v_win[anchor_name].clear();
	// 3) localization based g2o  
	CommonPose odom_pose =  v_com_odom[total_num-1];  
	RobotPose relative_o_se3 = computeT1inverse_x_T2(odom_pose.anchorPose, odom_pose.tagPose);
	RobotPose init;
	RobotPose relative_pose;
	std::map<std::string, int>::iterator it_flag = m_flag1.find(anchor_name+tag_name);
	if(it_flag == m_flag1.end())
		m_flag1[anchor_name+tag_name] = 0;

	double dist1 = v_robot_uwb_measure[total_num - 1].range;
	if(m_flag1[anchor_name+tag_name] == 0)
	{
		m_flag1[anchor_name+tag_name] = 1;
	    init = relative_o_se3;
	}
	else
	{	
		RobotPose old_rel_se3 = m_old_rel[anchor_name+tag_name];
		RobotPose T2 = computeT1inverse_x_T2(m_old_odom_pose[anchor_name+tag_name].anchorPose, odom_pose.anchorPose);
		RobotPose T1 = computeT1inverse_x_T2(m_old_odom_pose[anchor_name+tag_name].tagPose, odom_pose.tagPose);
		RobotPose old_rel_se3_T1 = computeT1_x_T2(old_rel_se3, T1);
		RobotPose now_pose = computeT1inverse_x_T2(T2, old_rel_se3_T1);
		init = now_pose;
		// getchar();	
	}
	ros::Time begin_g2o = ros::Time::now();
	relative_pose = g2o_process(v_g2o_measure, v_g2o_measure.size(), 20, init);
	ros::Time end_g2o = ros::Time::now();
	relative_pose.timestamp = odom_pose.timeStamp;
	m_v_rel_uwb[anchor_name+tag_name].push_back(relative_pose);
	m_old_rel[anchor_name+tag_name] = relative_pose;
	m_est_rel[anchor_name+tag_name] = relative_pose;
	m_old_odom[anchor_name+tag_name] = relative_o_se3;
	m_old_odom_pose[anchor_name+tag_name] = odom_pose;
	// 4) compute position error
	if(anchor_name == "robot1")
	{
		RobotPose glo_ = computeT1_x_T2(odom_pose.anchorPose, relative_pose);
		m_v_uwb_rel[tag_name].push_back(glo_);
	}
	// cout << "rel_opt_time: " <<  fabs(begin_g2o.toSec()-end_g2o.toSec()) << endl;
	return relative_pose;
}
//------------------------------------------------------------------------

//========================================================================
std::map<std::string, UWBConfigure> m_uwb_configure;
void loadConfigureFile(std::string file_name)
{
	std::ifstream file(file_name.c_str(), std::ios::in);
    if(file.is_open() == false)
    {
        std::cerr << "Not open file: " << file_name << std::endl;
        exit(1);
    }
    std::istream *f_in = nullptr;
    f_in = new istream(file.rdbuf());
    if(f_in == nullptr)
    {
        std::cerr << file_name << " is empty..." << std::endl;
        exit(2);
    }
    std::string line;
    std::string invalid_word;
    std::istringstream iss_line;
    while(f_in->good())
    {
        iss_line.clear();
        getline(*f_in, line);
        
        if(line.length() <= 0 || line[0] == '#')
            continue;
        
        iss_line.str(line);

        std::string key;
		UWBConfigure uwb;
        iss_line >> key;
		if(key.find("robot") == 0)
		{
			iss_line >> uwb.uwb_id;
			iss_line >> uwb.uwb_pose.x;
			iss_line >> uwb.uwb_pose.y;
			dx = uwb.uwb_pose.x;
			dy = uwb.uwb_pose.y;
			iss_line >> uwb.uwb_pose.yaw;
			m_uwb_configure[key] = uwb;
		}
    }
    file.close();
}
//========================================================================


std::vector<double> v_uwb_error;
// const double time_limit = 0.1;

// bool timeSynchronization(std::string anchor_name, std::string tag_name, CommonPose &now_odom)
// {
// 	std::map<std::string, IndexNum>::iterator it_index = m_index_record.find(anchor_name+tag_name);
// 	if(it_index == m_index_record.end())
// 		m_index_record[anchor_name+tag_name].index1 = 0;
// 	double timestamp = m_v_odom[anchor_name].back().timestamp;
// 	double time_min_diff = INFINITY;
// 	int record_i=-1;
// 	for(int i=m_index_record[anchor_name+tag_name].index1; i<m_v_odom[tag_name].size(); i++)
// 	{
// 		double time_diff = fabs(timestamp - m_v_odom[tag_name][i].timestamp);
// 		if(time_diff < time_min_diff)
// 		{
// 			time_min_diff = time_diff;
// 			record_i  = i;
// 		}
// 	}
// 	if(time_min_diff > time_limit||record_i < 0)
// 		return false;
// 	now_odom.anchorPose = m_v_odom[anchor_name].back();
// 	now_odom.tagPose = m_v_odom[tag_name][record_i];
// 	now_odom.timeStamp = timestamp;
// 	m_index_record[anchor_name+tag_name].index1 = record_i;
// 	return true;
// }
std::map<std::string, CommonPose> m_old_odom_of_uwb;
double uwbKalmanFilter(UWBMessage uwb_message, CommonPose now_odom, RobotPose est_rel, KalmanFilter *kf, std::string name)
{
	double rx_fp = fabs(uwb_message.rxRssi - uwb_message.fpRssi); 
	double fp = fabs(uwb_message.fpRssi + 80.0);
	bool NLOS_flag;
	
	// Current
	v_win_fp_rx.push_back(rx_fp);
	double sum=0;
	for(int i=0; i<v_win_fp_rx.size(); i++)
	{
		sum += v_win_fp_rx[i];
	}  
	double mean_rx_fp = sum/(double)(v_win_fp_rx.size());
	sum=0;
	for(int i=0; i<v_win_fp_rx.size(); i++)
	{
		sum += (v_win_fp_rx[i]-mean_rx_fp)* (v_win_fp_rx[i]-mean_rx_fp);
	}
	double std_rx_fp =   sqrt(sum/(double)(v_win_fp_rx.size()));
	// 1)
	uwb_message.range = 0.122288 + 1.0013*uwb_message.range;
	if(fp >= 5 || std_rx_fp>2)
	{
		NLOS_flag = true;   
	}
	else
	{ 
		NLOS_flag = false;
	}
	// static CommonPose old_odom;
	RobotPose rel_;
	double dist_pre;
	if(est_rel.timestamp == 666)
	{
		rel_ = computeT1inverse_x_T2(now_odom.anchorPose, now_odom.tagPose);
		dist_pre = sqrt((rel_.x * rel_.x) + (rel_.y * rel_.y));
	}
	else if(est_rel.timestamp == 777)
	{   
		RobotPose T2 = computeT1inverse_x_T2(m_old_odom_of_uwb[name].anchorPose, now_odom.anchorPose);
		RobotPose T1 = computeT1inverse_x_T2(m_old_odom_of_uwb[name].tagPose, now_odom.tagPose);
		RobotPose rel_se3_T1 = computeT1_x_T2(est_rel, T1);
		rel_ = computeT1inverse_x_T2(T2, rel_se3_T1);
		dist_pre = sqrt((rel_.x * rel_.x) + (rel_.y * rel_.y));
	}
	m_old_odom_of_uwb[name] = now_odom;
	return kf->Kalman_filters(uwb_message.range, NLOS_flag, dist_pre);			
}
void saveComData(CommonPose odom, CommonPose amcl, UWBMessage uwb, double true_dist)
{
	std::ofstream com_odom(com_odom_file, std::ios::app);
	std::ofstream com_amcl(com_amcl_file, std::ios::app);
	std::ofstream com_uwb(com_uwb_measure_file, std::ios::app);
	std::ofstream com_dist(com_trues_dist_file, std::ios::app);

	com_odom << to_string(odom.timeStamp) << "\t" << to_string(odom.tagPose.x) << "\t" << to_string(odom.tagPose.y) << "\t" << to_string(odom.tagPose.yaw) << "\t"
	         << to_string(odom.anchorPose.x) << "\t" << to_string(odom.anchorPose.y) << "\t" << to_string(odom.anchorPose.yaw) << "\n";
	com_odom.close();

	com_amcl << to_string(amcl.timeStamp) << "\t" << to_string(amcl.tagPose.x) << "\t" << to_string(amcl.tagPose.y) << "\t" << to_string(amcl.tagPose.yaw) << "\t"
	         << to_string(amcl.anchorPose.x) << "\t" << to_string(amcl.anchorPose.y) << "\t" << to_string(amcl.anchorPose.yaw) << "\n";
	com_amcl.close();

	com_uwb << to_string(uwb.timestamp) << "\t" << to_string(uwb.range) << "\t" << to_string(uwb.rxRssi) << "\t" << to_string(uwb.fpRssi) << "\n";
	com_uwb.close();

	com_dist << to_string(uwb.timestamp) << "\t" << to_string(true_dist) << "\n";
	com_dist.close();
}
void saveG2oData(double time, RobotPose est_res, CommonPose odom, std::string anchor_name,  std::string tag_name)
{
	if(anchor_name == "robot1")
	{
		RobotPose tag_pose = computeT1_x_T2(odom.anchorPose, est_res);
		std::string name_tag;
		if(tag_name == "robot2")
		{
			name_tag = "drone2";
			m_v_robot_pose["drone1"].push_back(odom.anchorPose);
			m_v_odom_pose["drone2"].push_back(odom.tagPose);
		}
		else if(tag_name == "robot3")
		{
			name_tag = "drone3";
			m_v_odom_pose["drone3"].push_back(odom.tagPose);
		}
#ifdef SAVE_G2O_DATA
		std::ofstream rel_pose_file(drone_rel_pose_file, std::ios::app);
		rel_pose_file << name_tag << "\t" << to_string(time) << "\t" << to_string(tag_pose.x) << "\t" << to_string(tag_pose.y) << "\t" << to_string(tag_pose.yaw) << "\n";
		rel_pose_file.close();
#endif
		tag_pose.timestamp = time;
		m_v_robot_pose[name_tag].push_back(tag_pose);
	}
	if(anchor_name == "robot1" && tag_name == "robot2")
	{
#ifdef SAVE_G2O_DATA
		std::ofstream est_file(drone_est12_file, std::ios::app);
		est_file << "drone1" << "\t" << "drone2" << "\t" << to_string(time) << "\t" << to_string(est_res.x) << "\t" << to_string(est_res.y) << "\t" << to_string(est_res.yaw) << "\n";
		est_file.close();
#endif
		est_res.timestamp = time;
		m_v_rel_pose["drone1"].g2o_estimate["drone2"].push_back(est_res);
	}
	if(anchor_name == "robot1" && tag_name == "robot3")
	{
#ifdef SAVE_G2O_DATA
		std::ofstream est_file(drone_est13_file, std::ios::app);
		est_file << "drone1" << "\t" << "drone3" << "\t" << to_string(time) << "\t" << to_string(est_res.x) << "\t" << to_string(est_res.y) << "\t" << to_string(est_res.yaw) << "\n";
		est_file.close();
#endif
		est_res.timestamp = time;
		m_v_rel_pose["drone1"].g2o_estimate["drone3"].push_back(est_res);
	}
	if(anchor_name == "robot2" && tag_name == "robot3")
	{
#ifdef SAVE_G2O_DATA
		std::ofstream est_file(drone_est23_file, std::ios::app);
		est_file << "drone2" << "\t" << "drone3" << "\t" << to_string(time) << "\t" << to_string(est_res.x) << "\t" << to_string(est_res.y) << "\t" << to_string(est_res.yaw) << "\n";
		est_file.close();
#endif
		est_res.timestamp = time;
		m_v_rel_pose["drone2"].g2o_estimate["drone3"].push_back(est_res);
	}

	// inverse
	// RobotPose est_inver_pose = computeG2oSE3ToPose(computePoseToG2oSE3(est_res).inverse());
	// if(tag_name == "robot1")
	// {
	// 	RobotPose tag_pose = computeT1_x_T2(odom.tagPose, est_inver_pose);
	// 	std::string name_tag;
	// 	if(anchor_name == "robot2")
	// 		name_tag = "drone2";
	// 	else if(anchor_name == "robot3")
	// 		name_tag = "drone3";
	// 	std::ofstream rel_pose_file(drone_rel_pose_file, std::ios::app);
	// 	rel_pose_file << name_tag << "\t" << to_string(time) << "\t" << to_string(tag_pose.x) << "\t" << to_string(tag_pose.y) << "\t" << to_string(tag_pose.yaw) << "\n";
	// 	rel_pose_file.close();
	// }
	// if(anchor_name == "robot2" && tag_name == "robot1")
	// {
	// 	std::ofstream est_file(drone_est12_file, std::ios::app);
	// 	est_file << "drone1" << "\t" << "drone2" << "\t" << to_string(time) << "\t" << to_string(est_inver_pose.x) << "\t" << to_string(est_inver_pose.y) << "\t" << to_string(est_inver_pose.yaw) << "\n";
	// 	est_file.close();
	// }
	// if(anchor_name == "robot3" && tag_name == "robot1")
	// {
	// 	std::ofstream est_file(drone_est13_file, std::ios::app);
	// 	est_file << "drone1" << "\t" << "drone3" << "\t" << to_string(time) << "\t" << to_string(est_inver_pose.x) << "\t" << to_string(est_inver_pose.y) << "\t" << to_string(est_inver_pose.yaw) << "\n";
	// 	est_file.close();
	// }
	// if(anchor_name == "robot3" && tag_name == "robot2")
	// {
	// 	std::ofstream est_file(drone_est23_file, std::ios::app);
	// 	est_file << "drone2" << "\t" << "drone3" << "\t" << to_string(time) << "\t" << to_string(est_inver_pose.x) << "\t" << to_string(est_inver_pose.y) << "\t" << to_string(est_inver_pose.yaw) << "\n";
	// 	est_file.close();
	// }
}
void prepareOptimationData(std::string anchor_name, std::string tag_name, double time, std::map<int, UWBMessage> m_node,  
						   KalmanFilter *kf,
						   boost::circular_buffer<UWBMessage> &v_robot_uwb_measure)
{
	std::map<int, UWBMessage>::iterator it_node;
	for(it_node=m_node.begin(); it_node!=m_node.end(); it_node++)
	{
		if(it_node->first == m_uwb_configure[tag_name].uwb_id)
		{
			UWBMessage uwb;
			uwb.range = it_node->second.range;
			uwb.fpRssi = it_node->second.fpRssi;
			uwb.rxRssi = it_node->second.rxRssi;
			m_v_win_uwb[anchor_name+tag_name].push_back(uwb);
		}
	}
	if(m_v_odom.find(tag_name) == m_v_odom.end())
		return;
	if((m_v_odom[anchor_name].back().timestamp - time) > UPDATE_TIME) return;
	if((m_v_odom[tag_name].back().timestamp - time) > UPDATE_TIME) return;
	ros::Time begin = ros::Time::now();
	if(m_v_win_uwb.find(anchor_name+tag_name) != m_v_win_uwb.end())
	{	
		CommonPose now_odom;
		now_odom.timeStamp = time;
		now_odom.anchorPose = m_v_odom[anchor_name].back();
		now_odom.tagPose = m_v_odom[tag_name].back();
		double delta_anchor_dist = now_odom.anchorPose.accumulated_dist - m_old_odom_anchor_acc_dist[anchor_name+tag_name];
		double delta_tag_dist = now_odom.tagPose.accumulated_dist - m_old_odom_tag_acc_dist[anchor_name+tag_name];
		double delta_anchor_angle = now_odom.anchorPose.accumulated_orientation - m_old_odom_anchor_acc_angle[anchor_name+tag_name];
		double delta_tag_angle = now_odom.tagPose.accumulated_orientation - m_old_odom_tag_acc_angle[anchor_name+tag_name];
		if((delta_anchor_dist < UPDATE_DIST) && (delta_anchor_angle < UPDATE_ANGLE) && (delta_tag_dist < UPDATE_DIST) && (delta_tag_angle < UPDATE_ANGLE))
			return;
		m_old_odom_anchor_acc_dist[anchor_name+tag_name] = now_odom.anchorPose.accumulated_dist;
		m_old_odom_anchor_acc_angle[anchor_name+tag_name] = now_odom.anchorPose.accumulated_orientation;
		m_old_odom_tag_acc_dist[anchor_name+tag_name] = now_odom.tagPose.accumulated_dist;
		m_old_odom_tag_acc_angle[anchor_name+tag_name] = now_odom.tagPose.accumulated_orientation;

		int cnt = m_v_win_uwb[anchor_name+tag_name].size();
		if(cnt <= 0)
			return;	
		UWBMessage sum;
		for(int i=0; i<cnt; i++)
		{
			sum.range += m_v_win_uwb[anchor_name+tag_name][i].range;
			sum.fpRssi += m_v_win_uwb[anchor_name+tag_name][i].fpRssi;
			sum.rxRssi += m_v_win_uwb[anchor_name+tag_name][i].rxRssi;
		}
		m_v_win_uwb[anchor_name+tag_name].clear();
		UWBMessage uwb_message;
		uwb_message.timestamp = m_v_odom[anchor_name].back().timestamp;
		uwb_message.range = sum.range/(double)(cnt);
		uwb_message.fpRssi = sum.fpRssi/(double)(cnt);
		uwb_message.rxRssi = sum.rxRssi/(double)(cnt);
		
		RobotPose est_rel;
		std::map<std::string, RobotPose>::iterator it_est = m_est_rel.find(anchor_name+tag_name);
		if(it_est == m_est_rel.end())
			est_rel.timestamp = 666;
		else
		{
			est_rel =  m_est_rel[anchor_name+tag_name];
			est_rel.timestamp = 777;
		}
		uwb_message.range =  uwbKalmanFilter(uwb_message, now_odom, est_rel, kf, anchor_name+tag_name);
		
		m_v_com_odom[anchor_name+tag_name].push_back(now_odom);
		v_robot_uwb_measure.push_back(uwb_message);
		
		// cout << anchor_name+tag_name  << ":\t" << m_v_com_odom[anchor_name+tag_name].size() << "\t" << v_robot_uwb_measure.size() << "\t" << endl;
		int total_num = m_v_com_odom[anchor_name+tag_name].size();
		int total_num1 = m_v_com_odom["robot1robot2"].size();
		int total_num2 = m_v_com_odom["robot1robot3"].size();
		int total_num3 = m_v_com_odom["robot2robot3"].size();
		RobotPose relative_pose;
		
		if(total_num1-TIME_WINDOW_SIZE > TIME_GRAP && total_num2-TIME_WINDOW_SIZE > TIME_GRAP && total_num3-TIME_WINDOW_SIZE > TIME_GRAP  && TIME_WINDOW_SIZE <= FINAL_TIME_WINDOW_SIZE)
		{
			TIME_WINDOW_SIZE +=  TIME_GRAP;
			TIME_GRAP = TIME_WINDOW_SIZE/NUM_POINTS;
		}
		ros::Time begin_g2o;
		ros::Time end ;
		if(total_num > TIME_WINDOW_SIZE)
		{
			begin_g2o = ros::Time::now();
			relative_pose = nonlinearOptimization_process(total_num, anchor_name, tag_name, m_v_com_odom[anchor_name+tag_name], v_robot_uwb_measure);
			end = ros::Time::now();
			saveG2oData(time, relative_pose, now_odom, anchor_name, tag_name);
		}
		
		double time_cast = end.toSec() - begin.toSec();
		double g2o_time_cast = end.toSec() - begin_g2o.toSec();
		// cout << "time_cast: " << time_cast << "\t" << g2o_time_cast << endl; 
	}	
}
//========================================================================
void robot1UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	if(m_v_odom.find("robot1") == m_v_odom.end()) return;
	double time = fabs(msg->sec + (double)(msg->usec)/1000000.0 - firstTimestamp);
	int uwb_id = msg->id;
	if(uwb_id != m_uwb_configure["robot1"].uwb_id) 
		return;
	std::map<int, UWBMessage> m_node;
	for(int i=0; i<msg->node.size(); i++)
	{
		int key = msg->node[i].id;
		m_node[key].range = msg->node[i].dis;
		m_node[key].fpRssi = msg->node[i].fpRssi;
		m_node[key].rxRssi = msg->node[i].rxRssi;
	}

	
	prepareOptimationData("robot1", "robot2", time, m_node, kf12, v_robot12_uwb_measure);
	prepareOptimationData("robot1", "robot3", time, m_node, kf13, v_robot13_uwb_measure);

}
//------------------------------------------------------------------------
void robot2UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	if(m_v_odom.find("robot2") == m_v_odom.end()) return;
	double time = fabs(msg->sec + (double)(msg->usec)/1000000.0 - firstTimestamp);
	int uwb_id = msg->id;
	if(uwb_id != m_uwb_configure["robot2"].uwb_id) 
		return;
	std::map<int, UWBMessage> m_node;
	for(int i=0; i<msg->node.size(); i++)
	{
		int key = msg->node[i].id;
		m_node[key].range = msg->node[i].dis;
		m_node[key].fpRssi = msg->node[i].fpRssi;
		m_node[key].rxRssi = msg->node[i].rxRssi;
	}

	// prepareOptimationData("robot2", "robot1", time, m_node, kf21, v_robot21_uwb_measure);
	prepareOptimationData("robot2", "robot3", time, m_node, kf23, v_robot23_uwb_measure);
}
//------------------------------------------------------------------------
void robot3UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	if(m_v_odom.find("robot3") == m_v_odom.end()) return;
	double time = fabs(msg->sec + (double)(msg->usec)/1000000.0 - firstTimestamp);
	int uwb_id = msg->id;
	if(uwb_id != m_uwb_configure["robot3"].uwb_id) 
		return;
	std::map<int, UWBMessage> m_node;
	for(int i=0; i<msg->node.size(); i++)
	{
		int key = msg->node[i].id;
		m_node[key].range = msg->node[i].dis;
		m_node[key].fpRssi = msg->node[i].fpRssi;
		m_node[key].rxRssi = msg->node[i].rxRssi;
	}

	prepareOptimationData("robot3", "robot1", time, m_node, kf31, v_robot31_uwb_measure);
	prepareOptimationData("robot3", "robot2", time, m_node, kf32, v_robot32_uwb_measure);
}
//------------------------------------------------------------------------

void robot1OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
#ifdef INITIAL_POSE_IS_AMCL
	if(m_v_amcl.find("robot1") == m_v_amcl.end()) return;
#endif
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	RobotPose est_pose;
	static int once_flag0 = 0;
	pose.timestamp = timestamp;
	if(once_flag0 == 0)
	{
		once_flag0 = 1;
#ifdef INITIAL_POSE_IS_AMCL
		robot1_current_odom.x = m_v_amcl["robot1"][0].x;
		robot1_current_odom.y = m_v_amcl["robot1"][0].y;
		robot1_current_odom.yaw =  m_v_amcl["robot1"][0].yaw;
		est_current_odom1.x = m_v_amcl["robot1"][0].x;
		est_current_odom1.y = m_v_amcl["robot1"][0].y;
		est_current_odom1.yaw =  m_v_amcl["robot1"][0].yaw;
#else
		robot1_current_odom.x = INITIAL1_X;
		robot1_current_odom.y = INITIAL1_Y;
		robot1_current_odom.yaw =  INITIAL1_YAW;
		est_current_odom1.x = INITIAL1_X;
		est_current_odom1.y = INITIAL1_Y;
		est_current_odom1.yaw =  INITIAL1_YAW;
#endif
		pose.x=robot1_current_odom.x;
		pose.y=robot1_current_odom.y;
		pose.yaw=robot1_current_odom.yaw; 
		est_pose.x=robot1_current_odom.x;
		est_pose.y=robot1_current_odom.y;
		est_pose.yaw=robot1_current_odom.yaw; 
	}

	if(robot1_acc_angle<0)
	{

		robot1_acc_distance=0;
		robot1_acc_angle=0;

		robot1_previous_odom.x=msg->pose.pose.position.x;
		robot1_previous_odom.y=msg->pose.pose.position.y;
		robot1_previous_odom.yaw=yaw;
		pose.accumulated_dist=robot1_acc_distance;
		pose.accumulated_orientation=robot1_acc_angle;
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
		double sigma_rot1;
		if(sigma_trans < 0.01)//the robot may static and not moving
		{
			sigma_rot1 = 0.0;
		}	    	
		else
		{
			sigma_rot1=atan2(current_y-previous_y,current_x-previous_x)-previous_theta;	
			sigma_rot1=carmen_normalize_theta(sigma_rot1);
		}

		double sigma_rot2=current_theta-previous_theta-sigma_rot1;
		sigma_rot2=carmen_normalize_theta(sigma_rot2);


		// We want to treat backward and forward motion symmetrically for the
		// noise model to be applied below.  The standard model seems to assume
		// forward motion.
		double delta_rot1_noise = std::min(fabs(angle_diff(sigma_rot1,0.0)),
				fabs(angle_diff(sigma_rot1,M_PI)));
		double delta_rot2_noise = std::min(fabs(angle_diff(sigma_rot2,0.0)),
				fabs(angle_diff(sigma_rot2,M_PI)));


		Eigen::Matrix3d jacobian_g;
		Eigen::Matrix3d jacobian_v;
		Eigen::Matrix3d noise_q;

		double M02_g=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M12_g=sigma_trans*cos(previous_theta+sigma_rot1);


		double M00_v=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M01_v=cos(previous_theta+sigma_rot1);
		double M10_v=sigma_trans*cos(previous_theta+sigma_rot1);
		double M11_v=sin(previous_theta+sigma_rot1);

		double M00_noise=alpha1*delta_rot1_noise*delta_rot1_noise+alpha2*sigma_trans*sigma_trans;
		double M11_noise=alpha3*sigma_trans*sigma_trans+alpha4*(delta_rot1_noise*delta_rot1_noise+delta_rot2_noise*delta_rot2_noise);
		double M22_noise=alpha1*delta_rot2_noise*delta_rot2_noise+alpha2*sigma_trans*sigma_trans;


		//fill in the jacobian matrix
		jacobian_g<<1, 0, M02_g, 0, 1, M12_g, 0,0,1;

		jacobian_v<<M00_v, M01_v, 0, M10_v, M11_v, 0, 1,0,1;

		noise_q<<M00_noise, 0, 0, 0, M11_noise, 0, 0, 0, M22_noise;

		//update the covariance matrix, C=G*C*G^T+V*Q*V^T
		covariance1=jacobian_g*covariance1*jacobian_g.transpose()+jacobian_v*noise_q*jacobian_v.transpose();			
		// std::ofstream file2(test_file2_name, std::ios::app);
		// file2 << "\t" << to_string(covariance2.determinant()) << endl;
		// file2.close();
		pose.uwbCovariance = covariance1.determinant();
		//===================

		double moving_theta=fabs(carmen_normalize_theta(robot1_previous_odom.yaw-yaw));

		robot1_acc_distance=robot1_acc_distance+sigma_trans;		
		robot1_acc_angle=robot1_acc_angle+moving_theta;

		pose.accumulated_dist=robot1_acc_distance;
		pose.accumulated_orientation=robot1_acc_angle;

		robot1_previous_odom.x=msg->pose.pose.position.x;
		robot1_previous_odom.y=msg->pose.pose.position.y;
		robot1_previous_odom.yaw=yaw;

		robot1_current_odom.x = robot1_current_odom.x + sigma_trans*cos(robot1_current_odom.yaw);
		robot1_current_odom.y = robot1_current_odom.y + sigma_trans*sin(robot1_current_odom.yaw);
		robot1_current_odom.yaw = robot1_current_odom.yaw + actual_delta_theta;
		robot1_current_odom.yaw = carmen_normalize_theta(robot1_current_odom.yaw);
		
		est_current_odom1.x = est_current_odom1.x + sigma_trans*cos(est_current_odom1.yaw);
		est_current_odom1.y = est_current_odom1.y + sigma_trans*sin(est_current_odom1.yaw);
		est_current_odom1.yaw = est_current_odom1.yaw + actual_delta_theta;
		est_current_odom1.yaw = carmen_normalize_theta(est_current_odom1.yaw);
		est_pose.x = est_current_odom1.x;
		est_pose.y = est_current_odom1.y;
		est_pose.yaw = est_current_odom1.yaw;
		est_pose.accumulated_dist=robot1_acc_distance;
		pose.timestamp = timestamp;
		pose.x = robot1_current_odom.x;
		pose.y = robot1_current_odom.y;
		pose.yaw = robot1_current_odom.yaw;
	}
	est_pose.timestamp = timestamp;
	m_v_odom["robot1"].push_back(pose);
	// if(g2o_flag)
	m_v_est_globoal_pose["robot1"].push_back(est_pose);
	nav_msgs::Odometry est_pose_;
	est_pose_.header.stamp = msg->header.stamp;
	est_pose_.pose.pose.position.x = est_pose.x;
	est_pose_.pose.pose.position.y = est_pose.y;
	est_pose_.pose.pose.position.z = est_pose.yaw;
	est_pose_.pose.pose.orientation.x = est_pose.accumulated_dist;
	robot1_pub.publish(est_pose_);
	// plotODOM();
}
//------------------------------------------------------------------------
void robot2OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
#ifdef INITIAL_POSE_IS_AMCL
	if(m_v_amcl.find("robot2") == m_v_amcl.end()) return;
#endif
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	RobotPose est_pose;
	pose.timestamp = timestamp;

	static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
#ifdef INITIAL_POSE_IS_AMCL
		robot2_current_odom.x = m_v_amcl["robot2"][0].x;
		robot2_current_odom.y = m_v_amcl["robot2"][0].y;
		robot2_current_odom.yaw =  m_v_amcl["robot2"][0].yaw;
		est_current_odom2.x = m_v_amcl["robot2"][0].x;
		est_current_odom2.y = m_v_amcl["robot2"][0].y;
		est_current_odom2.yaw =  m_v_amcl["robot2"][0].yaw;
#else
		robot2_current_odom.x = INITIAL2_X;
		robot2_current_odom.y = INITIAL2_Y;
		robot2_current_odom.yaw =  INITIAL2_YAW;
		est_current_odom2.x = INITIAL2_X;
		est_current_odom2.y = INITIAL2_Y;
		est_current_odom2.yaw =  INITIAL2_YAW;
#endif
		pose.x=robot2_current_odom.x;
		pose.y=robot2_current_odom.y;
		pose.yaw=robot2_current_odom.yaw; 
		est_pose.x=robot2_current_odom.x;
		est_pose.y=robot2_current_odom.y;
		est_pose.yaw=robot2_current_odom.yaw; 
	}

	if(robot2_acc_angle<0)
	{

		robot2_acc_distance=0;
		robot2_acc_angle=0;

		robot2_previous_odom.x=msg->pose.pose.position.x;
		robot2_previous_odom.y=msg->pose.pose.position.y;
		robot2_previous_odom.yaw=yaw;
		pose.accumulated_dist=robot2_acc_distance;
		pose.accumulated_orientation=robot2_acc_angle;
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
		double sigma_rot1;
		if(sigma_trans < 0.01)//the robot may static and not moving
		{
			sigma_rot1 = 0.0;
		}	    	
		else
		{
			sigma_rot1=atan2(current_y-previous_y,current_x-previous_x)-previous_theta;	
			sigma_rot1=carmen_normalize_theta(sigma_rot1);
		}

		double sigma_rot2=current_theta-previous_theta-sigma_rot1;
		sigma_rot2=carmen_normalize_theta(sigma_rot2);


		// We want to treat backward and forward motion symmetrically for the
		// noise model to be applied below.  The standard model seems to assume
		// forward motion.
		double delta_rot1_noise = std::min(fabs(angle_diff(sigma_rot1,0.0)),
				fabs(angle_diff(sigma_rot1,M_PI)));
		double delta_rot2_noise = std::min(fabs(angle_diff(sigma_rot2,0.0)),
				fabs(angle_diff(sigma_rot2,M_PI)));


		Eigen::Matrix3d jacobian_g;
		Eigen::Matrix3d jacobian_v;
		Eigen::Matrix3d noise_q;

		double M02_g=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M12_g=sigma_trans*cos(previous_theta+sigma_rot1);


		double M00_v=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M01_v=cos(previous_theta+sigma_rot1);
		double M10_v=sigma_trans*cos(previous_theta+sigma_rot1);
		double M11_v=sin(previous_theta+sigma_rot1);

		double M00_noise=alpha1*delta_rot1_noise*delta_rot1_noise+alpha2*sigma_trans*sigma_trans;
		double M11_noise=alpha3*sigma_trans*sigma_trans+alpha4*(delta_rot1_noise*delta_rot1_noise+delta_rot2_noise*delta_rot2_noise);
		double M22_noise=alpha1*delta_rot2_noise*delta_rot2_noise+alpha2*sigma_trans*sigma_trans;


		//fill in the jacobian matrix
		jacobian_g<<1, 0, M02_g, 0, 1, M12_g, 0,0,1;

		jacobian_v<<M00_v, M01_v, 0, M10_v, M11_v, 0, 1,0,1;

		noise_q<<M00_noise, 0, 0, 0, M11_noise, 0, 0, 0, M22_noise;

		//update the covariance matrix, C=G*C*G^T+V*Q*V^T
		covariance2=jacobian_g*covariance2*jacobian_g.transpose()+jacobian_v*noise_q*jacobian_v.transpose();			
		pose.uwbCovariance = covariance2.determinant();
		//===================

		double moving_theta=fabs(carmen_normalize_theta(robot2_previous_odom.yaw-yaw));

		robot2_acc_distance=robot2_acc_distance+sigma_trans;		
		robot2_acc_angle=robot2_acc_angle+moving_theta;

		pose.accumulated_dist=robot2_acc_distance;
		pose.accumulated_orientation=robot2_acc_angle;

		robot2_previous_odom.x=msg->pose.pose.position.x;
		robot2_previous_odom.y=msg->pose.pose.position.y;
		robot2_previous_odom.yaw=yaw;

		robot2_current_odom.x = robot2_current_odom.x + sigma_trans*cos(robot2_current_odom.yaw);
		robot2_current_odom.y = robot2_current_odom.y + sigma_trans*sin(robot2_current_odom.yaw);
		robot2_current_odom.yaw = robot2_current_odom.yaw + actual_delta_theta;
		robot2_current_odom.yaw = carmen_normalize_theta(robot2_current_odom.yaw);
		
		est_current_odom2.x = est_current_odom2.x + sigma_trans*cos(est_current_odom2.yaw);
		est_current_odom2.y = est_current_odom2.y + sigma_trans*sin(est_current_odom2.yaw);
		est_current_odom2.yaw = est_current_odom2.yaw + actual_delta_theta;
		est_current_odom2.yaw = carmen_normalize_theta(est_current_odom2.yaw);
		
		
		est_pose.x = est_current_odom2.x;
		est_pose.y = est_current_odom2.y;
		est_pose.yaw = est_current_odom2.yaw;
		est_pose.accumulated_dist=robot2_acc_distance;

		pose.timestamp = timestamp;
		pose.x = robot2_current_odom.x;
		pose.y = robot2_current_odom.y;
		pose.yaw = robot2_current_odom.yaw;
	}
	est_pose.timestamp = timestamp;
	m_v_odom["robot2"].push_back(pose);
	// if(g2o_flag)
	m_v_est_globoal_pose["robot2"].push_back(est_pose);
	nav_msgs::Odometry est_pose_;
	est_pose_.header.stamp = msg->header.stamp;
	est_pose_.pose.pose.position.x = est_pose.x;
	est_pose_.pose.pose.position.y = est_pose.y;
	est_pose_.pose.pose.position.z= est_pose.yaw;
	est_pose_.pose.pose.orientation.x = est_pose.accumulated_dist;
	robot2_pub.publish(est_pose_);
}
//------------------------------------------------------------------------
void robot3OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
#ifdef INITIAL_POSE_IS_AMCL
	if(m_v_amcl["robot3"].size() == 0) return;
#endif
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	RobotPose est_pose;
	pose.timestamp = timestamp;

	static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
#ifdef INITIAL_POSE_IS_AMCL
		robot3_current_odom.x = m_v_amcl["robot3"][0].x;
		robot3_current_odom.y = m_v_amcl["robot3"][0].y;
		robot3_current_odom.yaw =  m_v_amcl["robot3"][0].yaw;
		est_current_odom3.x = m_v_amcl["robot3"][0].x;
		est_current_odom3.y = m_v_amcl["robot3"][0].y;
		est_current_odom3.yaw =  m_v_amcl["robot3"][0].yaw;
#else
		robot3_current_odom.x = INITIAL3_X;
		robot3_current_odom.y = INITIAL3_Y;
		robot3_current_odom.yaw =  INITIAL3_YAW;
		est_current_odom3.x = INITIAL3_X;
		est_current_odom3.y = INITIAL3_Y;
		est_current_odom3.yaw =  INITIAL3_YAW;
#endif
		pose.x=robot3_current_odom.x;
		pose.y=robot3_current_odom.y;
		pose.yaw=robot3_current_odom.yaw; 
		est_pose.x=robot3_current_odom.x;
		est_pose.y=robot3_current_odom.y;
		est_pose.yaw=robot3_current_odom.yaw; 
	}

	if(robot3_acc_angle<0)
	{

		robot3_acc_distance=0;
		robot3_acc_angle=0;

		robot3_previous_odom.x=msg->pose.pose.position.x;
		robot3_previous_odom.y=msg->pose.pose.position.y;
		robot3_previous_odom.yaw=yaw;
		pose.accumulated_dist=robot3_acc_distance;
		pose.accumulated_orientation=robot3_acc_angle;
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
		double sigma_rot1;
		if(sigma_trans < 0.01)//the robot may static and not moving
		{
			sigma_rot1 = 0.0;
		}	    	
		else
		{
			sigma_rot1=atan2(current_y-previous_y,current_x-previous_x)-previous_theta;	
			sigma_rot1=carmen_normalize_theta(sigma_rot1);
		}

		double sigma_rot2=current_theta-previous_theta-sigma_rot1;
		sigma_rot2=carmen_normalize_theta(sigma_rot2);


		// We want to treat backward and forward motion symmetrically for the
		// noise model to be applied below.  The standard model seems to assume
		// forward motion.
		double delta_rot1_noise = std::min(fabs(angle_diff(sigma_rot1,0.0)),
				fabs(angle_diff(sigma_rot1,M_PI)));
		double delta_rot2_noise = std::min(fabs(angle_diff(sigma_rot2,0.0)),
				fabs(angle_diff(sigma_rot2,M_PI)));


		Eigen::Matrix3d jacobian_g;
		Eigen::Matrix3d jacobian_v;
		Eigen::Matrix3d noise_q;

		double M02_g=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M12_g=sigma_trans*cos(previous_theta+sigma_rot1);


		double M00_v=-sigma_trans*sin(previous_theta+sigma_rot1);
		double M01_v=cos(previous_theta+sigma_rot1);
		double M10_v=sigma_trans*cos(previous_theta+sigma_rot1);
		double M11_v=sin(previous_theta+sigma_rot1);

		double M00_noise=alpha1*delta_rot1_noise*delta_rot1_noise+alpha2*sigma_trans*sigma_trans;
		double M11_noise=alpha3*sigma_trans*sigma_trans+alpha4*(delta_rot1_noise*delta_rot1_noise+delta_rot2_noise*delta_rot2_noise);
		double M22_noise=alpha1*delta_rot2_noise*delta_rot2_noise+alpha2*sigma_trans*sigma_trans;


		//fill in the jacobian matrix
		jacobian_g<<1, 0, M02_g, 0, 1, M12_g, 0,0,1;

		jacobian_v<<M00_v, M01_v, 0, M10_v, M11_v, 0, 1,0,1;

		noise_q<<M00_noise, 0, 0, 0, M11_noise, 0, 0, 0, M22_noise;

		//update the covariance matrix, C=G*C*G^T+V*Q*V^T
		covariance3=jacobian_g*covariance3*jacobian_g.transpose()+jacobian_v*noise_q*jacobian_v.transpose();			
		pose.uwbCovariance = covariance3.determinant();
		//===================

		double moving_theta=fabs(carmen_normalize_theta(robot3_previous_odom.yaw-yaw));

		robot3_acc_distance=robot3_acc_distance+sigma_trans;		
		robot3_acc_angle=robot3_acc_angle+moving_theta;

		pose.accumulated_dist=robot3_acc_distance;
		pose.accumulated_orientation=robot3_acc_angle;

		robot3_previous_odom.x=msg->pose.pose.position.x;
		robot3_previous_odom.y=msg->pose.pose.position.y;
		robot3_previous_odom.yaw=yaw;

		robot3_current_odom.x = robot3_current_odom.x + sigma_trans*cos(robot3_current_odom.yaw);
		robot3_current_odom.y = robot3_current_odom.y + sigma_trans*sin(robot3_current_odom.yaw);
		robot3_current_odom.yaw = robot3_current_odom.yaw + actual_delta_theta;
		robot3_current_odom.yaw = carmen_normalize_theta(robot3_current_odom.yaw);

		est_current_odom3.x = est_current_odom3.x + sigma_trans*cos(est_current_odom3.yaw);
		est_current_odom3.y = est_current_odom3.y + sigma_trans*sin(est_current_odom3.yaw);
		est_current_odom3.yaw = est_current_odom3.yaw + actual_delta_theta;
		est_current_odom3.yaw = carmen_normalize_theta(est_current_odom3.yaw);
		est_pose.x = est_current_odom3.x;
		est_pose.y = est_current_odom3.y;
		est_pose.yaw = est_current_odom3.yaw;	
		est_pose.accumulated_dist=robot3_acc_distance;
		pose.timestamp = timestamp;
		pose.x = robot3_current_odom.x;
		pose.y = robot3_current_odom.y;
		pose.yaw = robot3_current_odom.yaw;
	}
	est_pose.timestamp = timestamp;
	m_v_odom["robot3"].push_back(pose);
	// if(g2o_flag)
	m_v_est_globoal_pose["robot3"].push_back(est_pose);
	nav_msgs::Odometry est_pose_;
	est_pose_.header.stamp = msg->header.stamp;
	est_pose_.pose.pose.position.x = est_pose.x;
	est_pose_.pose.pose.position.y = est_pose.y;
	est_pose_.pose.pose.position.z= est_pose.yaw;
	est_pose_.pose.pose.orientation.x = est_pose.accumulated_dist;
	robot3_pub.publish(est_pose_);
}
//------------------------------------------------------------------------

void robot1AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	double timestamp = fabs(msg->header.stamp.toSec() - firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);	
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose robot_gt;
    robot_gt.x=msg->pose.pose.position.x;
	robot_gt.y=msg->pose.pose.position.y;	
	robot_gt.yaw=yaw;
    robot_gt.timestamp = timestamp;
	m_v_amcl["robot1"].push_back(robot_gt);
	m_v_amcl_drone["drone1"].push_back(robot_gt);
	// plotAMCL();
#ifdef SIMULATION_ENABLE
	est_current_odom1 = robot_gt;
#endif
#ifdef SAVE_G2O_DATA
	std::ofstream amcl_file(drone_AMCL_file1, std::ios::app);
	amcl_file << "drone1" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	amcl_file.close();
#endif
}
//------------------------------------------------------------------------
void robot2AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	double timestamp = fabs(msg->header.stamp.toSec() - firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);	
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose robot_gt;
    robot_gt.x=msg->pose.pose.position.x;
	robot_gt.y=msg->pose.pose.position.y;	
	robot_gt.yaw=yaw;
    robot_gt.timestamp = timestamp;
	m_v_amcl["robot2"].push_back(robot_gt);
	m_v_amcl_drone["drone2"].push_back(robot_gt);
#ifdef SIMULATION_ENABLE
	est_current_odom2 = robot_gt;
#endif
#ifdef SAVE_G2O_DATA
	std::ofstream amcl_file(drone_AMCL_file2, std::ios::app);
	amcl_file << "drone2" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	amcl_file.close();
#endif
}
//------------------------------------------------------------------------
void robot3AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	double timestamp = fabs(msg->header.stamp.toSec() - firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);	
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose robot_gt;
    robot_gt.x=msg->pose.pose.position.x;
	robot_gt.y=msg->pose.pose.position.y;	
	robot_gt.yaw=yaw;
    robot_gt.timestamp = timestamp;
	m_v_amcl["robot3"].push_back(robot_gt);
	m_v_amcl_drone["drone3"].push_back(robot_gt);
#ifdef SIMULATION_ENABLE
	est_current_odom3 = robot_gt;
#endif
#ifdef SAVE_G2O_DATA
	std::ofstream amcl_file(drone_AMCL_file3, std::ios::app);
	amcl_file << "drone3" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	amcl_file.close();
#endif
}
//========================================================================

// --------------------------------------------------------------------------
void generate_time_based_random_seed()
// --------------------------------------------------------------------------
{
	unsigned int seed;
	struct timeval tv;

	if ( gettimeofday(&tv, NULL) < 0 )
		fprintf( stderr, "error in gettimeofday : %s\n", strerror( errno) );
	seed = tv.tv_sec + tv.tv_usec;
	srand( seed );
}

void* computePositionError(void* args)
{
	while (1)
	{	
		plotUWBRel();
		plotEstGlo();
		
		plot_all_odom_tracks();
		sleep(10);
		// continue;
		// compute uwb error relative m_v_amcl m_v_rel_uwb
		std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
		it_amcl = m_v_amcl.find("robot1");
		if(it_amcl == m_v_amcl.end() || m_v_amcl.find("robot2") == m_v_amcl.end() || m_v_rel_uwb.find("robot1robot2") == m_v_rel_uwb.end())
			continue;
		std::vector<RobotPose> v_robot1_amcl = it_amcl->second;
		std::vector<double> v_robot12_pose_error;
		std::vector<double> v_robot12_yaw_error;	
		for(int i=0; i<v_robot1_amcl.size(); i++)
		{
			double timestamp=v_robot1_amcl[i].timestamp;
			std::vector<RobotPose> v_robot2_amcl = m_v_amcl["robot2"];
			int record_j = -1;
			double min_time = INFINITY;
			for(int j=0; j<v_robot2_amcl.size(); j++)
			{
				double diff = fabs(timestamp - v_robot2_amcl[j].timestamp);
				if(diff < min_time)
				{
					min_time = diff;
					record_j = j;
				}
			}
			if(record_j < 0 || min_time > time_limit)
				continue;
			CommonPose amcl;
			amcl.anchorPose = v_robot1_amcl[i];
			amcl.tagPose = v_robot2_amcl[record_j];

			RobotPose rel_amcl_se3 = computeT1inverse_x_T2(amcl.anchorPose, amcl.tagPose);
			
			
			record_j = -1;
			min_time = INFINITY;
			// robot12
			{
				std::vector<RobotPose> v_rel_uwb = m_v_rel_uwb["robot1robot2"];
				for(int j=0; j<v_rel_uwb.size(); j++)
				{
					double diff = fabs(timestamp - v_rel_uwb[j].timestamp);
					if(diff < min_time)
					{
						min_time = diff;
						record_j = j;
					}
				}
				if(record_j < 0 || min_time > time_limit)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];

				RobotPose error = computeT1inverse_x_T2(rel_amcl_se3, relative_pose);

				// cout << rel_amcl1.yaw << endl;
				double pose_error = sqrt(error.x*error.x + error.y*error.y);
				double yaw_error = fabs(error.yaw);
				
				v_robot12_pose_error.push_back(pose_error);
				v_robot12_yaw_error.push_back(yaw_error);
			}
			// robot21
			record_j = -1;
			min_time = INFINITY;
			{
				std::vector<RobotPose> v_rel_uwb = m_v_rel_uwb["robot2robot1"];	
				for(int j=0; j<v_rel_uwb.size(); j++)
				{
					double diff = fabs(timestamp - v_rel_uwb[j].timestamp);
					if(diff < min_time)
					{
						min_time = diff;
						record_j = j;
					}
				}
				if(record_j < 0 || min_time > time_limit)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];

				RobotPose error = computeT1_x_T2(rel_amcl_se3, relative_pose);

				// cout << rel_amcl1.yaw << endl;
				double pose_error = sqrt(error.x*error.x + error.y*error.y);
				double yaw_error = fabs(error.yaw);

				v_robot12_pose_error.push_back(pose_error);
				v_robot12_yaw_error.push_back(yaw_error);
			}
			// robot13
			if(m_v_amcl.find("robot3") == m_v_amcl.end() || m_v_rel_uwb.find("robot1robot3") == m_v_rel_uwb.end())
				continue;
			std::vector<RobotPose> v_robot3_amcl = m_v_amcl["robot3"];
			record_j = -1;
			min_time = INFINITY;
			for(int j=0; j<v_robot3_amcl.size(); j++)
			{
				double diff = fabs(timestamp - v_robot3_amcl[j].timestamp);
				if(diff < min_time)
				{
					min_time = diff;
					record_j = j;
				}
			}
			if(record_j < 0 || min_time > time_limit)
				continue;
			amcl.tagPose = v_robot3_amcl[record_j];
			rel_amcl_se3 = computeT1inverse_x_T2(amcl.anchorPose, amcl.tagPose);


			record_j = -1;
			min_time = INFINITY;
			{
				std::vector<RobotPose> v_rel_uwb = m_v_rel_uwb["robot1robot3"];	
				for(int j=0; j<v_rel_uwb.size(); j++)
				{
					double diff = fabs(timestamp - v_rel_uwb[j].timestamp);
					if(diff < min_time)
					{
						min_time = diff;
						record_j = j;
					}
				}
				if(record_j < 0 || min_time > time_limit)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];
				
				RobotPose error = computeT1_x_T2(rel_amcl_se3, relative_pose);

				// cout << rel_amcl1.yaw << endl;
				double pose_error = sqrt(error.x*error.x + error.y*error.y);
				double yaw_error = fabs(error.yaw);

				v_robot12_pose_error.push_back(pose_error);
				v_robot12_yaw_error.push_back(yaw_error);
			}
		}
		
		it_amcl = m_v_amcl.find("robot2");
		std::vector<RobotPose> v_robot2_amcl = it_amcl->second;
		for(int i=0; i<v_robot2_amcl.size(); i++)
		{
			double timestamp=v_robot2_amcl[i].timestamp;
			if(m_v_amcl.find("robot3") == m_v_amcl.end()) continue;
			std::vector<RobotPose> v_robot3_amcl = m_v_amcl["robot3"];
			int record_j = -1;
			double min_time = INFINITY;
			for(int j=0; j<v_robot3_amcl.size(); j++)
			{
				double diff = fabs(timestamp - v_robot3_amcl[j].timestamp);
				if(diff < min_time)
				{
					min_time = diff;
					record_j = j;
				}
			}
			if(record_j < 0 || min_time > time_limit)
				continue;
			CommonPose amcl;
			amcl.anchorPose = v_robot2_amcl[i];
			amcl.tagPose = v_robot3_amcl[record_j];

			RobotPose rel_amcl_se3 = computeT1inverse_x_T2(amcl.anchorPose, amcl.tagPose);
			
			
			record_j = -1;
			min_time = INFINITY;
			// robot12
			{
				if(m_v_rel_uwb.find("robot2robot3") == m_v_rel_uwb.end()) continue;
				std::vector<RobotPose> v_rel_uwb = m_v_rel_uwb["robot2robot3"];
				
				for(int j=0; j<v_rel_uwb.size(); j++)
				{
					double diff = fabs(timestamp - v_rel_uwb[j].timestamp);
					if(diff < min_time)
					{
						min_time = diff;
						record_j = j;
					}
				}
				if(record_j < 0 || min_time > time_limit)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];

				RobotPose error = computeT1inverse_x_T2(rel_amcl_se3, relative_pose);

				// cout << rel_amcl1.yaw << endl;
				double pose_error = sqrt(error.x*error.x + error.y*error.y);
				double yaw_error = fabs(error.yaw);
				
				v_robot12_pose_error.push_back(pose_error);
				v_robot12_yaw_error.push_back(yaw_error);
			}
		}
		

		double mean_pose_error=0;
		double mean_pose_std=0;
		double mean_yaw_error=0;
		double mean_yaw_std=0;
		computeMeanVariance(v_robot12_pose_error, mean_pose_error, mean_pose_std);
		computeMeanVariance(v_robot12_yaw_error, mean_yaw_error, mean_yaw_std);
		
		std::cout << "robot12" << "-poseError:" << mean_pose_error << "\t" << mean_pose_std << "\n"
			            << "       -yawError:" << mean_yaw_error*180/M_PI << "\t" << mean_yaw_std*180/M_PI << std::endl;
	}
}

void computeG2oRelativeError(std::map<std::string, std::map<double, RobotPose> > m_m_data, std::string anchor_name, std::string tag_name, std::vector<double> &v_pose_error, std::vector<double> &v_yaw_error)
{
	std::map<double, RobotPose> data = m_m_data[anchor_name]; // find  g2o results of anchor robot 
	cout << data.size() << endl;
	for(int i=0; i<data.size(); i++) // anchor robot vexter size
	{
// g2o
		std::map <double, RobotPose>::iterator it_data=data.find(double(i)); // the key of map that robot1 results after g2o is 0 to 10000, robot2  and robot3 is not.

		RobotPose drone_pose=it_data->second;

		double drone1_time=drone_pose.timestamp;
		double drone1_x=drone_pose.x;
		double drone1_y=drone_pose.y;
		double drone1_yaw=drone_pose.yaw;
		// cout << to_string(drone1_time) << "  " <<  drone1_x << "   " << drone1_y << "  " << drone1_yaw << endl;
		std::map<double, RobotPose>  m_drone2_pose = m_m_data[tag_name] ; 
		std::map<double, RobotPose>::iterator it_drone2;
		double timestamp_temp=INF;
		int recard_i;
		for(it_drone2 = m_drone2_pose.begin(); it_drone2 != m_drone2_pose.end(); it_drone2++)
		{
			RobotPose drone2_pose = it_drone2->second;
			double drone2_time = drone2_pose.timestamp;
			double time_diff = fabs(drone2_time - drone1_time);
			if(time_diff < timestamp_temp)
			{
				timestamp_temp = time_diff;
				recard_i = it_drone2->first;
			}
		}
		if(timestamp_temp > time_limit) 
			continue;
static int old_recard_i = 0;
if(recard_i == old_recard_i)
	continue;
old_recard_i = recard_i;
		RobotPose drone2_current_pose = m_drone2_pose[recard_i];
		// cout << to_string(drone2_current_pose.timestamp) << "  " <<  drone2_current_pose.x << "   " << drone2_current_pose.y << "  " << drone2_current_pose.yaw << endl;
		Eigen::Vector3d drone1_trans(drone1_x,drone1_y,0);
		Eigen::Quaterniond drone1_q;
		Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(AngleAxisd(drone1_yaw,Vector3d::UnitZ()));
		drone1_q=rollAngle*pitchAngle*yawAngle;
		g2o::SE3Quat drone1_se3(drone1_q,drone1_trans);

		Eigen::Vector3d drone2_trans(drone2_current_pose.x, drone2_current_pose.y,0);
		Eigen::Quaterniond drone2_q;
		Eigen::AngleAxisd rollAngle1(AngleAxisd(0,Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle1(AngleAxisd(0,Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle1(AngleAxisd(drone2_current_pose.yaw,Vector3d::UnitZ()));
		drone2_q = rollAngle1*pitchAngle1*yawAngle1;
		g2o::SE3Quat drone2_se3(drone2_q,drone2_trans);

		g2o::SE3Quat drone1_2_rel_se3 = drone1_se3.inverse() * drone2_se3;
		
		Eigen::Quaterniond qd_est_g = drone1_2_rel_se3.rotation();
		Eigen::Vector3d t_est_g = drone1_2_rel_se3.translation();
		auto euler_est_g = qd_est_g.toRotationMatrix().eulerAngles(0, 1, 2);

		// cout << t_est_g.x() << "   " << t_est_g.y() << "   " << euler_est_g[2] << endl;
		double est_g_x = t_est_g.x();
		double est_g_y = t_est_g.y();
		double est_g_yaw = carmen_normalize_theta(euler_est_g[2]);
// amcl
		std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
		it_amcl = m_v_amcl_drone.find(anchor_name);
		std::vector<RobotPose> drone1_amcl = it_amcl->second;
		timestamp_temp=INF;
		for(int i=0; i<drone1_amcl.size(); i++)
		{
			double time_diff = fabs(drone1_amcl[i].timestamp - drone1_time);
			if(time_diff < timestamp_temp)
			{
				timestamp_temp = time_diff;
				recard_i = i;
			}
		}
		if(timestamp_temp > time_limit) 
			continue;
		RobotPose drone1_current_amcl = drone1_amcl[recard_i];
	// cout << to_string(drone1_current_amcl.timestamp) << "  " <<  drone1_current_amcl.x << "   " << drone1_current_amcl.y << "  " << drone1_current_amcl.yaw << endl;
		it_amcl = m_v_amcl_drone.find(tag_name);
		std::vector<RobotPose> drone2_amcl = it_amcl->second;
		timestamp_temp=INF;
		for(int i=0; i<drone2_amcl.size(); i++)
		{
			double time_diff = fabs(drone2_amcl[i].timestamp - drone1_time);
			if(time_diff < timestamp_temp)
			{
				timestamp_temp = time_diff;
				recard_i = i;
			}
		}
		if(timestamp_temp > time_limit) 
		continue;
		RobotPose drone2_current_amcl = drone2_amcl[recard_i];
	// cout << to_string(drone2_current_amcl.timestamp)  << "  " <<  drone2_current_amcl.x << "   " << drone2_current_amcl.y << "  " << drone2_current_amcl.yaw  << endl;
		Eigen::Vector3d amcl1_trans(drone1_current_amcl.x,drone1_current_amcl.y,0);
		Eigen::Quaterniond amcl1_q;
		Eigen::AngleAxisd rollAngle2(AngleAxisd(0,Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle2(AngleAxisd(0,Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle2(AngleAxisd(drone1_current_amcl.yaw,Vector3d::UnitZ()));
		amcl1_q=rollAngle2*pitchAngle2*yawAngle2;
		g2o::SE3Quat amcl1_se3(amcl1_q,amcl1_trans);

		Eigen::Vector3d amcl2_trans(drone2_current_amcl.x, drone2_current_amcl.y,0);
		Eigen::Quaterniond amcl2_q;
		Eigen::AngleAxisd rollAngle3(AngleAxisd(0,Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle3(AngleAxisd(0,Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle3(AngleAxisd(drone2_current_amcl.yaw,Vector3d::UnitZ()));
		amcl2_q = rollAngle3*pitchAngle3*yawAngle3;
		g2o::SE3Quat amcl2_se3(amcl2_q,amcl2_trans);

		g2o::SE3Quat amcl1_2_rel_se3 = amcl1_se3.inverse() * amcl2_se3;
		Eigen::Quaterniond qd_est_a = amcl1_2_rel_se3.rotation();
		Eigen::Vector3d t_est_a = amcl1_2_rel_se3.translation();
		auto euler_est_a = qd_est_a.toRotationMatrix().eulerAngles(0, 1, 2);
		// cout << t_est_a.x() << "   " << t_est_a.y() << "   " << euler_est_a[2] << endl;
		double est_a_x = t_est_a.x();
		double est_a_y = t_est_a.y();
		double est_a_yaw = carmen_normalize_theta(euler_est_a[2]);
// compute Error
		g2o::SE3Quat relative_error_se3 = amcl1_2_rel_se3.inverse() * drone1_2_rel_se3;

		Eigen::Quaterniond qd_est = relative_error_se3.rotation();
		Eigen::Vector3d t_est = relative_error_se3.translation();
		auto euler_est = qd_est.toRotationMatrix().eulerAngles(0, 1, 2);
		
		double yaw_error = carmen_normalize_theta(euler_est[2]);
		yaw_error = fabs(yaw_error);
		double pose_error = sqrt((t_est.x()*t_est.x()) +( t_est.y()*t_est.y()));
		double pose_error1 =sqrt( (drone1_x-drone2_current_pose.x) *(drone1_x-drone2_current_pose.x)
														+ (drone1_y-drone2_current_pose.y)*(drone1_y-drone2_current_pose.y));
		double pose_error2 =sqrt( (drone1_current_amcl.x-drone2_current_amcl.x) *(drone1_current_amcl.x-drone2_current_amcl.x)
														+ (drone1_current_amcl.y-drone2_current_amcl.y)*(drone1_current_amcl.y-drone2_current_amcl.y));
		double pose_e =fabs (pose_error1 - pose_error2);
		double pose_ee = sqrt((est_g_x - est_a_x)*(est_g_x - est_a_x)
								+ (est_g_y-est_a_y)*(est_g_y-est_a_y));
		double yaw_ee = fabs(est_a_yaw - est_g_yaw);
		// cout << pose_error <<   "  " <<  pose_e << "  " <<  yaw_error << "   " <<  << "\n---" << endl;
		//	 cout << pose_error <<   "  " <<  pose_e << "  " <<  yaw_error << "   " << pose_ee << "   " << yaw_ee <<"\n---" << endl;
		v_pose_error.push_back(pose_error);
		v_yaw_error.push_back(yaw_error);
		static double cnt = 0;
		double amcl_dist = sqrt((drone2_current_amcl.x-drone1_current_amcl.x)*(drone2_current_amcl.x-drone1_current_amcl.x) + (drone2_current_amcl.y-drone1_current_amcl.y)*(drone2_current_amcl.y-drone1_current_amcl.y));
		double g2o_dist = sqrt((drone1_x-drone2_current_pose.x)*(drone1_x-drone2_current_pose.x) + (drone1_y-drone2_current_pose.y)*(drone1_y-drone2_current_pose.y));
		double dist_error = fabs(amcl_dist-g2o_dist);
		std::ofstream file1("..//output//error1.txt", std::ios::app);
		file1 << "\t" << to_string(drone_pose.timestamp) << "\t" << to_string(pose_error) << "\t" << to_string(yaw_error) << "\n";
		file1.close();
		std::ofstream file("..//output//error.txt", std::ios::app);
		file << "\t" << to_string(drone_pose.timestamp-1604387234.482793) << "\t" << to_string(pose_error) << "\t" << to_string(yaw_error) << "\n";
		file.close();
		cnt = cnt + 0.7;
	}
}

typedef g2o::BlockSolver< BlockSolverX >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
void* g2o_process(void* args)
{
	while(1)
	{
		// cout << m_v_odom.size() << "\t" << m_v_robot_pose.size() <<  "\t" << m_v_rel_pose.size() <<endl;
		if(m_v_odom_pose.size() < 1 || m_v_robot_pose.size() < 3 || m_v_rel_pose.size() < 1)
			continue;
		
		// g2o
		double begin_g2o = 0;
		SparseOptimizer optimizer;
		//allocating the optimizer
		SlamLinearSolver* linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
				g2o::make_unique<SlamBlockSolver>(
						g2o::make_unique<LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>>()));
		optimizer.setAlgorithm(solver);

		// add the parameter representing the sensor offset
		ParameterSE3Offset* sensorOffset = new ParameterSE3Offset;
		Eigen::Isometry3d sensorOffsetTransf=Eigen::Isometry3d::Identity();
		sensorOffset->setOffset(sensorOffsetTransf);
		sensorOffset->setId(0);
		optimizer.addParameter(sensorOffset);

		int robot1_index=0;
		int robot2_index=10000;
		int robot3_index=20000;
		int vexter_num12=0;
		int vexter_num3=0;
		std::vector<RobotPose> drone_position_vecter;
		std::vector<RobotPose> drone1_vexter_vecter;
		// add drone2 vexter
		std::map< std::string, std::vector<RobotPose> >::iterator it_m_drone_position=m_v_robot_pose.find("drone2");
		if(it_m_drone_position != m_v_robot_pose.end())
		{
			drone_position_vecter=it_m_drone_position->second;
			for(int i=0; i<drone_position_vecter.size(); i++)
			{
				RobotPose drone_pose=drone_position_vecter[i];

				double drone_timestamp=drone_pose.timestamp;
				double se3_x=drone_pose.x;
				double se3_y=drone_pose.y;
				double se3_theta=drone_pose.yaw;

				Eigen::Quaterniond pose_q;
				Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
				Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
				Eigen::AngleAxisd yawAngle(AngleAxisd(se3_theta,Vector3d::UnitZ()));

				pose_q=yawAngle*pitchAngle*rollAngle;

				Eigen::Vector3d trans(se3_x,se3_y,0);
				g2o::SE3Quat current(pose_q,trans);

				VertexSE3* robot_2 =  new VertexSE3;

				robot_2->setId(robot2_index);
				robot_2->setEstimate(current);
				optimizer.addVertex(robot_2);
				robot2_index++;
				vexter_num12++;
			}
		}

		it_m_drone_position=m_v_robot_pose.find("drone1");
		if(it_m_drone_position != m_v_robot_pose.end())
		{
		    drone1_vexter_vecter=it_m_drone_position->second;
			for(int i=0; i<drone1_vexter_vecter.size(); i++)
			{
				RobotPose drone_pose=drone1_vexter_vecter[i];

				double drone_timestamp=drone_pose.timestamp;
				double se3_x=drone_pose.x;
				double se3_y=drone_pose.y;
				double se3_theta=drone_pose.yaw;

				Eigen::Quaterniond pose_q;
				Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
				Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
				Eigen::AngleAxisd yawAngle(AngleAxisd(se3_theta,Vector3d::UnitZ()));

				pose_q=yawAngle*pitchAngle*rollAngle;

				Eigen::Vector3d trans(se3_x,se3_y,0);
				g2o::SE3Quat current(pose_q,trans);

				VertexSE3* robot_1 =  new VertexSE3;

				robot_1->setId(robot1_index);
				robot_1->setEstimate(current);
				optimizer.addVertex(robot_1);
				robot1_index++;
			}
		}
		// add drone3 vexter
		it_m_drone_position=m_v_robot_pose.find("drone3");
		if(it_m_drone_position != m_v_robot_pose.end())
		{
			drone_position_vecter=it_m_drone_position->second;
			for(int i=0; i<drone_position_vecter.size(); i++)
			{
				RobotPose drone_pose=drone_position_vecter[i];

				double drone_timestamp=drone_pose.timestamp;
				double se3_x=drone_pose.x;
				double se3_y=drone_pose.y;
				double se3_theta=drone_pose.yaw;

				Eigen::Quaterniond pose_q;
				Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
				Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
				Eigen::AngleAxisd yawAngle(AngleAxisd(se3_theta,Vector3d::UnitZ()));

				pose_q=yawAngle*pitchAngle*rollAngle;

				Eigen::Vector3d trans(se3_x,se3_y,0);
				g2o::SE3Quat current(pose_q,trans);

				VertexSE3* robot_3 =  new VertexSE3;

				robot_3->setId(robot3_index);
				robot_3->setEstimate(current);
				optimizer.addVertex(robot_3);
				robot3_index++;
				vexter_num3++;
			}
		}

		// add drone1 odom egde
		double drone1_x_pre;
		double drone1_y_pre;
		double drone1_yaw_pre;
		for(int i=0 ; i<drone1_vexter_vecter.size(); i++)
		{
			RobotPose drone1_pose=drone1_vexter_vecter[i];

			double x=drone1_pose.x;
			double y=drone1_pose.y;
			double yaw=drone1_pose.yaw;

			Eigen::Quaterniond pose_q;
			Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
			Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
			Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));

			pose_q=yawAngle*pitchAngle*rollAngle;

			int pre_i=i-1;
			if(pre_i >= 0)
			{
				EdgeSE3* odometry = new EdgeSE3;

				odometry->vertices()[0] = optimizer.vertex(robot1_index+pre_i - vexter_num12);
				odometry->vertices()[1] = optimizer.vertex(robot1_index+i - vexter_num12);

				double pre_x=drone1_x_pre;
				double pre_y=drone1_y_pre;
				double pre_yaw=drone1_yaw_pre;

				Eigen::Quaterniond pre_pose_q;
				Eigen::AngleAxisd pre_rollAngle(AngleAxisd(0,Vector3d::UnitX()));
				Eigen::AngleAxisd pre_pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
				Eigen::AngleAxisd pre_yawAngle(AngleAxisd(pre_yaw,Vector3d::UnitZ()));

				pre_pose_q=pre_yawAngle*pre_pitchAngle*pre_rollAngle;

				Eigen::Vector3d previous_trans(pre_x,pre_y,0);
				Eigen::Quaterniond previous_q=pre_pose_q;//w,x,y,z

				Eigen::Vector3d trans(x,y,0);
				Eigen::Quaterniond q=pose_q;//w,x,y,z

				g2o::SE3Quat previous(previous_q,previous_trans);
				g2o::SE3Quat egde_current(q,trans);

				g2o::SE3Quat relative_odom=previous.inverse()*egde_current;//relative transformtion T=inverse(A)*B,
				Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
				Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
				Eigen::Vector3d t_relative_odom=relative_odom.translation();
				Eigen::Matrix3f matrix_relative_odom=q_relative_odom.toRotationMatrix();
				Eigen::Affine3f affine_relative_odom(matrix_relative_odom);

				Eigen::Quaterniond temp_q;
				temp_q.x()=q_relative_odom.x();
				temp_q.y()=q_relative_odom.y();
				temp_q.z()=q_relative_odom.z();
				temp_q.w()=q_relative_odom.w();

				Eigen::Matrix3d temp_mat=temp_q.toRotationMatrix();
				Eigen::Vector3d ea1 = temp_mat.eulerAngles(2,1,0); 

				//noise of robot odometry
				double tran_noise=0.05;
				double rotation_noise=0.05*M_PI/180.0;

				Eigen::Matrix<double,6,6> covariance;
				covariance.fill(0);

				covariance(0,0)=carmen_square(fabs(t_relative_odom(0)*tran_noise)+0.01);
				covariance(1,1)=carmen_square(fabs(t_relative_odom(1)*tran_noise)+0.01);
				covariance(2,2)=carmen_square(fabs(t_relative_odom(2)*tran_noise)+0.01);

				covariance(3,3)=carmen_square(fabs(ea1[0]*rotation_noise)+0.01*M_PI/180.0);
				covariance(4,4)=carmen_square(fabs(ea1[1]*rotation_noise)+0.01*M_PI/180.0);
				covariance(5,5)=carmen_square(fabs(ea1[2]*rotation_noise)+0.01*M_PI/180.0);

				Eigen::Matrix<double,6,6> information=covariance.inverse();

				odometry->setMeasurement(relative_odom);
				odometry->setInformation(information);
				optimizer.addEdge(odometry);
			}

			drone1_x_pre=x;
			drone1_y_pre=y;
			drone1_yaw_pre=yaw;
		}


		// add drone2  drone3 odom egde
		std::map< std::string, std::vector<RobotPose> >::iterator it_odom_egde;
		for(it_odom_egde=m_v_odom_pose.begin(); it_odom_egde!=m_v_odom_pose.end(); ++it_odom_egde)
		{
			std::string drone_name=it_odom_egde->first;

			double drone_x_pre;
			double drone_y_pre;
			double drone_yaw_pre;

			if(drone_name == "drone2")
			{
				std::vector<RobotPose> odom_vector=it_odom_egde->second;

				for(int i=0; i< odom_vector.size(); i++)
				{

					RobotPose drone_pose=odom_vector[i];

					double x=drone_pose.x;
					double y=drone_pose.y;
					double yaw=drone_pose.yaw;

					Eigen::Quaterniond pose_q;
					Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
					Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
					Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));

					pose_q=yawAngle*pitchAngle*rollAngle;

					int pre_i=i-1;
					if(pre_i >= 0)
					{
						EdgeSE3* odometry = new EdgeSE3;

						odometry->vertices()[0] = optimizer.vertex(robot2_index+pre_i - vexter_num12);
						odometry->vertices()[1] = optimizer.vertex(robot2_index+i - vexter_num12);

						double pre_x=drone_x_pre;
						double pre_y=drone_y_pre;
						double pre_yaw=drone_yaw_pre;

						Eigen::Quaterniond pre_pose_q;
						Eigen::AngleAxisd pre_rollAngle(AngleAxisd(0,Vector3d::UnitX()));
						Eigen::AngleAxisd pre_pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
						Eigen::AngleAxisd pre_yawAngle(AngleAxisd(pre_yaw,Vector3d::UnitZ()));

						pre_pose_q=pre_yawAngle*pre_pitchAngle*pre_rollAngle;

						Eigen::Vector3d previous_trans(pre_x,pre_y,0);
						Eigen::Quaterniond previous_q=pre_pose_q;//w,x,y,z

						Eigen::Vector3d trans(x,y,0);
						Eigen::Quaterniond q=pose_q;//w,x,y,z

						g2o::SE3Quat previous(previous_q,previous_trans);
						g2o::SE3Quat egde_current(q,trans);

						g2o::SE3Quat relative_odom=previous.inverse()*egde_current;//relative transformtion T=inverse(A)*B,
						Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
						Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
						Eigen::Vector3d t_relative_odom=relative_odom.translation();
						Eigen::Matrix3f matrix_relative_odom=q_relative_odom.toRotationMatrix();
						Eigen::Affine3f affine_relative_odom(matrix_relative_odom);

						Eigen::Quaterniond temp_q;
						temp_q.x()=q_relative_odom.x();
						temp_q.y()=q_relative_odom.y();
						temp_q.z()=q_relative_odom.z();
						temp_q.w()=q_relative_odom.w();

						Eigen::Matrix3d temp_mat=temp_q.toRotationMatrix();
						Eigen::Vector3d ea1 = temp_mat.eulerAngles(2,1,0); 

						//noise of robot odometry
						double tran_noise=0.05;
						double rotation_noise=0.05*M_PI/180.0;

						Eigen::Matrix<double,6,6> covariance;
						covariance.fill(0);

						covariance(0,0)=carmen_square(fabs(t_relative_odom(0)*tran_noise)+0.01);
						covariance(1,1)=carmen_square(fabs(t_relative_odom(1)*tran_noise)+0.01);
						covariance(2,2)=carmen_square(fabs(t_relative_odom(2)*tran_noise)+0.01);

						covariance(3,3)=carmen_square(fabs(ea1[0]*rotation_noise)+0.01*M_PI/180.0);
						covariance(4,4)=carmen_square(fabs(ea1[1]*rotation_noise)+0.01*M_PI/180.0);
						covariance(5,5)=carmen_square(fabs(ea1[2]*rotation_noise)+0.01*M_PI/180.0);

						Eigen::Matrix<double,6,6> information=covariance.inverse();

						odometry->setMeasurement(relative_odom);
						odometry->setInformation(information);
						optimizer.addEdge(odometry);
					}

					drone_x_pre=x;
					drone_y_pre=y;
					drone_yaw_pre=yaw;
				}
			}
			if(drone_name == "drone3")
			{
				std::vector<RobotPose> odom_vector=it_odom_egde->second;

				for(int i=0; i< odom_vector.size(); i++)
				{
					RobotPose drone_pose=odom_vector[i];

					double x=drone_pose.x;
					double y=drone_pose.y;
					double yaw=drone_pose.yaw;

					Eigen::Quaterniond pose_q;
					Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
					Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
					Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));

					pose_q=yawAngle*pitchAngle*rollAngle;

					int pre_i=i-1;
					if(pre_i >= 0)
					{
						EdgeSE3* odometry = new EdgeSE3;

						odometry->vertices()[0] = optimizer.vertex(robot3_index+pre_i - vexter_num3);
						odometry->vertices()[1] = optimizer.vertex(robot3_index+i - vexter_num3);

						double pre_x=drone_x_pre;
						double pre_y=drone_y_pre;
						double pre_yaw=drone_yaw_pre;

						Eigen::Quaterniond pre_pose_q;
						Eigen::AngleAxisd pre_rollAngle(AngleAxisd(0,Vector3d::UnitX()));
						Eigen::AngleAxisd pre_pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
						Eigen::AngleAxisd pre_yawAngle(AngleAxisd(pre_yaw,Vector3d::UnitZ()));

						pre_pose_q=pre_yawAngle*pre_pitchAngle*pre_rollAngle;

						Eigen::Vector3d previous_trans(pre_x,pre_y,0);
						Eigen::Quaterniond previous_q=pre_pose_q;//w,x,y,z

						Eigen::Vector3d trans(x,y,0);
						Eigen::Quaterniond q=pose_q;//w,x,y,z

						g2o::SE3Quat previous(previous_q,previous_trans);
						g2o::SE3Quat egde_current(q,trans);

						g2o::SE3Quat relative_odom=previous.inverse()*egde_current;//relative transformtion T=inverse(A)*B,
						Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
						Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
						Eigen::Vector3d t_relative_odom=relative_odom.translation();
						Eigen::Matrix3f matrix_relative_odom=q_relative_odom.toRotationMatrix();
						Eigen::Affine3f affine_relative_odom(matrix_relative_odom);

						Eigen::Quaterniond temp_q;
						temp_q.x()=q_relative_odom.x();
						temp_q.y()=q_relative_odom.y();
						temp_q.z()=q_relative_odom.z();
						temp_q.w()=q_relative_odom.w();

						Eigen::Matrix3d temp_mat=temp_q.toRotationMatrix();
						Eigen::Vector3d ea1 = temp_mat.eulerAngles(2,1,0); 

						//noise of robot odometry
						double tran_noise=0.05;
						double rotation_noise=0.05*M_PI/180.0;

						Eigen::Matrix<double,6,6> covariance;
						covariance.fill(0);

						covariance(0,0)=carmen_square(fabs(t_relative_odom(0)*tran_noise)+0.01);
						covariance(1,1)=carmen_square(fabs(t_relative_odom(1)*tran_noise)+0.01);
						covariance(2,2)=carmen_square(fabs(t_relative_odom(2)*tran_noise)+0.01);

						covariance(3,3)=carmen_square(fabs(ea1[0]*rotation_noise)+0.01*M_PI/180.0);
						covariance(4,4)=carmen_square(fabs(ea1[1]*rotation_noise)+0.01*M_PI/180.0);
						covariance(5,5)=carmen_square(fabs(ea1[2]*rotation_noise)+0.01*M_PI/180.0);

						Eigen::Matrix<double,6,6> information=covariance.inverse();

						odometry->setMeasurement(relative_odom);
						odometry->setInformation(information);
						optimizer.addEdge(odometry);
					}

					drone_x_pre=x;
					drone_y_pre=y;
					drone_yaw_pre=yaw;
				}
			}
		}

		// add the uwb egde between drone1 and drone2 and drone3		
		
		std::map< std::string, std::vector<RobotPose> >::iterator it;
		std::map< std::string, ObjectConfig >::iterator it_m_g2o_esti=m_v_rel_pose.find("drone1");
		if(it_m_g2o_esti != m_v_rel_pose.end())
		{
			for(it=it_m_g2o_esti->second.g2o_estimate.begin(); it!=it_m_g2o_esti->second.g2o_estimate.end(); ++it)
			{
				std::string follow_name=it->first;
				std::vector<RobotPose> follow_drone_vector=it->second;
				if(follow_name == "drone2")
				{
					for(int i=0; i<follow_drone_vector.size(); i++)
					{
						
						double relative_x=follow_drone_vector[i].x;
						double relative_y=follow_drone_vector[i].y;
						double relative_yaw=follow_drone_vector[i].yaw;

						EdgeSE3* uwb_egde = new EdgeSE3;

						uwb_egde->vertices()[0] = optimizer.vertex(robot1_index+i - vexter_num12);
						uwb_egde->vertices()[1] = optimizer.vertex(robot2_index+i - vexter_num12);

						Eigen::Vector3d uwb_trans(relative_x,relative_y,0);
						Eigen::Quaterniond pose_q;
						Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
						Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
						Eigen::AngleAxisd yawAngle(AngleAxisd(relative_yaw,Vector3d::UnitZ()));

						pose_q=rollAngle*pitchAngle*yawAngle;

						g2o::SE3Quat uwb_current(pose_q,uwb_trans);

						Eigen::Quaternion<float> q_relative_uwb=uwb_current.rotation().cast<float>();
						Eigen::Vector3d t_relative_uwb=uwb_current.translation();
						Eigen::Matrix3f matrix_relative_uwb=q_relative_uwb.toRotationMatrix();
						Eigen::Affine3f affine_relative_odom(matrix_relative_uwb);

						Eigen::Matrix<double,6,6> covariance_uwb;
						covariance_uwb.fill(0);

						covariance_uwb(0,0)=carmen_square(0.6);
						covariance_uwb(1,1)=carmen_square(0.6);
						covariance_uwb(2,2)=carmen_square(1);

						covariance_uwb(3,3)=carmen_square(1);
						covariance_uwb(4,4)=carmen_square(1);
						covariance_uwb(5,5)=carmen_square(0.6);

						Eigen::Matrix<double,6,6> information_uwb=covariance_uwb.inverse();

						uwb_egde->setMeasurement(uwb_current);
						uwb_egde->setInformation(information_uwb);
						uwb_egde->setParameterId(0, 0);
						optimizer.addEdge(uwb_egde);
					}
				}
				else if(follow_name == "drone3")
				{
					for(int i=0; i<drone1_vexter_vecter.size(); i++)
					{
						RobotPose drone1_pose=drone1_vexter_vecter[i];
						
						double drone1_time=drone1_pose.timestamp;

						double timestamp_temp=INF;
						double t_num=0;
						for(int t=0; t<follow_drone_vector.size(); t++)
						{
							double time_diff=fabs(drone1_time - follow_drone_vector[t].timestamp);
							if(time_diff < timestamp_temp)
							{
								timestamp_temp=time_diff;
								t_num=t;
							}
						}
						if(timestamp_temp > time_limit)
							continue;
		static int old_t_num = 0;
		if(old_t_num == t_num)
			continue;			
		old_t_num = t_num;
						double relative_x=follow_drone_vector[t_num].x;
						double relative_y=follow_drone_vector[t_num].y;
						double relative_yaw=follow_drone_vector[t_num].yaw;
						EdgeSE3* uwb_egde = new EdgeSE3;

						uwb_egde->vertices()[0] = optimizer.vertex(robot1_index+i - vexter_num12);
						uwb_egde->vertices()[1] = optimizer.vertex(robot3_index+t_num - vexter_num3);

						Eigen::Vector3d uwb_trans(relative_x,relative_y,0);
						Eigen::Quaterniond pose_q;
						Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
						Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
						Eigen::AngleAxisd yawAngle(AngleAxisd(relative_yaw,Vector3d::UnitZ()));

						pose_q=rollAngle*pitchAngle*yawAngle;

						g2o::SE3Quat uwb_current(pose_q,uwb_trans);

						Eigen::Quaternion<float> q_relative_uwb=uwb_current.rotation().cast<float>();
						Eigen::Vector3d t_relative_uwb=uwb_current.translation();
						Eigen::Matrix3f matrix_relative_uwb=q_relative_uwb.toRotationMatrix();
						Eigen::Affine3f affine_relative_odom(matrix_relative_uwb);

						Eigen::Matrix<double,6,6> covariance_uwb;
						covariance_uwb.fill(0);

						covariance_uwb(0,0)=carmen_square(0.6);
						covariance_uwb(1,1)=carmen_square(0.6);
						covariance_uwb(2,2)=carmen_square(1);

						covariance_uwb(3,3)=carmen_square(1);
						covariance_uwb(4,4)=carmen_square(1);
						covariance_uwb(5,5)=carmen_square(0.6);

						Eigen::Matrix<double,6,6> information_uwb=covariance_uwb.inverse();

						uwb_egde->setMeasurement(uwb_current);
						uwb_egde->setInformation(information_uwb);
						uwb_egde->setParameterId(0, 0);
						optimizer.addEdge(uwb_egde);
					}
				}
			}
		}

		// add the egde between robot2 and robot3
		it_m_g2o_esti=m_v_rel_pose.find("drone2");
		if(it_m_g2o_esti != m_v_rel_pose.end())
		{
			for(it=it_m_g2o_esti->second.g2o_estimate.begin(); it!=it_m_g2o_esti->second.g2o_estimate.end(); ++it)
			{
				std::string follow_name=it->first;
				std::vector<RobotPose> follow_drone_vector=it->second;
				for(int i=0; i<drone1_vexter_vecter.size(); i++)
				{
					RobotPose drone1_pose=drone1_vexter_vecter[i];

					double drone1_time=drone1_pose.timestamp;
					double timestamp_temp=INF;
					double t_num1=0;
					for(int t=0; t<follow_drone_vector.size(); t++)
					{
						double time_diff=fabs(drone1_time - follow_drone_vector[t].timestamp);
						if(time_diff < timestamp_temp)
						{
							timestamp_temp=time_diff;
							t_num1 = t;
						}
					}
					if(timestamp_temp > time_limit)
							continue;

					double drone23_time = follow_drone_vector[t_num1].timestamp;
					timestamp_temp=INF;
					int t_num2;
					for(int t=0; t<drone_position_vecter.size(); t++)
					{
						double time_diff=fabs(drone23_time - drone_position_vecter[t].timestamp);
						if(time_diff < timestamp_temp)
						{
							timestamp_temp=time_diff;
							t_num2 = t;
						}
					}
					if(timestamp_temp > time_limit)
							continue;

					double relative_x=follow_drone_vector[t_num1].x;
					double relative_y=follow_drone_vector[t_num1].y;
					double relative_yaw=follow_drone_vector[t_num1].yaw;
		static int old_t_num2 = 0;
		if(old_t_num2 == t_num2)
			continue;			
		old_t_num2 = t_num2;

					EdgeSE3* uwb_egde = new EdgeSE3;

					uwb_egde->vertices()[0] = optimizer.vertex(robot2_index+i - vexter_num12);
					uwb_egde->vertices()[1] = optimizer.vertex(robot3_index+t_num2 - vexter_num3);
					
					
					Eigen::Vector3d uwb_trans(relative_x,relative_y,0);
					Eigen::Quaterniond pose_q;
					Eigen::AngleAxisd rollAngle(AngleAxisd(0,Vector3d::UnitX()));
					Eigen::AngleAxisd pitchAngle(AngleAxisd(0,Vector3d::UnitY()));
					Eigen::AngleAxisd yawAngle(AngleAxisd(relative_yaw,Vector3d::UnitZ()));

					pose_q=rollAngle*pitchAngle*yawAngle;

					g2o::SE3Quat uwb_current(pose_q,uwb_trans);

					Eigen::Quaternion<float> q_relative_uwb=uwb_current.rotation().cast<float>();
					Eigen::Vector3d t_relative_uwb=uwb_current.translation();
					Eigen::Matrix3f matrix_relative_uwb=q_relative_uwb.toRotationMatrix();
					Eigen::Affine3f affine_relative_odom(matrix_relative_uwb);

					Eigen::Matrix<double,6,6> covariance_uwb;
					covariance_uwb.fill(0);

					covariance_uwb(0,0)=carmen_square(0.8);
					covariance_uwb(1,1)=carmen_square(0.8);
					covariance_uwb(2,2)=carmen_square(1);

					covariance_uwb(3,3)=carmen_square(1);
					covariance_uwb(4,4)=carmen_square(1);
					covariance_uwb(5,5)=carmen_square(0.8);

					Eigen::Matrix<double,6,6> information_uwb=covariance_uwb.inverse();

					uwb_egde->setMeasurement(uwb_current);
					uwb_egde->setInformation(information_uwb);
					uwb_egde->setParameterId(0, 0);
					optimizer.addEdge(uwb_egde);
				}
			}
		}


		// start optimization
		VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
		firstRobotPose->setFixed(true);
		optimizer.setVerbose(false);

		optimizer.initializeOptimization();
		optimizer.optimize(30);

		OptimizableGraph::VertexIDMap graph_vertices=optimizer.vertices();
		OptimizableGraph::VertexIDMap::iterator it_map;	

		std::map <std::string, std::map <int, carmen_6d_point_t>> record_data;
		std::map <std::string, std::vector< carmen_6d_point_t >> g2o_track_plot;
		int nsize = graph_vertices.size();
		int robot1_lab=0;
		int robot2_lab=0;
		int robot3_lab=0;
		for(it_map = graph_vertices.begin(); it_map != graph_vertices.end(); ++it_map)
		{
			VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
			g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();      //robot  pose

			Eigen::Quaterniond q=pose_se3quat.rotation();

			Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);			//robot orientation

			double robot_yaw = euler[2] + carmen_gaussian_random(0, 0.00);
			double robot_x=pose_se3quat.translation()[0] + carmen_gaussian_random(0, 0.00);
			double robot_y=pose_se3quat.translation()[1] + carmen_gaussian_random(0, 0.00);
			
			// adjust results pose
			RobotPose rotation_se3;
			rotation_se3.x = ROTATION_X;
			rotation_se3.y = ROTATION_Y;
			rotation_se3.yaw = ROTATION_YAW;

			RobotPose robot_se3;
			robot_se3.x = robot_x;
			robot_se3.y = robot_y;
			robot_se3.yaw = robot_yaw;

			RobotPose result_ = computeT1_x_T2(rotation_se3, robot_se3);


			carmen_6d_point_t robot_pose;
			robot_pose.x = result_.x;
			robot_pose.y = result_.y;
			robot_pose.theta_z = result_.yaw;
			
			std::string drone;
			int timestamp_num=0;
			if(it_map->first < 10000){
				drone="drone1";
				timestamp_num=it_map->first;
				if(timestamp_num >= robot1_lab)
				{
					robot1_lab=timestamp_num;
				}
				record_data[drone][timestamp_num]=robot_pose;
			}
			else if ((it_map->first >= 10000)&&(it_map->first < 20000)){
				drone = "drone2";
				timestamp_num=it_map->first-10000;
				if(timestamp_num >= robot2_lab)
				{
					robot2_lab=timestamp_num;
				}
				record_data[drone][timestamp_num]=robot_pose;
			}
			else if ((it_map->first >= 20000)&&(it_map->first < 30000)){
				drone = "drone3";
				timestamp_num=it_map->first-20000;
				if(timestamp_num >= robot3_lab)
				{
					robot3_lab=timestamp_num;
				}
				record_data[drone][timestamp_num]=robot_pose;
			}
			
			g2o_track_plot[drone].push_back(robot_pose);
		}

		static int old_robot1_lab = 0;
		static int old_robot2_lab = 0;
		static int old_robot3_lab = 0;
		if(robot1_lab != 0 && robot1_lab != old_robot1_lab)
		{
			est_current_odom1.x = record_data["drone1"][robot1_lab].x;
			est_current_odom1.y = record_data["drone1"][robot1_lab].y;
			est_current_odom1.yaw = carmen_normalize_theta(record_data["drone1"][robot1_lab].theta_z);
		}
		if(robot2_lab != 0 && robot2_lab != old_robot2_lab)
		{
			est_current_odom2.x = record_data["drone2"][robot2_lab].x;
			est_current_odom2.y = record_data["drone2"][robot2_lab].y;
			est_current_odom2.yaw = carmen_normalize_theta(record_data["drone2"][robot2_lab].theta_z);
		}
		if(robot3_lab != 0 && robot3_lab != old_robot3_lab)
		{
			est_current_odom3.x = record_data["drone3"][robot3_lab].x;
			est_current_odom3.y = record_data["drone3"][robot3_lab].y;
			est_current_odom3.yaw = carmen_normalize_theta(record_data["drone3"][robot3_lab].theta_z);
		}
		old_robot1_lab = robot1_lab;
		old_robot2_lab = robot2_lab;
		old_robot3_lab = robot3_lab;
		//optimization data
		std::map <std::string, std::map <double, RobotPose>> adjusted_data;
		std::map <std::string, std::map <int, carmen_6d_point_t>>::iterator it_record;
		for(it_record=record_data.begin(); it_record!=record_data.end(); ++it_record)
		{
			std::string drone_name=it_record->first;

			std::map <int, carmen_6d_point_t> data=it_record->second;
			if(drone_name == "drone3")
			{
				for(int i = 0; i < drone_position_vecter.size(); i++)
				{
					std::map <int, carmen_6d_point_t>::iterator it_data=data.find(int(i));
					RobotPose pose=drone_position_vecter[i];
					carmen_6d_point_t optimi_pose=it_data->second;
					
					double time=pose.timestamp;
					double x=optimi_pose.x;
					double y=optimi_pose.y;
					double yaw=optimi_pose.theta_z;

					RobotPose adjusted;
					adjusted.x=x;
					adjusted.y=y;
					adjusted.yaw=yaw;
					adjusted.timestamp=time;

					adjusted_data[drone_name][int(i)]=adjusted;

					RobotPose pose1;
					pose1.timestamp = time;
					pose1.x = x;
					pose1.y = y;
					pose1.yaw = yaw;
					m_g2o_results[drone_name].push_back(pose1);
				}
			}
			else
			{
				for(int i=0; i<drone1_vexter_vecter.size(); i++)
				{
					std::map <int, carmen_6d_point_t>::iterator it_data=data.find(int(i));

					RobotPose pose=drone1_vexter_vecter[i];
					carmen_6d_point_t optimi_pose=it_data->second;

					double time=pose.timestamp;
					double x=optimi_pose.x;
					double y=optimi_pose.y;
					double yaw=optimi_pose.theta_z;

					RobotPose adjusted;
					adjusted.x=x;
					adjusted.y=y;
					adjusted.yaw=yaw;
					adjusted.timestamp=time;

					adjusted_data[drone_name][int(i)]=adjusted;


					RobotPose pose1;
					pose1.timestamp = time;
					pose1.x = x;
					pose1.y = y;
					pose1.yaw = yaw;
					m_g2o_results[drone_name].push_back(pose1);
				}
			}
		}
		plot_all_g2o_tracks();
		std::map<int, std::vector<double> > m_v_pose_error;
		std::map<int, std::vector<double> > m_v_yaw_error;
		// cout << "12" << endl;
		computeG2oRelativeError(adjusted_data, "drone1", "drone2", m_v_pose_error[0], m_v_yaw_error[0]);
		// cout << "13" << endl;
		computeG2oRelativeError(adjusted_data, "drone1", "drone3", m_v_pose_error[1], m_v_yaw_error[1]);
		// cout << "23" << endl;
		computeG2oRelativeError(adjusted_data, "drone3", "drone2", m_v_pose_error[2], m_v_yaw_error[2]);
		double mean_pose_error=0;
		double mean_yaw_error=0;
		double std_pose_error=0;
		double std_yaw_error=0;
		double sum_rel_pose_error = 0;
		double sum_rel_yaw_error = 0;
		double std_sum_rel_pose_error = 0;
		double std_sum_rel_yaw_error = 0;
		for(int i=0; i<m_v_pose_error.size(); i++)
		{
			double robot_pose_error=0;
			double robot_yaw_error=0;
			double std_robot_pose_error=0;
			double std_robot_yaw_error=0;
			computeMeanVariance(m_v_pose_error[i], robot_pose_error, std_robot_pose_error);
			computeMeanVariance(m_v_yaw_error[i], robot_yaw_error, std_robot_yaw_error);
			switch(i)
			{
				case 0: std::cout << "robot1_2: " << robot_pose_error << "   " << std_robot_pose_error << "   " << robot_yaw_error << "   " << std_robot_yaw_error << std::endl; break;
				case 1: std::cout << "robot1_3: " << robot_pose_error << "   " << std_robot_pose_error << "   " << robot_yaw_error << "   " << std_robot_yaw_error << std::endl; break;
				case 2: std::cout << "robot3_2: " << robot_pose_error << "   " << std_robot_pose_error << "   " << robot_yaw_error << "   " << std_robot_yaw_error << std::endl; break;
				default: break;
			}
			sum_rel_pose_error += robot_pose_error;
			sum_rel_yaw_error += robot_yaw_error;
			std_sum_rel_pose_error += std_robot_pose_error;
			std_sum_rel_yaw_error += std_robot_yaw_error;
		}
		mean_pose_error = sum_rel_pose_error/(double)(m_v_pose_error.size());
		mean_yaw_error = sum_rel_yaw_error/(double)(m_v_pose_error.size());
		std_pose_error = std_sum_rel_pose_error/(double)(m_v_pose_error.size());
		std_yaw_error = std_sum_rel_yaw_error/(double)(m_v_pose_error.size());

		
		std::cout << m_v_pose_error[0].size() << "   " << m_v_pose_error[1].size() << "   " << m_v_pose_error[2].size() << std::endl;
		std::cout << "G2O Error: " <<  mean_pose_error << "   " << std_pose_error << "   " << mean_yaw_error*180/3.14 << "   " << std_yaw_error*180/3.14 << std::endl;
		m_g2o_results.clear();
		sleep(2);
	}
}


int main(int argc, char *argv[]){
	generate_time_based_random_seed();
	std::ofstream file5(uwb_measure_name, std::ios::out);
	std::ofstream file6(amcl_measure_name, std::ios::out);
	std::ofstream file7(odom_measure_name, std::ios::out);
	std::ofstream file8(test_file_name, std::ios::out);
	std::ofstream filea(com_trues_dist_file, std::ios::out);
	std::ofstream fileb(com_uwb_measure_file, std::ios::out);
	std::ofstream filec(com_odom_file, std::ios::out);
	std::ofstream filed(com_amcl_file, std::ios::out);
	std::ofstream file11(drone_odom_file1, std::ios::out);
	std::ofstream file12(drone_odom_file2, std::ios::out);
	std::ofstream file13(drone_odom_file3, std::ios::out);
	std::ofstream file14(drone_AMCL_file1, std::ios::out);
	std::ofstream file15(drone_AMCL_file2, std::ios::out);
	std::ofstream file16(drone_AMCL_file3, std::ios::out);
	std::ofstream file17(drone_est12_file, std::ios::out);
	std::ofstream file18(drone_est13_file, std::ios::out);
	std::ofstream file19(drone_est23_file, std::ios::out);
	std::ofstream file1a(drone_rel_pose_file, std::ios::out);
	
	covariance1 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	covariance2 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	covariance3 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	ros::init(argc, argv, "relative_localization_node");

	ros::NodeHandle n("~");

	std::string configure_file;

	n.getParam("configure", configure_file);
	std::cout<<configure_file.c_str()<<std::endl;

	loadConfigureFile(configure_file);
	
	ros::Subscriber sub_robot1_amcl = n.subscribe("/robot1/amcl_pose", 1000, robot1AMCLCallback);
	ros::Subscriber sub_robot2_amcl = n.subscribe("/robot2/amcl_pose", 1000, robot2AMCLCallback);
	ros::Subscriber sub_robot3_amcl = n.subscribe("/robot3/amcl_pose", 1000, robot3AMCLCallback);

#ifndef SIMULATION_ENABLE
	ros::Subscriber sub_uwb_robot1 = n.subscribe("/robot1/nlink_linktrack_nodeframe2", 1000, robot1UWBCallback);
	ros::Subscriber sub_uwb_robot2 = n.subscribe("/robot2/nlink_linktrack_nodeframe2", 1000, robot2UWBCallback);	
	// ros::Subscriber sub_uwb_robot3 = n.subscribe("/robot3/nlink_linktrack_nodeframe2", 1000, robot3UWBCallback);
#endif

	ros::Subscriber sub_robot1_odom = n.subscribe("/robot1/odom", 1000, robot1OdomCallback);
	ros::Subscriber sub_robot2_odom = n.subscribe("/robot2/odom", 1000, robot2OdomCallback);
	ros::Subscriber sub_robot3_odom = n.subscribe("/robot3/odom", 1000, robot3OdomCallback);
	
	robot1_pub = n.advertise<nav_msgs::Odometry>("/robot1/est_odom", 10);
	robot2_pub = n.advertise<nav_msgs::Odometry>("/robot2/est_odom", 10);
	robot3_pub = n.advertise<nav_msgs::Odometry>("/robot3/est_odom", 10);
	
	pthread_t tid;
	int ret = pthread_create(&tid, NULL, computePositionError, NULL);
#ifndef SIMULATION_ENABLE	
	pthread_t tid1;
	int ret1 = pthread_create(&tid1, NULL, g2o_process, NULL);
#endif
	ros::spin();
	return 0;
}
