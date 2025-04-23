#include "relative_localization_node.h"
std::map<std::string, RobotPose> m_rel_pose;
const int TIME_WINDOW_SIZE = 1000;
const int TIME_GRAP = 50;
const int NUM_POINTS = TIME_WINDOW_SIZE / TIME_GRAP;
const int LAMDA = 20;
ros::Publisher robot2_pub;
//------------------------------------------------------------------------
std::string odom1_file_name("/home/dzy/Desktop/odom1_file_p.txt");
std::string odom2_file_name("/home/dzy/Desktop/odom2_file_p.txt");
std::string amcl1_file_name("/home/dzy/Desktop/amcl1_file_p.txt");
std::string amcl2_file_name("/home/dzy/Desktop/amcl2_file_p.txt");
std::string uwb_measure_name("/home/dzy/Desktop/uwb_measure.txt");
std::string amcl_measure_name("/home/dzy/Desktop/amcl_measure.txt");
std::string odom_measure_name("/home/dzy/Desktop/odom_measure.txt");
std::string test_file_name("/home/dzy/Desktop/test1.txt");
std::string com_trues_dist_file("/home/dzy/Desktop/trues_dist.txt");
std::string com_uwb_measure_file("/home/dzy/Desktop/uwb_dist.txt");
std::string com_odom_file("/home/dzy/Desktop/com_odom.txt");
std::string com_amcl_file("/home/dzy/Desktop/com_amcl.txt");
std::string gt_file_name("/home/dzy/post-graduation_ws/src/single_uwb_relative_localization/data/amcl_measure.txt");
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

		// //estimation*odom
        RobotPose est;
        est.x = x;
        est.y = y;
        est.yaw = theta;
        RobotPose tag_odom;
        tag_odom.x = x_tag;
        tag_odom.y = y_tag;
        tag_odom.yaw = theta_tag;
        g2o::SE3Quat est_se3 = computePoseToG2oSE3(est);
        g2o::SE3Quat tag_odom_se3 = computePoseToG2oSE3(tag_odom);
		g2o::SE3Quat tag_robot_se3 = est_se3*tag_odom_se3;//relative transformtion T=A*B,

        RobotPose delta_pose;
        delta_pose.x = delta_tag_x;
        delta_pose.y = delta_tag_y;
        delta_pose.yaw = 0;
        g2o::SE3Quat delta_pose_se3 = computePoseToG2oSE3(delta_pose);
        g2o::SE3Quat tag_uwb_se3 = tag_robot_se3 * delta_pose_se3;
        RobotPose tag_uwb = computeG2oSE3ToPose(tag_uwb_se3);

        RobotPose anchor_robot;
        anchor_robot.x = peer_tag_x;
        anchor_robot.y = peer_tag_y;
        anchor_robot.yaw = peer_tag_theta;
        g2o::SE3Quat anchor_robot_se3 = computePoseToG2oSE3(anchor_robot);
        g2o::SE3Quat anchor_uwb_se3 = anchor_robot_se3 * delta_pose_se3;
        RobotPose anchor_uwb = computeG2oSE3ToPose(anchor_uwb_se3);

        double fval = sqrt((tag_uwb.x-anchor_uwb.x)*(tag_uwb.x-anchor_uwb.x) + (tag_uwb.y-anchor_uwb.y)*(tag_uwb.y-anchor_uwb.y));

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
std::map<std::string, std::vector<RobotPose> > m_g2o_res;
void plot_all_g2o_tracks(int minX=-8, int maxX=15, int minY=-8, int maxY=20)
// --------------------------------------------------------------------------
{

	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'g2o Estimation track'\n";
	cmd+="set size ratio -1\n";	
	cmd+="unset arrow\n";

	std::map <std::string, std::vector< RobotPose >>::iterator it;


	cmd+="set key top right height 0.5 width -2\n";


	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";

	int line_style=1;
	//set the line colors
	for(it = m_v_amcl.begin(); it != m_v_amcl.end(); ++it)
	{
		std::string line_color="#00FF00";	

		if(it->first=="robot1")
		{
			line_color="#00FF00";
		}			
		else if(it->first=="robot2")
		{
			line_color="#00FF00";
		}			
		else if(it->first=="robot3")
		{
			line_color="#00FF00";
		}			

		
		cmd+="'-' u 1:2 w lp lw 1 pt 9 ps 0.5 lc rgb '"+line_color+"' ti '"+it->first+"-amcl',";
		line_style=line_style+1;
	}
	
	for(it = m_g2o_res.begin(); it != m_g2o_res.end(); ++it)
	{
		std::string line_color="#00FF00";	

		if(it->first=="robot1")
		{
			line_color="#FF0000";
		}			
		else if(it->first=="robot2")
		{
			line_color="#0000FF";
		}			
		else if(it->first=="robot3")
		{
			line_color="#000000";
		}			

		
		cmd+="'-' u 1:2 w lp lw 1 pt 9 ps 0.5 lc rgb '"+line_color+"' ti '"+it->first+"-g2o',";
		line_style=line_style+1;
	}
	cmd += "\n";
	for(it = m_v_amcl.begin(); it != m_v_amcl.end(); ++it)
	{

		//output the measurements
		std::vector< RobotPose > trajectories_low_frequency=it->second;

		for(int i=0;i<trajectories_low_frequency.size();i=i+1)
		{
			cmd += toString( trajectories_low_frequency[i].x ) + ' ' + toString( trajectories_low_frequency[i].y ) + ' ' + toString( 0.5 ) + '\n';				
		}
		cmd += "e\n";
	}

	for(it = m_g2o_res.begin(); it != m_g2o_res.end(); ++it)
	{

		//output the measurements
		std::vector< RobotPose > trajectories_low_frequency=it->second;

		for(int i=0;i<trajectories_low_frequency.size();i=i+1)
		{
			cmd += toString( trajectories_low_frequency[i].x ) + ' ' + toString( trajectories_low_frequency[i].y ) + ' ' + toString( 0.5 ) + '\n';				
		}
		cmd += "e\n";
	}
	
	m_plot_g2o_track->commandStr( cmd );
}
//========================================================================

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
		Xbar=7.852299;
		Pbar = 0.5;
		Q=0.5;
		R=0.1;
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
};
inline double KalmanFilter::Kalman_filters(double z, bool NLOS_flag, double d_pre)                                                                                                                                                                                                             
{
    static int flag=1;
    static double old_d_pre;
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
        Xbar = X_bar + K*(z-X_bar);
        Pbar = P_bar - K*P_bar;
        // cout << Q << "\t" << R << "\t" << Xbar << "\t" << Pbar << "\t" << K << "\n------" << endl;
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
		// g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber();	
		// rk->setDelta(0.2);
		// e->setRobustKernel(rk);
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
		
		g2o::SE3Quat tag_se3 =  computePoseToG2oSE3(tag_odom);
		g2o::SE3Quat lag_tag_se3 =  computePoseToG2oSE3(lag_tag_odom);
		g2o::SE3Quat relative_tag = tag_se3.inverse() * lag_tag_se3;

		RobotPose relative_tag_pose = computeG2oSE3ToPose(relative_tag);
		// 1.2)) anchor odom

		g2o::SE3Quat anchor_se3 =  computePoseToG2oSE3(anchor_odom);
		g2o::SE3Quat lag_anchor_se3 =  computePoseToG2oSE3(lag_anchor_odom);
		g2o::SE3Quat relative_anchor = anchor_se3.inverse() * lag_anchor_se3;

		RobotPose relative_anchor_pose = computeG2oSE3ToPose(relative_anchor);
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
	plotWindow();
	m_v_win[tag_name].clear();
	m_v_win[anchor_name].clear();
	// 3) localization based g2o  
	CommonPose odom_pose =  v_com_odom[total_num-1];  
	g2o::SE3Quat tag_o_se3 = computePoseToG2oSE3(odom_pose.tagPose);
	g2o::SE3Quat anchor_o_se3 = computePoseToG2oSE3(odom_pose.anchorPose);
	g2o::SE3Quat relative_o_se3 = anchor_o_se3.inverse() * tag_o_se3;
	
	RobotPose init;
	RobotPose relative_pose;
	std::map<std::string, int>::iterator it_flag = m_flag1.find(anchor_name+tag_name);
	if(it_flag == m_flag1.end())
		m_flag1[anchor_name+tag_name] = 0;

	double dist1 = v_robot_uwb_measure[total_num - 1].range;
	if(m_flag1[anchor_name+tag_name] == 0)
	{
		m_flag1[anchor_name+tag_name] = 1;
	    init=computeG2oSE3ToPose(relative_o_se3);
	}
	else
	{
		g2o::SE3Quat old_rel_se3 = computePoseToG2oSE3(m_old_rel[anchor_name+tag_name]);
		g2o::SE3Quat T2 = computePoseToG2oSE3(m_old_odom_pose[anchor_name+tag_name].anchorPose).inverse() * computePoseToG2oSE3(odom_pose.anchorPose);
		g2o::SE3Quat T1 = computePoseToG2oSE3(m_old_odom_pose[anchor_name+tag_name].tagPose).inverse() * computePoseToG2oSE3(odom_pose.tagPose); 	
		g2o::SE3Quat now_pose_se3 = T2.inverse() * (old_rel_se3 * T1);
		RobotPose now_pose = computeG2oSE3ToPose(now_pose_se3);
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
	m_old_odom[anchor_name+tag_name] = computeG2oSE3ToPose(relative_o_se3);
	m_old_odom_pose[anchor_name+tag_name] = odom_pose;
	// 4) compute position error
	g2o::SE3Quat rel_se3 = computePoseToG2oSE3(relative_pose);
	g2o::SE3Quat odom_se3 = computePoseToG2oSE3(odom_pose.anchorPose);
	g2o::SE3Quat glo_se3 = odom_se3 * rel_se3;
	RobotPose glo_ = computeG2oSE3ToPose(glo_se3);
	m_v_uwb_rel[tag_name].push_back(glo_);
	return relative_pose;
}
//------------------------------------------------------------------------

std::map<std::string, std::vector<RobotPose> > m_v_vertex;
std::map<std::string, std::vector<RobotPose> > m_v_odom_edge;
std::vector<RobotPose> v_uwb12_edge;
std::vector<RobotPose> v_uwb13_edge;
std::vector<RobotPose> v_uwb23_edge;
std::vector<double> v_graph_time;
void* poseGraph_process(void* args)
{
	while(1)
	{
		// 1. create a g2o problem
		SparseOptimizer optimizer;
		//Levenberg,block solver
		typedef g2o::BlockSolver< BlockSolverX >  SlamBlockSolver;
		typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
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

		int vertex1_num = 0;
		int vertex2_num = 0;
		int vertex3_num = 0;
		// 2. add vertex --- vertex is robot1 odom transform other robot result of relative pose
		int vertex_index=0;
		std::map<std::string, std::vector<RobotPose> >::iterator it_vertex;
		for(it_vertex=m_v_vertex.begin(); it_vertex!=m_v_vertex.end(); it_vertex++)
		{
			std::vector<RobotPose> v_vertex = it_vertex->second;
			for(int i=0; i<v_vertex.size(); i++)
			{
				VertexSE3* vertex =  new VertexSE3;
				g2o::SE3Quat vertex_se3 = computePoseToG2oSE3(v_vertex[i]);
				vertex->setId(vertex_index);
				vertex->setEstimate(vertex_se3);
				optimizer.addVertex(vertex);
				vertex_index++;
				if(it_vertex->first == "robot1")
					vertex1_num++;
				else if(it_vertex->first == "robot2")
					vertex2_num++;
				else if(it_vertex->first == "robot3")\
					vertex3_num++;
			}
		}

		// 3.add odom edge
		int odom_edge_index = 0;
		std::map<std::string, std::vector<RobotPose> >::iterator it_edge;
		for(it_edge=m_v_odom_edge.begin(); it_edge!=m_v_odom_edge.end(); it_edge++)
		{
			RobotPose old_odom_edge;
			std::vector<RobotPose> v_odom_edge = it_edge->second;
			for(int i=0; i<v_odom_edge.size(); i++)
			{
				RobotPose odom_edge = v_odom_edge[i];
				if(i>=1)
				{
					EdgeSE3* odometry = new EdgeSE3;

					odometry->vertices()[0] = optimizer.vertex(odom_edge_index-1);
					odometry->vertices()[1] = optimizer.vertex(odom_edge_index);
					g2o::SE3Quat odom_edge_se3 = computePoseToG2oSE3(old_odom_edge).inverse() * computePoseToG2oSE3(odom_edge);
					RobotPose odom = computeG2oSE3ToPose(odom_edge_se3);
					//noise of robot odometry
					double tran_noise=0.05;
					double rotation_noise=0.05*M_PI/180.0;

					Eigen::Matrix<double,6,6> covariance;
					covariance.fill(0);

					covariance(0,0)=carmen_square((odom.x*tran_noise)+0.01);
					covariance(1,1)=carmen_square((odom.y*tran_noise)+0.01);
					covariance(2,2)=carmen_square((0*tran_noise)+0.01);

					covariance(3,3)=carmen_square(0+0.01*M_PI/180.0);
					covariance(4,4)=carmen_square(0+0.01*M_PI/180.0);
					covariance(5,5)=carmen_square((odom.yaw*rotation_noise)+0.01*M_PI/180.0);

					Eigen::Matrix<double,6,6> information=covariance.inverse();

					odometry->setMeasurement(odom_edge_se3);
					odometry->setInformation(information);
					optimizer.addEdge(odometry);
				}
				old_odom_edge = odom_edge;
				odom_edge_index++;
			}
		}

		// 3. add UWB edge
		for(int i=0; i<v_uwb12_edge.size(); i++)
		{
			EdgeSE3* uwb_egde = new EdgeSE3;

			uwb_egde->vertices()[0] = optimizer.vertex(i);
			uwb_egde->vertices()[1] = optimizer.vertex(vertex1_num+i);

			g2o::SE3Quat uwb_current = computePoseToG2oSE3(v_uwb12_edge[i]);


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
		for(int i=0; i<v_uwb13_edge.size(); i++)
		{
			EdgeSE3* uwb_egde = new EdgeSE3;
			double timestamp =v_uwb13_edge[i].timestamp;
			double min_diff = INFINITY;
			int record_j = -1;
			for(int j=0; j< m_v_vertex["robot1"].size(); j++)
			{
				double diff = fabs(timestamp - m_v_vertex["robot1"][j].timestamp);
				if(diff < min_diff)
				{
					min_diff = diff;
					record_j = j;
				}
			}
			if(record_j < 0 || min_diff > 0.5)
				continue;
			static int old_recard_j = 0;
			if(record_j == old_recard_j)
				continue;
			old_recard_j = record_j;

			uwb_egde->vertices()[0] = optimizer.vertex(record_j);
			uwb_egde->vertices()[1] = optimizer.vertex(vertex1_num+vertex2_num+i);

			g2o::SE3Quat uwb_current = computePoseToG2oSE3(v_uwb13_edge[i]);


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
		for(int i=0; i<v_uwb23_edge.size(); i++)
		{
			EdgeSE3* uwb_egde = new EdgeSE3;
			double timestamp =v_uwb23_edge[i].timestamp;
			double min_diff = INFINITY;
			int record_j = -1;
			for(int j=0; j< m_v_vertex["robot2"].size(); j++)
			{
				double diff = fabs(timestamp - m_v_vertex["robot2"][j].timestamp);
				if(diff < min_diff)
				{
					min_diff = diff;
					record_j = j;
				}
			}
			if(record_j < 0 || min_diff > 0.5)
				continue;
			static int old_recard_j = 0;
			if(record_j == old_recard_j)
				continue;
			old_recard_j = record_j;
				
			min_diff = INFINITY;
			int record_k = -1;
			for(int k=0; k< m_v_vertex["robot3"].size(); k++)
			{
				double diff = fabs(timestamp - m_v_vertex["robot3"][k].timestamp);
				if(diff < min_diff)
				{
					min_diff = diff;
					record_k = k;
				}
			}
			if(record_k < 0 || min_diff > 0.5)
				continue;
			static int old_recard_k = 0;
			if(record_k == old_recard_k)
				continue;
			old_recard_k = record_k;

			uwb_egde->vertices()[0] = optimizer.vertex(vertex1_num+record_j);
			uwb_egde->vertices()[1] = optimizer.vertex(vertex1_num+vertex2_num+record_k);

			g2o::SE3Quat uwb_current = computePoseToG2oSE3(v_uwb23_edge[i]);


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
		if(vertex1_num*vertex2_num != 0)
		{
			ros::Time begin = ros::Time::now();
			VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
			firstRobotPose->setFixed(false);
			optimizer.setVerbose(false);

			optimizer.initializeOptimization();
			optimizer.optimize(30);
			ros::Time end = ros::Time::now();
			OptimizableGraph::VertexIDMap graph_vertices=optimizer.vertices();
			OptimizableGraph::VertexIDMap::iterator it_map;	
			std::map<std::string, std::map<int, RobotPose> > m_m_g2o_res;
			int nsize = graph_vertices.size();
			for(it_map = graph_vertices.begin(); it_map != graph_vertices.end(); ++it_map)
			{
				VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
				g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();      //robot  pose

				Eigen::Quaterniond q=pose_se3quat.rotation();

				Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);			//robot orientation

				double robot_yaw = euler[2] + carmen_gaussian_random(0, 0.00);
				double robot_x=pose_se3quat.translation()[0] + carmen_gaussian_random(0, 0.00);
				double robot_y=pose_se3quat.translation()[1] + carmen_gaussian_random(0, 0.00);

				RobotPose robot_pose;
				robot_pose.x = robot_x;
				robot_pose.y = robot_y;
				robot_pose.yaw = robot_yaw;
				
				if(it_map->first < vertex1_num)
				{
					robot_pose.timestamp = m_v_vertex["robot1"][it_map->first].timestamp;
					m_g2o_res["robot1"].push_back(robot_pose);
					m_m_g2o_res["robot1"][it_map->first] = robot_pose;
				}
				else if(it_map->first >= vertex1_num && it_map->first < vertex1_num+vertex2_num)
				{
					robot_pose.timestamp = m_v_vertex["robot2"][it_map->first-vertex1_num].timestamp;
					m_g2o_res["robot2"].push_back(robot_pose);
					m_m_g2o_res["robot2"][it_map->first-vertex1_num] = robot_pose;
				}
				else if(it_map->first >= vertex1_num+vertex2_num && it_map->first < vertex1_num+vertex2_num+vertex3_num)
				{
					robot_pose.timestamp = m_v_vertex["robot3"][it_map->first-vertex1_num+vertex2_num].timestamp;
					m_g2o_res["robot3"].push_back(robot_pose);
					m_m_g2o_res["robot3"][it_map->first-vertex1_num+vertex2_num] = robot_pose;
				}
			}
			plot_all_g2o_tracks();
			m_g2o_res.clear();
			double graph_time_cast = end.toSec() - begin.toSec();
			v_graph_time.push_back(graph_time_cast);
			double mean_time=0;
			double std_time=0;
			computeMeanVariance(v_graph_time, mean_time, std_time);
			cout << "graph Time cast mean: " << mean_time << endl;
			// for(int i=0; i<v_amcl12.size(); i++)
			// {
			// 	RobotPose amcl = v_amcl12[i];
			// 	RobotPose g2o_anchor = m_m_g2o_res["robot1"][i];
			// 	RobotPose g2o_tag = m_m_g2o_res["robot2"][i];

			// 	g2o::SE3Quat g2o_se3 = computePoseToG2oSE3(g2o_anchor).inverse() * computePoseToG2oSE3(g2o_tag);
			// 	g2o::SE3Quat amcl_se3 = computePoseToG2oSE3(amcl);

			// 	g2o::SE3Quat error_se3 = amcl_se3.inverse() * g2o_se3;
			// 	carmen_6d_point_t error = computeG2oSE3ToPose(error_se3);
			// 	double yaw_error = fabs(error.theta);
			// 	double pose_error = sqrt(error.x*error.x + error.y*error.y);
			// 	v_g2o_yaw_error.push_back(yaw_error);
			// 	v_g2o_pose_error.push_back(pose_error);

			// 	std::ofstream rel_est_file(rel_est_file_name, std::ios::app);
			// 	rel_est_file << "robot1\t" << to_string(g2o_anchor.x) << "\t" << to_string(g2o_anchor.y) << "\t"<< to_string(g2o_anchor.theta) << "\n"
			// 				<< "robot2\t" << to_string(g2o_tag.x) << "\t" << to_string(g2o_tag.y) << "\t"<< to_string(g2o_tag.theta) << "\n";
			// 	rel_est_file.close();
			// }
			// double mean_pose = 0;
			// double std_pose_error = 0;
			// double mean_yaw = 0;
			// double std_yaw_error = 0;
			// computeMeanVariance(v_g2o_pose_error, mean_pose, std_pose_error);
			// computeMeanVariance(v_g2o_yaw_error, mean_yaw, std_yaw_error);

			// cout << "g2o_error pose: " << mean_pose << "\t" << std_pose_error << "\n"
			// 	<< "-----------yaw: " << mean_yaw*180/3.14 << "\t" << std_yaw_error*180/3.14 << endl;
		}
		sleep(1);
	}
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
const double time_limit = 0.1;

bool timeSynchronization(std::string anchor_name, std::string tag_name, CommonPose &now_odom)
{
	std::map<std::string, IndexNum>::iterator it_index = m_index_record.find(anchor_name+tag_name);
	if(it_index == m_index_record.end())
		m_index_record[anchor_name+tag_name].index1 = 0;
	double timestamp = m_v_odom[anchor_name].back().timestamp;
	double time_min_diff = INFINITY;
	int record_i=-1;
	for(int i=m_index_record[anchor_name+tag_name].index1; i<m_v_odom[tag_name].size(); i++)
	{
		double time_diff = fabs(timestamp - m_v_odom[tag_name][i].timestamp);
		if(time_diff < time_min_diff)
		{
			time_min_diff = time_diff;
			record_i  = i;
		}
	}
	if(time_min_diff > time_limit||record_i < 0)
		return false;
	now_odom.anchorPose = m_v_odom[anchor_name].back();
	now_odom.tagPose = m_v_odom[tag_name][record_i];
	now_odom.timeStamp = timestamp;
	m_index_record[anchor_name+tag_name].index1 = record_i;
	return true;
}
std::map<std::string, CommonPose> m_old_odom_of_uwb;
void uwbKalmanFilter(UWBMessage &uwb_message, CommonPose now_odom, RobotPose est_rel, KalmanFilter *kf, std::string name)
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
	double mean = sum/(double)(v_win_fp_rx.size());
	double sum1=0;
	for(int i=0; i<v_win_fp_rx.size(); i++)
	{
		sum1 += (v_win_fp_rx[i]-mean)* (v_win_fp_rx[i]-mean);
	}
	double std =   sqrt(sum1/(double)(v_win_fp_rx.size()));
	// 1)
	// dist = 0.122288 + 1.0013*dist;
	if(fp >= 5 || std>2)
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
		g2o::SE3Quat tag_se3 = computePoseToG2oSE3(now_odom.tagPose);
		g2o::SE3Quat anchor_se3 = computePoseToG2oSE3(now_odom.anchorPose);
		g2o::SE3Quat rel_se3 = anchor_se3.inverse() * tag_se3;
		rel_ = computeG2oSE3ToPose(rel_se3);
		dist_pre = sqrt((rel_.x * rel_.x) + (rel_.y * rel_.y));
	}
	else if(est_rel.timestamp == 777)
	{   
		g2o::SE3Quat rel_se3 = computePoseToG2oSE3(est_rel);
		g2o::SE3Quat lag_rel_odom_se3 = computePoseToG2oSE3(m_old_odom_of_uwb[name].anchorPose).inverse() * computePoseToG2oSE3(m_old_odom_of_uwb[name].tagPose);
		g2o::SE3Quat rel_odom_se3 = computePoseToG2oSE3(now_odom.anchorPose).inverse() * computePoseToG2oSE3(now_odom.tagPose);
		g2o::SE3Quat T3 =  rel_odom_se3 * lag_rel_odom_se3.inverse();
		g2o::SE3Quat now_T = T3 * rel_se3 ;
		rel_ = computeG2oSE3ToPose(now_T);
		dist_pre = sqrt((rel_.x * rel_.x) + (rel_.y * rel_.y));
	}
	m_old_odom_of_uwb[name] = now_odom;
	if(rx_fp < 5)
	{
		double range = 0.122288 + 1.00113 * uwb_message.range;
		uwb_message.range = range;
	}
	uwb_message.range = kf->Kalman_filters(uwb_message.range, NLOS_flag, dist_pre);			
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
void prepareOptimationData(std::string anchor_name, std::string tag_1_name, std::string tag_2_name, double time, std::map<int, UWBMessage> m_node,  
						   KalmanFilter *kf1, KalmanFilter *kf2,
						   boost::circular_buffer<UWBMessage> &v_robot_uwb_measure1,
						   boost::circular_buffer<UWBMessage> &v_robot_uwb_measure2)
{
	std::map<int, UWBMessage>::iterator it_node;
	for(it_node=m_node.begin(); it_node!=m_node.end(); it_node++)
	{
		if(it_node->first == m_uwb_configure[tag_1_name].uwb_id)
		{
			UWBMessage uwb;
			uwb.range = it_node->second.range;
			uwb.fpRssi = it_node->second.fpRssi;
			uwb.rxRssi = it_node->second.rxRssi;
			m_v_win_uwb[anchor_name+tag_1_name].push_back(uwb);
		}
		else if(it_node->first == m_uwb_configure[tag_2_name].uwb_id)
		{
			UWBMessage uwb;
			uwb.range = it_node->second.range;
			uwb.fpRssi = it_node->second.fpRssi;
			uwb.rxRssi = it_node->second.rxRssi;
			m_v_win_uwb[anchor_name+tag_2_name].push_back(uwb);
		}
	}

	ros::Time begin = ros::Time::now();
	if(m_v_win_uwb.find(anchor_name+tag_1_name) != m_v_win_uwb.end())
	{	
		if(m_v_odom.find(tag_1_name) == m_v_odom.end())
			return;
		CommonPose now_odom;
		if(timeSynchronization(anchor_name, tag_1_name, now_odom))
		{
			double delta_anchor_dist = now_odom.anchorPose.accumulated_dist - old_odom_anchor_acc_dist;
			double delta_tag_dist = now_odom.tagPose.accumulated_dist - old_odom_tag_acc_dist;
			double delta_anchor_angle = now_odom.anchorPose.accumulated_orientation - old_odom_anchor_acc_angle;
			double delta_tag_angle = now_odom.tagPose.accumulated_orientation - old_odom_tag_acc_angle;
			if(!(delta_anchor_dist > 0.02 || delta_anchor_angle > 0.02 || delta_tag_dist > 0.02 || delta_tag_angle > 0.02))
				return;
			old_odom_anchor_acc_dist = now_odom.anchorPose.accumulated_dist;
			old_odom_anchor_acc_angle = now_odom.anchorPose.accumulated_orientation;
			old_odom_tag_acc_dist = now_odom.tagPose.accumulated_dist;
			old_odom_tag_acc_angle = now_odom.tagPose.accumulated_orientation;
		}
		else
			return;

		int cnt1 = m_v_win_uwb[anchor_name+tag_1_name].size();	
		UWBMessage sum;
		for(int i=0; i<cnt1; i++)
		{
			sum.range += m_v_win_uwb[anchor_name+tag_1_name][i].range;
			sum.fpRssi += m_v_win_uwb[anchor_name+tag_1_name][i].fpRssi;
			sum.rxRssi += m_v_win_uwb[anchor_name+tag_1_name][i].rxRssi;
		}
		m_v_win_uwb[anchor_name+tag_1_name].clear();
		UWBMessage uwb_message;
		uwb_message.timestamp = m_v_odom[anchor_name].back().timestamp;
		uwb_message.range = sum.range/(double)(cnt1);
		uwb_message.fpRssi = sum.fpRssi/(double)(cnt1);
		uwb_message.rxRssi = sum.rxRssi/(double)(cnt1);
		
			// now_odom.timeStamp = time;
			// now_odom.anchorPose = m_v_odom[anchor_name].back();
			// now_odom.tagPose = m_v_odom[tag_1_name].back();
			RobotPose est_rel;
			std::map<std::string, RobotPose>::iterator it_est = m_est_rel.find(anchor_name+tag_1_name);
			if(it_est == m_est_rel.end())
				est_rel.timestamp = 666;
			else
			{
				est_rel =  m_est_rel[anchor_name+tag_1_name];
				est_rel.timestamp = 777;
			}
			// uwbKalmanFilter(uwb_message, now_odom, est_rel, kf1, anchor_name+tag_1_name);
			
			m_v_com_odom[anchor_name+tag_1_name].push_back(now_odom);
			v_robot_uwb_measure1.push_back(uwb_message);
			// std::ofstream com_odom(com_odom_file, std::ios::app);
			// // cout << "anchor_timestamp: " << to_string(now_odom.timeStamp)  << "\t tag_timestamp: " << to_string(now_odom.tagPose.timestamp)  << "\tuwb_timestamp: " << to_string(time) << endl;
			cout << anchor_name+tag_1_name  << ":\t" << m_v_com_odom[anchor_name+tag_1_name].size() << "\t" << v_robot_uwb_measure1.size() << "\t" << endl;
			int total_num = m_v_com_odom[anchor_name+tag_1_name].size();
			RobotPose relative_pose;
			ros::Time begin_g2o = ros::Time::now();

			if(total_num > TIME_WINDOW_SIZE)
			{
				relative_pose = nonlinearOptimization_process(total_num, anchor_name, tag_1_name, m_v_com_odom[anchor_name+tag_1_name], v_robot_uwb_measure1);
				if(anchor_name+tag_1_name=="robot1robot2")
				{
					nav_msgs::Odometry rel_pose;
					rel_pose.pose.pose.orientation.x = time;
					rel_pose.pose.pose.position.x = relative_pose.x;
					rel_pose.pose.pose.position.y = relative_pose.y;
					rel_pose.pose.pose.position.z= relative_pose.yaw;
					robot2_pub.publish(rel_pose);
				}
			}
			ros::Time end = ros::Time::now();
			double time_cast = end.toSec() - begin.toSec();
			double g2o_time_cast = end.toSec() - begin_g2o.toSec();
			cout << "time_cast: " << time_cast << "\t" << g2o_time_cast << endl; 
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

	
	prepareOptimationData("robot1", "robot2", "robot3", time, m_node, kf12, kf13, v_robot12_uwb_measure, v_robot13_uwb_measure);

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

	prepareOptimationData("robot2", "robot1", "robot3", time, m_node, kf21, kf23, v_robot21_uwb_measure, v_robot23_uwb_measure);
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

	prepareOptimationData("robot3", "robot1", "robot2", time, m_node, kf31, kf32, v_robot31_uwb_measure, v_robot32_uwb_measure);
}
//------------------------------------------------------------------------

void robot1OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	// if(m_v_amcl["robot1"].size() == 0) return;
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	static int once_flag0 = 0;
	pose.timestamp = timestamp;
	if(once_flag0 == 0)
	{
		once_flag0 = 1;
		robot1_current_odom.x = 0;//m_v_amcl["robot1"][0].x;
		robot1_current_odom.y = 0;//m_v_amcl["robot1"][0].y;
		robot1_current_odom.yaw =  0;//m_v_amcl["robot1"][0].yaw;
		pose.x=robot1_current_odom.x;
		pose.y=robot1_current_odom.y;
		pose.yaw=robot1_current_odom.yaw; 
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

		pose.timestamp = timestamp;
		pose.x = robot1_current_odom.x;
		pose.y = robot1_current_odom.y;
		pose.yaw = robot1_current_odom.yaw;
	}
	m_v_odom["robot1"].push_back(pose);
	// plotODOM();
	// RobotPose pose1;
	// pose1.x = 0;
	// pose1.y = 0;
	// pose1.yaw = 0;
	// pose1.timestamp = pose.timestamp;
	// pose1.uwbCovariance = 0;
	// pose1.accumulated_dist = 0;
	// pose1.accumulated_orientation = 0;
	// m_v_odom["robot2"].push_back(pose1);
	std::ofstream odom_file(odom1_file_name, std::ios::app);
	odom_file << "drone2" << "\t" << to_string(pose.timestamp) << "\t" << to_string(pose.x) << "\t" << to_string(pose.y) << "\t" << to_string(pose.yaw) << "\n";
	odom_file.close();
	std::ofstream odom_file1(odom_measure_name, std::ios::app);
	odom_file1 << "robot1" << "\t" << to_string(pose.timestamp) << "\t" << to_string(pose.x) << "\t" << to_string(pose.y) << "\t" << to_string(pose.yaw) << "\n";
	        //    << "robot2" << "\t" << to_string(pose1.timestamp) << "\t" << to_string(pose1.x) << "\t" << to_string(pose1.y) << "\t" << to_string(pose1.yaw) << "\n";
	odom_file1.close();
}
//------------------------------------------------------------------------
void robot2OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	// if(m_v_amcl["robot2"].size() == 0) return;
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	pose.timestamp = timestamp;

	static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
		robot2_current_odom.x = 0;//m_v_amcl["robot2"][0].x;
		robot2_current_odom.y = 0;//m_v_amcl["robot2"][0].y;
		robot2_current_odom.yaw =  0;//m_v_amcl["robot2"][0].yaw;
		pose.x=robot2_current_odom.x;
		pose.y=robot2_current_odom.y;
		pose.yaw=robot2_current_odom.yaw; 
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
		// std::ofstream file2(test_file2_name, std::ios::app);
		// file2 << "\t" << to_string(covariance2.determinant()) << endl;
		// file2.close();
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

		pose.timestamp = timestamp;
		pose.x = robot2_current_odom.x;
		pose.y = robot2_current_odom.y;
		pose.yaw = robot2_current_odom.yaw;
	}
	m_v_odom["robot2"].push_back(pose);
	std::ofstream odom_file(odom2_file_name, std::ios::app);
	odom_file << "\t" << to_string(pose.timestamp) << "\t" << to_string(pose.x) << "\t" << to_string(pose.y) << "\t" << to_string(pose.yaw) << "\n";
	odom_file.close();
	std::ofstream odom_file1(odom_measure_name, std::ios::app);
	odom_file1 << "robot2" << "\t" << to_string(pose.timestamp) << "\t" << to_string(pose.x) << "\t" << to_string(pose.y) << "\t" << to_string(pose.yaw) << "\n";
	        //    << "robot2" << "\t" << to_string(pose1.timestamp) << "\t" << to_string(pose1.x) << "\t" << to_string(pose1.y) << "\t" << to_string(pose1.yaw) << "\n";
	odom_file1.close();
}
//------------------------------------------------------------------------
void robot3OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(m_v_amcl["robot3"].size() == 0) return;
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose pose;
	pose.timestamp = timestamp;

	static int once_flag1 = 0;
	if(once_flag1 == 0)
	{
		once_flag1 = 1;
		robot2_current_odom.x = m_v_amcl["robot3"][0].x;
		robot2_current_odom.y = m_v_amcl["robot3"][0].y;
		robot2_current_odom.yaw =  m_v_amcl["robot3"][0].yaw;
		pose.x=robot2_current_odom.x;
		pose.y=robot2_current_odom.y;
		pose.yaw=robot2_current_odom.yaw; 
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
		// std::ofstream file2(test_file2_name, std::ios::app);
		// file2 << "\t" << to_string(covariance2.determinant()) << endl;
		// file2.close();
		pose.uwbCovariance = covariance3.determinant();
		//===================

		double moving_theta=fabs(carmen_normalize_theta(robot3_previous_odom.yaw-yaw));

		robot3_acc_distance=robot2_acc_distance+sigma_trans;		
		robot3_acc_angle=robot2_acc_angle+moving_theta;

		pose.accumulated_dist=robot3_acc_distance;
		pose.accumulated_orientation=robot3_acc_angle;

		robot3_previous_odom.x=msg->pose.pose.position.x;
		robot3_previous_odom.y=msg->pose.pose.position.y;
		robot3_previous_odom.yaw=yaw;

		robot3_current_odom.x = robot3_current_odom.x + sigma_trans*cos(robot3_current_odom.yaw);
		robot3_current_odom.y = robot3_current_odom.y + sigma_trans*sin(robot3_current_odom.yaw);
		robot3_current_odom.yaw = robot3_current_odom.yaw + actual_delta_theta;
		robot3_current_odom.yaw = carmen_normalize_theta(robot3_current_odom.yaw);

		pose.timestamp = timestamp;
		pose.x = robot3_current_odom.x;
		pose.y = robot3_current_odom.y;
		pose.yaw = robot3_current_odom.yaw;
	}
	m_v_odom["robot3"].push_back(pose);
}
//------------------------------------------------------------------------

void robot1AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	// double timestamp = msg->header.stamp.toSec();
	double timestamp = fabs(msg->header.stamp.toSec() - firstTimestamp);
	static int flag = 0;
	
    // if(flag==1)
    // {
    //     firstTimestamp = timestamp;
    //     timestamp = 0;
    //    flag = 2;
    // }
	
	if(!flag) {flag = 1; return;}
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);	
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	RobotPose robot_gt;
    robot_gt.x=msg->pose.pose.position.x;
	robot_gt.y=msg->pose.pose.position.y;	
	robot_gt.yaw=yaw;
    robot_gt.timestamp = timestamp;
	m_v_amcl["robot1"].push_back(robot_gt);
	if(m_v_amcl.find("robot2") != m_v_amcl.end())
	{
		RobotPose relative_pose = computeG2oSE3ToPose(computePoseToG2oSE3(robot_gt).inverse() * computePoseToG2oSE3(m_v_amcl["robot2"].back()));
		nav_msgs::Odometry rel_pose;
		rel_pose.pose.pose.orientation.x = timestamp;
		rel_pose.pose.pose.position.x = relative_pose.x;
		rel_pose.pose.pose.position.y = relative_pose.y;
		rel_pose.pose.pose.position.z = relative_pose.yaw;
		robot2_pub.publish(rel_pose);
	}
		// plotAMCL();
	// RobotPose pose1;
	// pose1.x = 0;
	// pose1.y = 0;
	// pose1.yaw = 0;
	// pose1.timestamp = timestamp;
	// m_v_amcl["robot2"].push_back(pose1);
	std::ofstream amcl_file(amcl1_file_name, std::ios::app);
	amcl_file << "drone1" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	amcl_file.close();
	std::ofstream amcl_file1(amcl_measure_name , std::ios::app);
	amcl_file1 << "robot1" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	        //    << "robot2" << "\t" << to_string(pose1.timestamp) << "\t" << to_string(pose1.x) << "\t" << to_string(pose1.y) << "\t" << to_string(pose1.yaw) << "\n";
	amcl_file1.close();
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

	if(m_v_amcl.find("robot1") != m_v_amcl.end())
	{
		RobotPose relative_pose = computeG2oSE3ToPose((computePoseToG2oSE3(robot_gt).inverse() * computePoseToG2oSE3(m_v_amcl["robot1"].back())).inverse());
		nav_msgs::Odometry rel_pose;
		rel_pose.pose.pose.orientation.x = timestamp;
		rel_pose.pose.pose.position.x = relative_pose.x;
		rel_pose.pose.pose.position.y = relative_pose.y;
		rel_pose.pose.pose.position.z = relative_pose.yaw;
		robot2_pub.publish(rel_pose);
	}
	std::ofstream amcl_file1(amcl_measure_name , std::ios::app);
	amcl_file1 << "robot2" << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	        //    << "robot2" << "\t" << to_string(pose1.timestamp) << "\t" << to_string(pose1.x) << "\t" << to_string(pose1.y) << "\t" << to_string(pose1.yaw) << "\n";
	amcl_file1.close();

	std::ofstream amcl_file(amcl2_file_name, std::ios::app);
	amcl_file << "\t" << to_string(robot_gt.timestamp) << "\t" << to_string(robot_gt.x) << "\t" << to_string(robot_gt.y) << "\t" << to_string(robot_gt.yaw) << "\n";
	amcl_file.close();
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
		sleep(1);
		// compute uwb error relative m_v_amcl m_v_rel_uwb
		std::map<std::string, std::vector<RobotPose> >::iterator it_amcl;
		it_amcl = m_v_amcl.find("robot1");
		if(it_amcl == m_v_amcl.end() || m_v_amcl.find("robot2") == m_v_amcl.end() || m_v_rel_uwb.find("robot1robot2") == m_v_rel_uwb.end())
			continue;
		std::vector<RobotPose> v_robot1_amcl = it_amcl->second;
		std::vector<double> v_robot12_pose_error;
		std::vector<double> v_robot12_yaw_error;	
		std::vector<double> v_robot21_pose_error;
		std::vector<double> v_robot21_yaw_error;
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
			if(record_j < 0 || min_time > 0.1)
				continue;
			CommonPose amcl;
			amcl.anchorPose = v_robot1_amcl[i];
			amcl.tagPose = v_robot2_amcl[record_j];

			g2o::SE3Quat anchor_se3 = computePoseToG2oSE3(amcl.anchorPose);
			g2o::SE3Quat tag_se3 = computePoseToG2oSE3(amcl.tagPose);
			g2o::SE3Quat rel_amcl_se3 = anchor_se3.inverse() * tag_se3;
			RobotPose rel_amcl = computeG2oSE3ToPose(rel_amcl_se3);
			
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
				if(record_j < 0 || min_time > 0.1)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];

				g2o::SE3Quat error_se3 = rel_amcl_se3.inverse() * computePoseToG2oSE3(relative_pose);
				RobotPose error = computeG2oSE3ToPose(error_se3);

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
				if(record_j < 0 || min_time > 0.1)
					continue;
				RobotPose relative_pose = v_rel_uwb[record_j];
				g2o::SE3Quat error_se3 = rel_amcl_se3 * computePoseToG2oSE3(relative_pose);
				RobotPose error = computeG2oSE3ToPose(error_se3);

				// cout << rel_amcl1.yaw << endl;
				double pose_error = sqrt(error.x*error.x + error.y*error.y);
				double yaw_error = fabs(error.yaw);
				
				v_robot21_pose_error.push_back(pose_error);
				v_robot21_yaw_error.push_back(yaw_error);
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
int main(int argc, char *argv[]){
	generate_time_based_random_seed();
	std::ofstream file1(odom1_file_name, std::ios::out);
	std::ofstream file2(odom2_file_name, std::ios::out);
	std::ofstream file3(amcl1_file_name, std::ios::out);
	std::ofstream file4(amcl2_file_name, std::ios::out);
	std::ofstream file5(uwb_measure_name, std::ios::out);
	std::ofstream file6(amcl_measure_name, std::ios::out);
	std::ofstream file7(odom_measure_name, std::ios::out);
	std::ofstream file8(test_file_name, std::ios::out);
	std::ofstream filea(com_trues_dist_file, std::ios::out);
	std::ofstream fileb(com_uwb_measure_file, std::ios::out);
	std::ofstream filec(com_odom_file, std::ios::out);
	std::ofstream filed(com_amcl_file, std::ios::out);
	// generate_time_based_random_seed();
	
	covariance1 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	covariance2 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	covariance3 << 1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9;
	ros::init(argc, argv, "relative_localization_node");

	ros::NodeHandle n("~");

	std::string configure_file;

	n.getParam("configure", configure_file);
	std::cout<<configure_file.c_str()<<std::endl;

	loadConfigureFile(configure_file);
	// loadAMCLFile(gt_file_name);
	// std::cout << m_v_amcl1["robot1"].size() << "\t" << m_v_amcl1["robot2"].size() << std::endl;
	// ros::Subscriber sub_robot1_amcl = n.subscribe("/amcl_pose", 1000, robot1AMCLCallback);
	ros::Subscriber sub_robot1_amcl = n.subscribe("/robot1/amcl_pose", 1000, robot1AMCLCallback);
	ros::Subscriber sub_robot2_amcl = n.subscribe("/robot2/amcl_pose", 1000, robot2AMCLCallback);
	ros::Subscriber sub_robot3_amcl = n.subscribe("/robot3/amcl_pose", 1000, robot3AMCLCallback);
	

	ros::Subscriber sub_uwb_robot1 = n.subscribe("/robot1/nlink_linktrack_nodeframe2", 1000, robot1UWBCallback);
	// ros::Subscriber sub_uwb_robot2 = n.subscribe("/robot2/nlink_linktrack_nodeframe2", 1000, robot2UWBCallback);	
	ros::Subscriber sub_uwb_robot3 = n.subscribe("/robot3/nlink_linktrack_nodeframe2", 1000, robot3UWBCallback);

	// ros::Subscriber sub_robot1_odom = n.subscribe("/odom", 1000, robot1OdomCallback);
	ros::Subscriber sub_robot1_odom = n.subscribe("/robot1/odom", 1000, robot1OdomCallback);
	ros::Subscriber sub_robot2_odom = n.subscribe("/robot2/odom", 1000, robot2OdomCallback);
	ros::Subscriber sub_robot3_odom = n.subscribe("/robot3/odom", 1000, robot3OdomCallback);
	
	robot2_pub = n.advertise<nav_msgs::Odometry>("/robot2/relative_pose", 1);
	// ros::Publisher robot3_pub = n.advertise<geometry_msgs::Point>("/robot3/relative_pose", 10000);
	
	pthread_t tid;
	int ret = pthread_create(&tid, NULL, computePositionError, NULL);
	ros::spin();
	// ros::Rate loop(50);
	// while(n.ok())
	// {
	// 	ros::spinOnce();
	// 	std::map<std::string, RobotPose>::iterator it_rel;
		
	// 	it_rel = m_rel_pose.find("robot1robot2");
	// 	if(it_rel != m_rel_pose.end())
	// 	{
	// 		geometry_msgs::Point rel_pose;
	// 		rel_pose.x = it_rel->second.x;
	// 		rel_pose.y = it_rel->second.y;
	// 		rel_pose.z = it_rel->second.yaw;
	// 		robot2_pub.publish(rel_pose);
	// 	}
	// 	it_rel = m_rel_pose.find("robot1robot3");
	// 	if(it_rel != m_rel_pose.end())
	// 	{
	// 		geometry_msgs::Point rel_pose;
	// 		rel_pose.x = it_rel->second.x;
	// 		rel_pose.y = it_rel->second.y;
	// 		rel_pose.z = it_rel->second.yaw;
	// 		robot3_pub.publish(rel_pose);
	// 	}
	// 	loop.sleep();
	// }
	return 0;
}
