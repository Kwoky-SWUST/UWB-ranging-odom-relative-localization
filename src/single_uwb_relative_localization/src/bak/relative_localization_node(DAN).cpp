#include "relative_localization_node.h"

const int TIME_WINDOW_SIZE = 100;
const int TIME_GRAP = 5;
const int NUM_POINTS = TIME_WINDOW_SIZE / TIME_GRAP;
const int LAMDA = 20;
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
//-------------------------------------------------------------------------

Matrix3d covariance1;
Matrix3d covariance2;

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
void plotUWBRel(int minX=-8, int maxX=8, int minY=-10, int maxY=10)
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
void plotODOM(int minX=0, int maxX=10, int minY=0, int maxY=18)
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
// void plotComODOMAMCL(int minX=0, int maxX=10, int minY=0, int maxY=18)
// {
// 	std::string cmd;

// 	cmd+="set grid\n";
// 	cmd+="set xlabel 'times'\n";
// 	cmd+="set ylabel 'y(m)'\n";
// 	cmd+="set title 'ODOM and AMCL track'\n";
// 	cmd+="set size ratio -1\n";
// 	cmd+="unset arrow\n";
// 	cmd+="set key top right height 0.5 width -2\n";
// 	cmd+="plot ["+toString(minX)+':'+toString(maxX)+"]["+toString(minY)+':'+toString(maxY)+"] ";
	
//     cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#FF0000' ti 'AMCL2',";
//     cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#0000FF' ti 'AMCL1',";
//     cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#00FF00' ti 'ODOM2',";
//     cmd+="'-' u 1:2 w lp lw 2 pt 6 ps 1 lc rgb '#000000' ti 'ODOM1',";
//     cmd += "\n";

//     for(int i=0;i<v_com_amcl.size();i=i+1)
//     {
//         // if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
//         //     continue;
//         cmd += toString( v_com_amcl[i].anchorPose.x ) + ' ' + toString( v_com_amcl[i].anchorPose.y ) + ' ' + toString( 0.5 ) + '\n';
//     }
//     cmd += "e\n";

//     for(int i=0;i<v_com_amcl.size();i=i+1)
//     {
//         //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
//         //     continue;
//         cmd += toString( v_com_amcl[i].tagPose.x ) + ' ' + toString( v_com_amcl[i].tagPose.y ) + ' ' + toString( 0.5 ) + '\n';
//     }
//     cmd += "e\n";

//     for(int i=0;i<v_com_odom.size();i=i+1)
//     {
//         //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
//         //     continue;
//         cmd += toString( v_com_odom[i].anchorPose.x ) + ' ' + toString( v_com_odom[i].anchorPose.y ) + ' ' + toString( 0.5 ) + '\n';
//     }
//     cmd += "e\n";

//     for(int i=0;i<v_com_odom.size();i=i+1)
//     {
//         //  if(v_com_amcl[i].timeStamp < 200 || v_com_amcl[i].timeStamp > 240)
//         //     continue;
//         cmd += toString( v_com_odom[i].tagPose.x ) + ' ' + toString( v_com_odom[i].tagPose.y ) + ' ' + toString( 0.5 ) + '\n';
//     }
//     cmd += "e\n";
	
// 	plot_com_odom_amcl->commandStr( cmd );
// }
//------------------------------------------------------------------------
GnuplotInterface * plot_win = new GnuplotInterface();
std::map<std::string, std::vector<RobotPose> > m_v_win;
void plotWindow(int minX=0, int maxX=10, int minY=0, int maxY=18)
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
//========================================================================

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
double dx=0;
double dy=0;
std::map<std::string, int> m_flag1;

void nonlinearOptimization_process(int total_num, std::string anchor_name, std::string tag_name, std::vector<CommonPose> v_com_amcl, std::vector<CommonPose> v_com_odom, boost::circular_buffer<UWBMessage> v_robot_uwb_measure)
{  
	valid_count++;
	std::vector<G2OMeasure> v_g2o_measure;
	CommonPose now_odom = v_com_odom[total_num-1];
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
		// 1.4)) compute weight
		double delta_d = ((now_odom.tagPose.accumulated_dist  - lag_odom.tagPose.accumulated_dist) + (now_odom.anchorPose.accumulated_dist - lag_odom.anchorPose.accumulated_dist))/2.0;
		double delta_a = ((now_odom.tagPose.accumulated_orientation  - lag_odom.tagPose.accumulated_orientation) + (now_odom.anchorPose.accumulated_orientation - lag_odom.anchorPose.accumulated_orientation))/2.0;
		double delta_da = delta_d * delta_a;
		
		double weight = exp(-(delta_da)/(2.0*LAMDA));
		cout << weight << endl;
		// cout << cov << "---" << delta_da << endl;
		g2o_measure.weight = 1;
		// 2) save uwb delta pose and distance
		g2o_measure.anchor_delta_x = 0;
		g2o_measure.anchor_delta_y = 0;
		g2o_measure.tag_delta_x = 0;
		g2o_measure.tag_delta_y = 0;
		g2o_measure.distance = v_robot_uwb_measure[i].range;

		v_g2o_measure.push_back(g2o_measure);

		RobotPose anchor_ = lag_anchor_odom;
		RobotPose tag_ = lag_tag_odom;
		m_v_win[tag_name].push_back(anchor_);
		m_v_win[anchor_name].push_back(tag_);
	}
	plotWindow();
	m_v_win[tag_name].clear();
	m_v_win[anchor_name].clear();
	// 3) localization based g2o  
	CommonPose amcl_pose =  v_com_amcl[total_num-1];  
	g2o::SE3Quat tag_amcl_se3 = computePoseToG2oSE3(amcl_pose.tagPose);
	g2o::SE3Quat anchor_amcl_se3 = computePoseToG2oSE3(amcl_pose.anchorPose);
	g2o::SE3Quat relative_amcl_se3 = anchor_amcl_se3.inverse() * tag_amcl_se3;
	RobotPose AMCL = computeG2oSE3ToPose(relative_amcl_se3);
	
	CommonPose odom_pose =  v_com_odom[total_num-1];  
	g2o::SE3Quat tag_o_se3 = computePoseToG2oSE3(odom_pose.tagPose);
	g2o::SE3Quat anchor_o_se3 = computePoseToG2oSE3(odom_pose.anchorPose);
	g2o::SE3Quat relative_o_se3 = anchor_o_se3.inverse() * tag_o_se3;
	
	RobotPose init;
	RobotPose relative_pose;
	// std::map<std::string, int>::iterator it_flag = m_flag1.find(anchor_name+tag_name);
	// if(it_flag == m_flag1.end())
	// 	m_flag1[anchor_name+tag_name] = 0;
	static int flag1=0;
	double dist1 = v_robot12_uwb_measure[total_num - 1].range;
	
	// if(m_flag1[anchor_name+tag_name] == 0)
	// {
	// 	m_flag1[anchor_name+tag_name] = 1;
	//     init=computeG2oSE3ToPose(relative_o_se3);
	// }
	// if(flag1 == 0)
	// {
	// 	flag1 = 1;
	// 	init=computeG2oSE3ToPose(relative_o_se3);
	// }
	// else
	// {
	// 	g2o::SE3Quat old_odom_se3 = computePoseToG2oSE3(old_odom);
	// 	g2o::SE3Quat o_se3 = old_odom_se3.inverse() * relative_o_se3;
	// 	g2o::SE3Quat old_rel_se3 = computePoseToG2oSE3(old_rel);
	// 	g2o::SE3Quat init_se3 = old_rel_se3 * o_se3;
	// 	init = computeG2oSE3ToPose(init_se3);	
		
	// 	// get distance
	// 	g2o::SE3Quat T2 = computePoseToG2oSE3(old_odom_pose.anchorPose).inverse() * computePoseToG2oSE3(odom_pose.anchorPose);
	// 	g2o::SE3Quat T1 = computePoseToG2oSE3(old_odom_pose.tagPose).inverse() * computePoseToG2oSE3(odom_pose.tagPose); 	
	// 	g2o::SE3Quat now_pose_se3 = T2.inverse() * (old_rel_se3 * T1);
	// 	// cout << T2.to_homogeneous_matrix() << "\n" 
	// 	// 	 << (T2.inverse()).to_homogeneous_matrix() << "\n"
	// 	// 	 << old_rel_se3.to_homogeneous_matrix() << "\n" 
	// 	// 	 << T1.to_homogeneous_matrix() << endl;
	// 	// while(1);
	// 	RobotPose now_pose = computeG2oSE3ToPose(now_pose_se3);

	// 	double dist2 = sqrt((init.x*init.x) + (init.y*init.y));
	// 	double dist3 = sqrt((now_pose.x*now_pose.x) + (now_pose.y*now_pose.y));
	// 	double dist4 = sqrt((AMCL.x*AMCL.x) + (AMCL.y*AMCL.y));
	// 	double dist1_e = fabs(dist1 - dist4);
	// 	double dist2_e = fabs(dist2 - dist4);
	// 	double dist3_e = fabs(dist3 - dist4);
	// 	std::ofstream test(test_file_name, std::ios::app);
	// 	//  << init.x << "\t" << init.y << "\t" << init.yaw << "\t" << now_pose.x << "\t" << now_pose.y << "\t" << now_pose.yaw << "\t"
	// 	// 	 << AMCL.x << "\t" << AMCL.y << "\t" << AMCL.yaw << "\n"
	// 	test << "MEA: " << dist1 << "\test1:" << dist2 << "\test2:" << dist3 << "\ttrue:" << dist4 << "\n"
	// 		 << "error1:"<< dist1_e << "\terror2:" << dist2_e << "\terror3:" << dist3_e << "\n"
	// 		 << "----";
	// 	test.close();
	// 	init = now_pose;
	// 	v_error1.push_back(dist1_e);
	// 	v_error2.push_back(dist2_e);
	// 	v_error3.push_back(dist3_e);
	// 	// getchar();	
	// }
	init  = AMCL;
	ros::Time begin_g2o = ros::Time::now();
	relative_pose = g2o_process(v_g2o_measure, v_g2o_measure.size(), 30, init);
	ros::Time end_g2o = ros::Time::now();
	old_rel = relative_pose;
	m_est_rel[anchor_name+tag_name] = relative_pose;
	old_odom = computeG2oSE3ToPose(relative_o_se3);
	old_odom_pose = odom_pose;
	// 4) compute position error
	g2o::SE3Quat rel_se3 = computePoseToG2oSE3(relative_pose);
	g2o::SE3Quat amcl_se3 = computePoseToG2oSE3(amcl_pose.anchorPose);
	g2o::SE3Quat glo_se3 = amcl_se3 * rel_se3;
	RobotPose glo_ = computeG2oSE3ToPose(glo_se3);
	m_v_uwb_rel[tag_name].push_back(glo_);

	g2o::SE3Quat error_odom_se3 = relative_amcl_se3.inverse() * relative_o_se3;
	RobotPose error_odom = computeG2oSE3ToPose(error_odom_se3);
	double position_error_odom = sqrt(error_odom.x*error_odom.x + error_odom.y*error_odom.y);
	double yaw_error_odom = fabs(carmen_normalize_theta(error_odom.yaw)) * 180 / 3.14;
	v_odom_pose_error.push_back(position_error_odom);
	v_odom_yaw_error.push_back(yaw_error_odom);

	g2o::SE3Quat relative_pose_se3 = computePoseToG2oSE3(relative_pose);
	g2o::SE3Quat error_se3 = relative_amcl_se3.inverse() * relative_pose_se3;
	RobotPose error = computeG2oSE3ToPose(error_se3);
	double position_error = sqrt(error.x*error.x + error.y*error.y);
	double yaw_error = fabs(error.yaw) * 180 / 3.14;
	v_pose_error.push_back(position_error);
	v_yaw_error.push_back(yaw_error);
	
	if(position_error > 1.0)
	{
		double max_error_time = v_com_odom[total_num-1].timeStamp;
		v_max_error_time.push_back(max_error_time);
		// cout << "---" << max_error_time << endl;
	} 
	
	double pose_odom_mean = 0;
    double pose_odom_std = 0;
    double yaw_odom_mean = 0;
    double yaw_odom_std = 0;
    computeMeanVariance(v_odom_pose_error, pose_odom_mean, pose_odom_std);
    computeMeanVariance(v_odom_yaw_error, yaw_odom_mean, yaw_odom_std);

    double pose_mean = 0;
    double pose_std = 0;
    double yaw_mean = 0;
    double yaw_std = 0;
    computeMeanVariance(v_pose_error, pose_mean, pose_std);
    computeMeanVariance(v_yaw_error, yaw_mean, yaw_std);

	double error1_mean = 0;
    double error1_std = 0;
	double error2_mean = 0;
    double error2_std = 0;
	double error3_mean = 0;
    double error3_std = 0;

    computeMeanVariance(v_error1, error1_mean, error1_std);
    computeMeanVariance(v_error2, error2_mean, error2_std);
	computeMeanVariance(v_error3, error3_mean, error3_std);

    cout << "position_error: " << pose_mean  << "\t " << pose_std << "\n" 
         << "yaw_error: " << yaw_mean << "\t" << yaw_std << "\n" 
		 << "position_odom_error: " << pose_odom_mean  << "\t " << pose_odom_std << "\n" 
         << "yaw_odom_error: " << yaw_odom_mean << "\t" << yaw_odom_std << endl;
	cout << "error1: " << error1_mean << "\t" << error1_std << "\n"
	     << "error2: " << error2_mean << "\t" << error2_std << "\n"
		 << "error3: " << error3_mean << "\t" << error3_std << "\n";
    cout << "total:" << valid_count << "\terror:" <<  v_max_error_time.size() << endl;
}
//------------------------------------------------------------------------
void poseGraph_process()
{

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
const double time_limit = 1.0;
std::map<std::string, std::vector<UWBMessage> > m_v_win_uwb;
std::map<std::string, int > m_flag;
std::map<std::string, IndexNum> m_index_record;
bool timeSynchronization(std::string anchor_name, std::string tag_name, CommonPose &now_amcl, CommonPose &now_odom)
{
	std::map<std::string, IndexNum>::iterator it_index = m_index_record.find(anchor_name+tag_name);
	if(it_index == m_index_record.end())
		m_index_record[anchor_name+tag_name].index1 = m_index_record[anchor_name+tag_name].index2 = m_index_record[anchor_name+tag_name].index3 = 0;
	double timestamp = m_v_amcl[anchor_name].back().timestamp;
	double time_min_diff = INFINITY;
	int record_i=-1;
	for(int i=m_index_record[anchor_name+tag_name].index1; i<m_v_amcl[tag_name].size(); i++)
	{
		double time_diff = fabs(timestamp - m_v_amcl[tag_name][i].timestamp);
		if(time_diff < time_min_diff)
		{
			time_min_diff = time_diff;
			record_i  = i;
		}
	}
	if(time_min_diff > time_limit||record_i < 0)
		return false;	
	now_amcl.timeStamp = timestamp;
	now_amcl.anchorPose =  m_v_amcl[anchor_name].back();
	now_amcl.tagPose = m_v_amcl[tag_name][record_i];
	m_index_record[anchor_name+tag_name].index1 = record_i;

	time_min_diff = INFINITY;
	record_i=-1;
	for(int i=m_index_record[anchor_name+tag_name].index2; i<m_v_odom[anchor_name].size(); i++)
	{
		double time_diff = fabs(timestamp - m_v_odom[anchor_name][i].timestamp);
		if(time_diff < time_min_diff)
		{
			time_min_diff = time_diff;
			record_i  = i;
		}
	}
	if(time_min_diff > time_limit||record_i < 0)
		return false;
	now_odom.anchorPose = m_v_odom[anchor_name][record_i];
	m_index_record[anchor_name+tag_name].index2 = record_i;

	time_min_diff = INFINITY;
	record_i=-1;
	for(int i=m_index_record[anchor_name+tag_name].index3; i<m_v_odom[tag_name].size(); i++)
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
	now_odom.tagPose = m_v_odom[tag_name][record_i];
	now_odom.timeStamp = timestamp;
	m_index_record[anchor_name+tag_name].index3 = record_i;
	
	return true;
}

void uwbKalmanFilter(UWBMessage &uwb_message, CommonPose now_odom, RobotPose est_rel, KalmanFilter *kf)
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
	static CommonPose old_odom;
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
		g2o::SE3Quat lag_rel_odom_se3 = computePoseToG2oSE3(old_odom.anchorPose).inverse() * computePoseToG2oSE3(old_odom.tagPose);
		g2o::SE3Quat rel_odom_se3 = computePoseToG2oSE3(now_odom.anchorPose).inverse() * computePoseToG2oSE3(now_odom.tagPose);
		g2o::SE3Quat T3 =  rel_odom_se3 * lag_rel_odom_se3.inverse();
		g2o::SE3Quat now_T = T3 * rel_se3 ;
		rel_ = computeG2oSE3ToPose(now_T);
		dist_pre = sqrt((rel_.x * rel_.x) + (rel_.y * rel_.y));
	}
	old_odom = now_odom;
	if(rx_fp < 5)
	{
		double range = 0.122288 + 1.00113 * uwb_message.range;
		uwb_message.range = range;
	}
	uwb_message.range = kf->Kalman_filters(uwb_message.range, NLOS_flag, dist_pre);			
}

void prepareOptimationData(std::string anchor_name, std::string tag_1_name, std::string tag_2_name, double time, std::map<int, UWBMessage> m_node,  KalmanFilter *kf, boost::circular_buffer<UWBMessage> &v_robot_uwb_measure)
{
	if(fabs(m_v_amcl[anchor_name].back().timestamp  - time) < 0.1)
	{
		m_flag[anchor_name] = 1;
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
	}
	if(m_flag[anchor_name]==1 && fabs(m_v_amcl[anchor_name].back().timestamp  - time) >= 0.1)
	{
		m_flag[anchor_name]  = 0;
		int cnt1 = m_v_win_uwb[anchor_name+tag_1_name].size();
		ros::Time begin = ros::Time::now();
		if(cnt1 != 0)
		{
			UWBMessage sum;
			for(int i=0; i<cnt1; i++)
			{
				sum.range += m_v_win_uwb[anchor_name+tag_1_name][i].range;
				sum.fpRssi += m_v_win_uwb[anchor_name+tag_1_name][i].fpRssi;
				sum.rxRssi += m_v_win_uwb[anchor_name+tag_1_name][i].rxRssi;
			}
			m_v_win_uwb[anchor_name+tag_1_name].clear();
			UWBMessage uwb_message;
			uwb_message.timestamp = m_v_amcl[anchor_name].back().timestamp;
			uwb_message.range = sum.range/(double)(cnt1);
			uwb_message.fpRssi = sum.fpRssi/(double)(cnt1);
			uwb_message.rxRssi = sum.rxRssi/(double)(cnt1);

			CommonPose now_amcl;
			CommonPose now_odom;
			if(timeSynchronization(anchor_name, tag_1_name, now_amcl, now_odom))
			{
				RobotPose est_rel;
				std::map<std::string, RobotPose>::iterator it_est = m_est_rel.find(anchor_name+tag_1_name);
				if(it_est == m_est_rel.end())
					est_rel.timestamp = 666;
				else
				{
					est_rel =  m_est_rel[anchor_name+tag_1_name];
					est_rel.timestamp = 777;
				}

				// uwbKalmanFilter(uwb_message, now_odom, est_rel, kf);
				m_v_com_amcl[anchor_name+tag_1_name].push_back(now_amcl);
				m_v_com_odom[anchor_name+tag_1_name].push_back(now_odom);
				v_robot_uwb_measure.push_back(uwb_message);
				cout << anchor_name+tag_1_name  << ":\t" << m_v_com_odom[anchor_name+tag_1_name].size() << "\t" << m_v_com_amcl[anchor_name+tag_1_name].size() << "\t" << v_robot_uwb_measure.size() << "\t" << endl;
				int total_num = m_v_com_amcl[anchor_name+tag_1_name].size();
				ros::Time begin_g2o = ros::Time::now();
				if(total_num > TIME_WINDOW_SIZE)
				{
					nonlinearOptimization_process(total_num, anchor_name, tag_1_name, m_v_com_amcl[anchor_name+tag_1_name], m_v_com_odom[anchor_name+tag_1_name], v_robot12_uwb_measure);
				}
				ros::Time end = ros::Time::now();

				double true_dist = sqrt((now_amcl.anchorPose.x-now_amcl.tagPose.x)*(now_amcl.anchorPose.x-now_amcl.tagPose.x)
							+(now_amcl.anchorPose.y-now_amcl.tagPose.y)*(now_amcl.anchorPose.y-now_amcl.tagPose.y));
				v_trues_dist.push_back(true_dist);
				v_fx_rx.push_back(uwb_message.rxRssi-uwb_message.fpRssi);
				double uwb_error = fabs(true_dist - uwb_message.range);
				v_uwb_error.push_back(uwb_error);
				double uwb_mean = 0;
				double uwb_mean_std = 0;
				computeMeanVariance(v_uwb_error, uwb_mean, uwb_mean_std);
				cout << "uwb_error: " << uwb_mean << "\t" << uwb_mean_std << endl; 
				double time_cast = end.toSec() - begin.toSec();
				double g2o_time_cast = end.toSec() - begin_g2o.toSec();
				cout << "time_cast: " << time_cast << "\t" << g2o_time_cast << endl; 
			}
		}		
	}
}
//========================================================================
void robot1UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	if(m_v_amcl["robot1"].size() <= 0) return;
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

	
	// prepareOptimationData("robot1", "robot2", "robot3", time, m_node, kf12, v_robot12_uwb_measure);

	
	// plotComODOMAMCL();
	plotUWBMeasure();
	plotRx_fx();
	plotUWBRel();
}
//------------------------------------------------------------------------
void robot2UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	if(m_v_amcl["robot2"].size() <= 0) return;
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

	prepareOptimationData("robot2", "robot1", "robot3", time, m_node, kf21, v_robot21_uwb_measure);
}
//------------------------------------------------------------------------
void robot3UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
	double time = fabs(msg->sec + (double)(msg->usec)/1000000.0 - firstTimestamp);
}
//------------------------------------------------------------------------

void robot1OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(m_v_amcl["robot1"].size() == 0) return;
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
		robot1_current_odom.x = m_v_amcl["robot1"][0].x;
		robot1_current_odom.y = m_v_amcl["robot1"][0].y;
		robot1_current_odom.yaw =  m_v_amcl["robot1"][0].yaw;
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
	if(m_v_amcl["robot2"].size() == 0) return;
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
		robot2_current_odom.x = m_v_amcl["robot2"][0].x;
		robot2_current_odom.y = m_v_amcl["robot2"][0].y;
		robot2_current_odom.yaw =  m_v_amcl["robot2"][0].yaw;
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
	double timestamp=(msg->header.stamp.toSec()-firstTimestamp);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	RobotPose pose;
	pose.timestamp=timestamp;
	pose.x=msg->pose.pose.position.x;
	pose.y=msg->pose.pose.position.y;
	pose.yaw=yaw;
	m_v_odom["robot3"].push_back(pose);
}
//------------------------------------------------------------------------

void robot1AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	// double timestamp = msg->header.stamp.toSec();
	double timestamp = fabs(msg->header.stamp.toSec() - firstTimestamp);
	static int flag = 0;
	
    if(flag==1)
    {
        firstTimestamp = timestamp;
        timestamp = 0;
       flag = 2;
    }
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

	ros::init(argc, argv, "relative_localization_node");

	ros::NodeHandle n("~");

	std::string configure_file;

	n.getParam("configure", configure_file);
	std::cout<<configure_file.c_str()<<std::endl;

	loadConfigureFile(configure_file);
	
	ros::Subscriber sub_robot1_amcl = n.subscribe("/robot1/amcl_pose", 1000, robot1AMCLCallback);
	ros::Subscriber sub_robot2_amcl = n.subscribe("/robot2/amcl_pose", 1000, robot2AMCLCallback);
	ros::Subscriber sub_robot3_amcl = n.subscribe("/robot3/amcl_pose", 1000, robot3AMCLCallback);
	

	ros::Subscriber sub_uwb_robot1 = n.subscribe("/robot1/nlink_linktrack_nodeframe2", 10000, robot1UWBCallback);
	ros::Subscriber sub_uwb_robot2 = n.subscribe("/robot2/nlink_linktrack_nodeframe2", 10000, robot2UWBCallback);	
	ros::Subscriber sub_uwb_robot3 = n.subscribe("/robot3/nlink_linktrack_nodeframe2", 10000, robot3UWBCallback);


	ros::Subscriber sub_robot1_odom = n.subscribe("/robot1/odom", 1000, robot1OdomCallback);
	ros::Subscriber sub_robot2_odom = n.subscribe("/robot2/odom", 1000, robot2OdomCallback);
	ros::Subscriber sub_robot3_odom = n.subscribe("/robot3/odom", 1000, robot3OdomCallback);

	ros::spin();
	return 0;
}
