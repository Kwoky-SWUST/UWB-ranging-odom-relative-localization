/*
 * Orca-Robotics Project: Components for robotics 
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2006 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef VFHDRIVER_H
#define VFHDRIVER_H
#include "vfh_algorithm.h"
#include "vfh_algorithmconfig.h"


namespace vfh {

class RobotVelocity{
public:
    RobotVelocity(){}
    double linear_x;
    double angular_z;
};

//
// Implements VFH.
// All the number-crunching is done by the VFH_Algorithm class.
// This thing just feeds in configuration and information, plus
// tries heuristic escape approaches if the robot gets stuck.
//
// @author Alex Brooks
//
class VfhDriver
{

public: 
    VfhDriver(std::string file_name);

    virtual ~VfhDriver();

    // Goal location is in robot's coordinate frame
    // virtual hydronavutil::Velocity getCommand( const IDriver::Inputs &inputs );
    geometry_msgs::Twist approachGoalCommand( double distanceTolerance,
                                    geometry_msgs::Twist &goal, 
                                    geometry_msgs::Twist &currentVelocity,
                                    std::vector<double>     &obsRanges );

    void setSpeedConstraints( float maxSpeed, float maxTurnrate );
private: 
    void loadVFHConfigFile(std::string file_name, VfhAlgorithmConfig &vfhConfig);
    
    // Copy to Player units
    void copyLaserScan( const std::vector<double> &obsRanges, double playerLaserScan[361][2] );

    // Class to handle the internal VFH algorithm
    // (like maintaining histograms etc).
    // This is the guy that does all the number-crunching.
    VFH_Algorithm *vfhAlgorithm_;

    // Range and bearing values in player units, to give to vfh engine
    // The '361' is what vfh_algorithm expects
    double playerLaserScan_[361][2];

    // speed constraints
    float maxSpeed_;
    float maxTurnrate_;

    // Configuration from file
    VfhAlgorithmConfig vfhConfig_;

};

} // namespace

#endif
