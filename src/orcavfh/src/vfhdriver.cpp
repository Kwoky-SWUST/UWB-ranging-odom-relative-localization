/*
 * Orca-Robotics Project: Components for robotics 
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2006 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include <iostream>
#include <cmath>
#include <string>
#include <geometry_msgs/Twist.h>
#include "vfhdriver.h"
#include "vfh_algorithm.h"
#include "vfh_algorithmconfig.h"
#include <sstream>
#include <fstream>
// using namespace localnav;
using namespace std;

namespace vfh {

    // need definition (in case its not inlined)
    VfhDriver::VfhDriver(std::string file_name)
    {
        // Configure and instantiate the core vfh algorithm
        loadVFHConfigFile(file_name, vfhConfig_ );

        std::cout<<"TRACE(vfhdriver.cpp): Instantiating VFH with: " << vfhConfig_ << std::endl;
        std::string warnings, errors;
        if ( vfhConfig_.checkSanity(warnings,errors) != 0 )
        {
            if ( warnings != "" )
            {
            std::cout << "Dodgy-looking VFH configuration: " + warnings << std::endl;
            }
            if ( errors != "" )
            {
            std::cout << "Erroneous VFH configuration: " + errors << std::endl;
            std::cout <<  "VfhDriver: Bad VFH config: " << errors << std::endl;
            }
        }

        vfhAlgorithm_ = new VFH_Algorithm( vfhConfig_ );
    }

    VfhDriver::~VfhDriver()
    {
        if ( vfhAlgorithm_ ) delete vfhAlgorithm_;
    }
    
    void 
    VfhDriver::loadVFHConfigFile(std::string file_name, vfh::VfhAlgorithmConfig &vfhConfig)
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
            iss_line >> invalid_word;
            if(invalid_word == "cellSize:")
                iss_line >> vfhConfig.cellSize;
            else if(invalid_word == "gridWidthInCells:")
                iss_line >> vfhConfig.gridWidthInCells;
            else if(invalid_word == "sectorAngle:")
                iss_line >> vfhConfig.sectorAngle;
            else if(invalid_word == "robotRadius:")
                iss_line >> vfhConfig.robotRadius;
            else if(invalid_word == "safetyDist0ms:")
                iss_line >> vfhConfig.safetyDist0ms;
            else if(invalid_word == "safetyDist1ms:")
                iss_line >> vfhConfig.safetyDist1ms;
            else if(invalid_word == "maxSpeed:")
                iss_line >> vfhConfig.maxSpeed;
            else if(invalid_word == "maxSpeedNarrowOpening:")
                iss_line >> vfhConfig.maxSpeedNarrowOpening;
            else if(invalid_word == "maxSpeedWideOpening:")
                iss_line >> vfhConfig.maxSpeedWideOpening;
            else if(invalid_word == "maxAcceleration:")
                iss_line >> vfhConfig.maxAcceleration;
            else if(invalid_word == "maxTurnrate0ms:")
                iss_line >> vfhConfig.maxTurnrate0ms;
            else if(invalid_word == "maxTurnrate1ms:")
                iss_line >> vfhConfig.maxTurnrate1ms;
            else if(invalid_word == "absoluteMaxTurnrate:")
                iss_line >> vfhConfig.absoluteMaxTurnrate;
            else if(invalid_word == "minTurnRadiusSafetyFactor:")
                iss_line >> vfhConfig.minTurnRadiusSafetyFactor;
            else if(invalid_word == "freeSpaceCutoff0ms:")
                iss_line >> vfhConfig.freeSpaceCutoff0ms;
            else if(invalid_word == "obsCutoff0ms:")
                iss_line >> vfhConfig.obsCutoff0ms;
            else if(invalid_word == "freeSpaceCutoff1ms:")
                iss_line >> vfhConfig.freeSpaceCutoff1ms;
            else if(invalid_word == "obsCutoff1ms:")
                iss_line >> vfhConfig.obsCutoff1ms;
            else if(invalid_word == "weightDesiredDir:")
                iss_line >> vfhConfig.weightDesiredDir;
            else if(invalid_word == "weightCurrentDir:")
                iss_line >> vfhConfig.weightCurrentDir;
            else{;}
        }
        file.close(); 
    }	
    
    void 
    VfhDriver::setSpeedConstraints( float maxSpeed, float maxTurnrate )
    {
        maxSpeed_ = maxSpeed;
        maxTurnrate_ = maxTurnrate;

        // Check that this doesn't exceed any of our hard-coded maximums
        if ( maxSpeed_ > vfhConfig_.maxSpeed )
        {
            std::cout << "VFH: requested maxSpeed ("
            <<maxSpeed_<<") faster than its configured maximum ("<<vfhConfig_.maxSpeed
            <<").  Thresholding.";
            maxSpeed_ = vfhConfig_.maxSpeed;
        }
        if ( maxTurnrate_ > vfhConfig_.maxTurnrate1ms )
        {
            std::cout << "VFH: requested maxTurnrate ("<<maxTurnrate*180.0/M_PI
            <<"deg) faster than its configured maximum ("<<vfhConfig_.absoluteMaxTurnrate*180.0/M_PI
            <<"deg).  Thresholding.";
            maxTurnrate_ = vfhConfig_.absoluteMaxTurnrate;
        }

        vfhAlgorithm_->SetCurrentMaxSpeed( (int) (maxSpeed_*1000.0) );
        vfhAlgorithm_->SetTurnrateThreshold( (int) (maxTurnrate_*180.0/M_PI) );
    }


    void 
    VfhDriver::copyLaserScan( const std::vector<double> &obsRanges, double playerLaserScan[401][2] )
    {
        double angleIncrement = 0.75;
        // Copy the ranges into a player-style structure.  This means converting units: m -> mm.
        int j = 0; // references into orca structure
        int c = 0;
        for ( int i=0; i < 361; i++ )
        {
            playerLaserScan[i][0] = obsRanges[i]*1000;
        }
    }

    geometry_msgs::Twist VfhDriver::approachGoalCommand( double distanceTolerance,
                                    geometry_msgs::Twist &goal, 
                                    geometry_msgs::Twist &currentVelocity,
                                    std::vector<double>     &obsRanges )
    {
        // Copy stuff into the format required by vfh_algorithm
        float goalDirection = (goal.angular.z*180/M_PI)+90.0;
        if ( goalDirection < 0.0 ) goalDirection += 360.0;

        // distance in mm
        float goalDistance = goal.linear.x * 1000.0;

        // tolerance in mm
        float goalDistanceTolerance = distanceTolerance * 1000.0;
        copyLaserScan( obsRanges, playerLaserScan_ );

        double currentSpeed = currentVelocity.linear.x;
        // Get VFH's choice
        int chosenSpeed, chosenTurnrate;
        vfhAlgorithm_->Update_VFH( playerLaserScan_,
                                currentSpeed,
                                goalDirection,
                                goalDistance,
                                goalDistanceTolerance,
                                chosenSpeed,
                                chosenTurnrate );
        geometry_msgs::Twist vel;
        vel.linear.x = (float)chosenSpeed/1000.0;
        vel.angular.z = chosenTurnrate*M_PI/180.0;
        // Now copy back from player format
        return vel;
    }

}

