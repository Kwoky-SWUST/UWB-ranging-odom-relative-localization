/*
 * Orca-Robotics Project: Components for robotics 
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2006 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include "vfh_algorithmconfig.h"
#include <iostream>
#include <cmath>
#include <sstream>
using namespace std;

namespace vfh {

VfhAlgorithmConfig::VfhAlgorithmConfig()
{
    cellSize                  = 0.0;
    gridWidthInCells          = 0;
    sectorAngle               = 0.0;
    robotRadius               = 0.0;
    safetyDist0ms             = 0.0;
    safetyDist1ms             = 0.0;
    maxSpeed                  = 0.0;
    maxSpeedNarrowOpening     = 0.0;
    maxSpeedWideOpening       = 0.0;
    maxAcceleration           = 0.0;
    maxTurnrate0ms            = 0.0;
    maxTurnrate1ms            = 0.0;
    absoluteMaxTurnrate       = 0.0;
    minTurnRadiusSafetyFactor = 0.0;
    freeSpaceCutoff0ms        = 0.0;
    freeSpaceCutoff1ms        = 0.0;
    obsCutoff0ms              = 0.0;
    obsCutoff1ms              = 0.0;
    weightDesiredDir          = 0.0;
    weightCurrentDir          = 0.0;
}

int
VfhAlgorithmConfig::checkSanity( std::string &warnings, std::string &errors )
{
    std::stringstream ssWarn, ssErr;

    if ( maxTurnrate0ms < maxTurnrate1ms )
        ssWarn << "Turnrate is set to _INCREASE_ with speed" << endl;
    if ( safetyDist0ms > safetyDist1ms )
        ssWarn << "Safety Dist is set to _DECREASE_ with speed" << endl;
    if ( freeSpaceCutoff0ms < freeSpaceCutoff1ms )
        ssWarn << "FreeSpaceCutoff is set to _INCREASE_ with speed" << endl;
    if ( obsCutoff0ms < obsCutoff1ms )
        ssWarn << "ObsCutoff is set to _INCREASE_ with speed" << endl;
    if ( maxSpeed < maxSpeedWideOpening )
        ssWarn << "Max Speed is set to _INCREASE_ through a wide opening" << endl;
    if ( maxSpeedNarrowOpening > maxSpeedWideOpening )
        ssWarn << "Max Speed is set to _INCREASE_ through a narrow opening" << endl;
    if ( robotRadius == 0.0 )
        ssWarn << "Zero robot radius" << endl;

    if ( cellSize <= 0.0 )
        ssErr << "Bad cell size: " << cellSize << endl;
    if ( gridWidthInCells <= 0 )
        ssErr << "Bad gridWidthInCells: " << gridWidthInCells << endl;
    if ( sectorAngle <= 0.0 )
        ssErr << "Bad sectorAngle: " << sectorAngle << endl;
    if ( robotRadius < 0.0 )
        ssErr << "Bad robot radius: " << robotRadius << endl;
    if ( safetyDist0ms < 0.0 || safetyDist1ms < 0.0 )
        ssErr << "negative safety dist" << endl;
    if ( maxSpeed <= 0.0 || maxSpeedNarrowOpening <= 0.0 || maxSpeedWideOpening <= 0.0 )
        ssErr << "speeds <= 0.0" << endl;
    if ( minTurnRadiusSafetyFactor < 0.0 )
        ssErr << "negative minTurnRadiusSafetyFactor" << endl;
    if ( absoluteMaxTurnrate <= 0.0 )
        ssErr << "absoluteMaxTurnrate <= 0" << endl;

    warnings = ssWarn.str();
    errors   = ssErr.str();

    if ( warnings == "" && errors == "" ) return 0;
    return -1;
}

std::ostream &operator<<( std::ostream &s, const VfhAlgorithmConfig &c )
{
    s << "VfhAlgorithm Properties:     " << endl;
    s << "\tCellSize:                  " << c.cellSize                  << endl
      << "\tgridWidthInCells:          " << c.gridWidthInCells          << endl
      << "\tsectorAngle:               " << c.sectorAngle*180.0/M_PI << "deg" << endl
      << "\trobotRadius:               " << c.robotRadius               << endl
      << "\tsafetyDist0ms:             " << c.safetyDist0ms             << endl
      << "\tsafetyDist1ms:             " << c.safetyDist1ms             << endl
      << "\tmaxSpeed:                  " << c.maxSpeed                  << endl
      << "\tmaxSpeedNarrowOpening:     " << c.maxSpeedNarrowOpening     << endl
      << "\tmaxSpeedWideOpening:       " << c.maxSpeedWideOpening       << endl
      << "\tmaxAcceleration:           " << c.maxAcceleration           << endl
      << "\tmaxTurnrate0ms:            " << c.maxTurnrate0ms*180.0/M_PI << "deg" << endl
      << "\tmaxTurnrate1ms:            " << c.maxTurnrate1ms*180.0/M_PI << "deg" << endl
      << "\tabsoluteMaxTurnrate:       " << c.absoluteMaxTurnrate*180.0/M_PI << "deg" << endl
      << "\tminTurnRadiusSafetyFactor: " << c.minTurnRadiusSafetyFactor << endl
      << "\tfreeSpaceCutoff0ms:        " << c.freeSpaceCutoff0ms        << endl
      << "\tfreeSpaceCutoff1ms:        " << c.freeSpaceCutoff1ms        << endl
      << "\tobsCutoff0ms:              " << c.obsCutoff0ms              << endl
      << "\tobsCutoff1ms:              " << c.obsCutoff1ms              << endl
      << "\tweightDesiredDir:          " << c.weightDesiredDir          << endl
      << "\tweightCurrentDir:          " << c.weightCurrentDir             ;

    return s;
}

}
