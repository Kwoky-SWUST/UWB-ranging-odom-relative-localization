#include "coordinate_transformation.h"
//
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>

using namespace std;

// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousTranslation( double x, double y, double z )
// --------------------------------------------------------------------------
{
  Eigen::Isometry3d M=Eigen::Isometry3d::Identity();

  M(0,3) = x;
  M(1,3) = y;
  M(2,3) = z;

  return M;
}


// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousRollPitchYaw( double thetaX, double thetaY, double thetaZ )
// --------------------------------------------------------------------------
{
  Eigen::Isometry3d M=Eigen::Isometry3d::Identity();

  M(0,0) = cos( thetaZ ) * cos( thetaY );
  M(0,1) = cos( thetaZ ) * sin( thetaY ) * sin( thetaX ) - sin( thetaZ ) * cos( thetaX );
  M(0,2) = cos( thetaZ ) * sin( thetaY ) * cos( thetaX ) + sin( thetaZ ) * sin( thetaX );

  M(1,0) = sin( thetaZ ) * cos( thetaY );
  M(1,1) = sin( thetaZ ) * sin( thetaY ) * sin( thetaX ) + cos( thetaZ ) * cos( thetaX );
  M(1,2) = sin( thetaZ ) * sin( thetaY ) * cos( thetaX ) - cos( thetaZ ) * sin( thetaX );

  M(2,0) = - sin( thetaY );
  M(2,1) = cos( thetaY ) * sin( thetaX );
  M(2,2) = cos( thetaY ) * cos( thetaX );

//   cout << "RPY(" << carmen_radians_to_degrees( thetaX ) << ","
//       << carmen_radians_to_degrees( thetaY ) << ","
//       << carmen_radians_to_degrees( thetaZ ) << "):\n" << M << endl;

  return M;
}


// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousRollPitchYaw( const carmen_6d_point_t& p )
// --------------------------------------------------------------------------
{
  return HomogeneousRollPitchYaw(p.theta_x,p.theta_y,p.theta_z);
}


// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousRollPitchYaw( const Pose& p )
// --------------------------------------------------------------------------
{
  return HomogeneousRollPitchYaw(p.theta_x,p.theta_y,p.theta_z);
}


// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousTransformation( double x, double y, double z,
                                  double thetaX, double thetaY, double thetaZ )
// --------------------------------------------------------------------------
{
  return HomogeneousTransformation( (carmen_6d_point_t) { x, y, z, thetaX, thetaY, thetaZ } );
}


// --------------------------------------------------------------------------
Eigen::Isometry3d HomogeneousTransformation( const carmen_6d_point_t& p )
// --------------------------------------------------------------------------
{
  Eigen::Isometry3d M=Eigen::Isometry3d::Identity();

  // //////////////////////////////////////////////////////
  // Fill translation column
  M(0,3) = p.x;
  M(1,3) = p.y;
  M(2,3) = p.z;

  // //////////////////////////////////////////////////////
  // Fill rotation part of matrix (upper left 3x3 block)
  const double cos_x = cos( p.theta_x );
  const double sin_x = sin( p.theta_x );
  const double cos_y = cos( p.theta_y );
  const double sin_y = sin( p.theta_y );
  const double cos_z = cos( p.theta_z );
  const double sin_z = sin( p.theta_z );

  const double cos_z_sin_y = cos_z * sin_y;
  const double sin_z_sin_y = sin_z * sin_y;

  M(0,0) = cos_z * cos_y;
  M(0,1) = cos_z_sin_y * sin_x - sin_z * cos_x;
  M(0,2) = cos_z_sin_y * cos_x + sin_z * sin_x;

  M(1,0) = sin_z * cos_y;
  M(1,1) = sin_z_sin_y * sin_x + cos_z * cos_x;
  M(1,2) = sin_z_sin_y * cos_x - cos_z * sin_x;

  M(2,0) = -sin_y;
  M(2,1) =  cos_y * sin_x;
  M(2,2) =  cos_y * cos_x;

  return M;
}
