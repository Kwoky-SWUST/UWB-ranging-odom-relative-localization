#ifndef SCIROCO__COORDINATE_TRANSFORMATION_H
#define SCIROCO__COORDINATE_TRANSFORMATION_H

#include "pose.h"
#include <Eigen/Eigen>

/** Return a homogeneous 4x4 matrix that represents a translation by
 *  the vector (x,y,z) and a rotation
 *  first by thetaX around the x-axis, then by thetaY around the y-axis,
 *  and finally by thetaZ around the z-axis.
 */
Eigen::Isometry3d HomogeneousTransformation( double x, double y, double z,
                                  double thetaX, double thetaY, double thetaZ );


/** Return a homogeneous 4x4 matrix that represents a translation by
 *  the vector (p.x, p.y, p.z) and a rotation
 *  first by p.theta_x around the x-axis, then by p.theta_y around the y-axis,
 *  and finally by p.theta_z around the z-axis.
 */
Eigen::Isometry3d HomogeneousTransformation( const carmen_6d_point_t& p );


/** Return a homogeneous 4x4 matrix that represents a translation by
 *  the vector (x,y,z).
 */
Eigen::Isometry3d HomogeneousTranslation( double x, double y, double z );


/** Return a homogeneous 4x4 matrix that represents a rotation
 *  first by thetaX around the x-axis, then by thetaY around the y-axis,
 *  and finally by thetaZ around the z-axis
 */
Eigen::Isometry3d HomogeneousRollPitchYaw( double thetaX, double thetaY, double thetaZ );
Eigen::Isometry3d HomogeneousRollPitchYaw( const carmen_6d_point_t& p );
Eigen::Isometry3d HomogeneousRollPitchYaw( const Pose& p );


#endif // SCIROCO__COORDINATE_TRANSFORMATION_H
