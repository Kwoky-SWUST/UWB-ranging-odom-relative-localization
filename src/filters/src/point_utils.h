#ifndef POINT_UTILS
#define POINT_UTILS

#include <vector>
#include <math.h>
#include <iosfwd>

#include "carmen.h"
#include "point_extension.h"

// T Y P E   C O N V E R S I O N S  ///////////////////////
extern inline void
carmen_point_to_3d_point( carmen_point_t* point, carmen_3d_point_t* point_3d )
{
  point_3d->x     = point->x;
  point_3d->y     = point->y;
  point_3d->theta = point->theta;
}


/** Convert (project) the given 6D pose to a 3D pose */
inline carmen_3d_point_t
carmen_6d_to_3d_point( carmen_6d_point_t p )
{
  return (carmen_3d_point_t) { p.x, p.y, p.z, p.theta_z };
}


/** Convert (project) the given 6D pose to a 2D pose */
inline carmen_point_t
carmen_6d_to_2d_point( carmen_6d_point_t p )
{
  return (carmen_point_t) { p.x, p.y, p.theta_z };
}


/** Convert the given 2D position (+orientation) to a 6D pose */
inline carmen_6d_point_t
carmen_2d_to_6d_point( carmen_point_t p )
{
  return (carmen_6d_point_t) { p.x, p.y, 0.0, 0.0, 0.0, p.theta };
}


/** Convert the given 3D position (+orientation) to a 6D pose */
inline carmen_6d_point_t
carmen_3d_to_6d_point( carmen_3d_point_t p )
{
  return (carmen_6d_point_t) { p.x, p.y, p.z, 0.0, 0.0, p.theta };
}


// R I G I D   T R A N S F O R M A T I O N S //////////////

/** Compute the displacement of the point 'to' relative to the pose 'from' expressed in 'from'.
 *  e.g. given frame A as pose, and point p
 *       compute_2d_displacement( A, p ) computes p in the coordinate-frame
 *       represented by A (A.x,A.y = origin, A.theta = global orientation of x-axis)
 *       example code:
 *           carmen_point_t frame, p;
 *           frame.x = 1; frame.y = 0; frame.theta = M_PI/2; // 90°
 *           p.x = 1; p.y = 2; p.theta = 0;
 *           carmen_point_t rel = compute_2d_displacement( frame, p ); // rel = (2,0)
 */
inline carmen_point_t
compute_2d_displacement( carmen_point_t from, carmen_point_t to )
{
  carmen_point_t displacement;
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double _cos = cos( -from.theta );
  double _sin = sin( -from.theta );
  displacement.x = dx * _cos - dy * _sin;
  displacement.y = dx * _sin + dy * _cos;
  displacement.theta = carmen_normalize_theta( to.theta - from.theta );
  return displacement;
}


/** Add the displacement to_add relative to the pose p, respecting
 *  the orientation of p. */
inline carmen_point_t
transform_2d( carmen_point_t p, carmen_point_t to_add )
{
  carmen_point_t result = p;
  double _cos = cos( result.theta );
  double _sin = sin( result.theta );
  result.x     += to_add.x * _cos - to_add.y * _sin;
  result.y     += to_add.x * _sin + to_add.y * _cos;
  result.theta  = carmen_normalize_theta( result.theta + to_add.theta );
  return result;
}


/** Compute the displacement of the point 'to' relative to the pose 'from'. */
inline carmen_point_t
compute_2d_displacement( carmen_3d_point_t from, carmen_point_t to )
{
  return compute_2d_displacement( (carmen_point_t) { from.x, from.y, from.theta }, to );
}


/** Compute the displacement of the point 'to' relative to the pose 'from'. */
inline carmen_point_t
compute_2d_displacement( carmen_6d_point_t from, carmen_6d_point_t to )
{
  return compute_2d_displacement( (carmen_point_t) { from.x, from.y, from.theta_z },
                                  (carmen_point_t) { to.x, to.y, to.theta_z }
                                );
}

/** Compute the displacement of the point 'to' relative to the pose 'from'. */
inline carmen_point_t
compute_2d_displacement( carmen_6d_point_t from, carmen_point_t to )
{
  return compute_2d_displacement( (carmen_point_t) { from.x, from.y, from.theta_z }, to );
}



/** Write pose to stream */
std::ostream& operator <<( std::ostream& ostr, const carmen_point_t p );


carmen_point_t mean_pose_from_vector( std::vector<carmen_point_t> positions, int num_items = -1 );


carmen_point_t mean_position_from_vector( std::vector<carmen_point_t> positions, int num_items = -1 );


double get_3d_squared_deviation( const carmen_point_t& a,
                                 const carmen_point_t& b,
                                 const double weight_xy,
                                 const double weight_theta
                               );

/** Normalize the difference of orientations, provided that the orientations
 *  of the subtracted orientations were normalized before
 */
inline double normalize_normalized_theta( double dtheta )
{
  if ( dtheta < -M_PI )
  {
    dtheta += 2*M_PI;
    if ( dtheta < -M_PI )
      dtheta += 2*M_PI;
  }
  else if ( dtheta > M_PI )
  {
    dtheta -= 2*M_PI;
    if ( dtheta > M_PI )
      dtheta -= 2*M_PI;
  }
  return dtheta;
}


// --------------------------------------------------------------------------
// Taken from carmen/robot/robot_main.c
inline double
interpolate_heading( double head1, double head2, double fraction)
// --------------------------------------------------------------------------
{
  double result;

  if( head1 == head2 || fraction == 0 )
    return head1;
  if( fraction == 1. )
    return head2;

  if(head1 > 0 && head2 < 0 && head1 - head2 > M_PI) {
    head2 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI)
      result -= 2 * M_PI;
    return result;
  } else if(head1 < 0 && head2 > 0 && head2 - head1 > M_PI) {
    head1 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI)
      result -= 2 * M_PI;
    return result;
  } else
    return head1 + fraction * (head2 - head1);
}


// --------------------------------------------------------------------------
inline bool is_similar_carmen_point( const carmen_point_t& a,
                                     const carmen_point_t& b,
                                     const double ERR_EPSILON = 1e-6 )
// --------------------------------------------------------------------------
{
  if( fabs(a.x - b.x) > ERR_EPSILON ||
      fabs(a.y - b.y) > ERR_EPSILON ||
      fabs(carmen_normalize_theta(a.theta) - carmen_normalize_theta(b.theta)) > ERR_EPSILON )
    return false;
  return true;
}


#endif
