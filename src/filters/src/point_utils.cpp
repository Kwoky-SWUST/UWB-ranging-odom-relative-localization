#include <iostream>
#include <vector>
#include "point_utils.h"


// --------------------------------------------------------------------------
std::ostream& operator <<( std::ostream& ostr, const carmen_point_t p )
// --------------------------------------------------------------------------
{
  ostr << "(" << p.x << ", " << p.y << "; " << p.theta << ")";
  return ostr;
}


// --------------------------------------------------------------------------
carmen_point_t
    mean_pose_from_vector( std::vector<carmen_point_t> positions,
                           int num_items )
// --------------------------------------------------------------------------
{
  uint _max = positions.size();
  if ( _max == 0 )
    return (carmen_point_t) { 0,0,0 };

  if ( num_items >= 0 )
    _max = (uint) num_items;

  carmen_point_t mean = {0,0,0};
  double theta_y = 0.0, theta_x = 0.0;
  for ( uint u = 0; u < _max; ++u )
  {
    mean.x += positions[u].x;
    mean.y += positions[u].y;
    theta_x += cos( positions[u].theta );
    theta_y += sin( positions[u].theta );
  }
  theta_x /= _max;
  theta_y /= _max;
  mean.x /= _max;
  mean.y /= _max;
  mean.theta = atan2( theta_y, theta_x );

  return mean;
}


// --------------------------------------------------------------------------
carmen_point_t
    mean_position_from_vector( std::vector<carmen_point_t> positions,
                               int num_items )
// --------------------------------------------------------------------------
{
  uint _max = positions.size();
  if ( _max == 0 )
    return (carmen_point_t) { 0,0,0 };

  if ( num_items >= 0 )
    _max = (uint) num_items;

  carmen_point_t mean = {0,0,0};
  for ( uint u = 0; u < _max; ++u )
  {
    mean.x += positions[u].x;
    mean.y += positions[u].y;
  }
  mean.x /= _max;
  mean.y /= _max;

  return mean;
}


// --------------------------------------------------------------------------
double get_3d_squared_deviation( const carmen_point_t& a,
                                 const carmen_point_t& b,
                                 const double weight_xy,
                                 const double weight_theta
                               )
// --------------------------------------------------------------------------
{
  return weight_xy * ( carmen_square( a.x - b.x ) + carmen_square( a.y - b.y ) )
      + weight_theta * carmen_square( carmen_normalize_theta( a.theta - b.theta ) );
}
