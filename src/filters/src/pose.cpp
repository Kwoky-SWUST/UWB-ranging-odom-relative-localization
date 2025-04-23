#include <assert.h>
#include <iostream>
#include <vector>
//
#include "pose.h"
#include "coordinate_transformation.h"


// --------------------------------------------------------------------------
std::ostream& operator <<( std::ostream& ostr, const Pose& p )
// --------------------------------------------------------------------------
{
  ostr << "(" << p.x << ", " << p.y << ", " << p.z << "; " << p.theta_x << ", " << p.theta_y << ", " << p.theta_z << ')';
  return ostr;
}


// --------------------------------------------------------------------------
Pose::
    Pose()
  : carmen_6d_point_t( (carmen_6d_point_t) { 0,0,0,0,0,0 } )
// --------------------------------------------------------------------------
{
}


// --------------------------------------------------------------------------
Pose::
    Pose( double x, double y, double z,
          double theta_x, double theta_y, double theta_z )
// --------------------------------------------------------------------------
{
  this->x = x;
  this->y = y;
  this->z = z;
  this->theta_x = theta_x;
  this->theta_y = theta_y;
  this->theta_z = theta_z;
}


// --------------------------------------------------------------------------
Pose::
    Pose( const carmen_6d_point_t& p )
// --------------------------------------------------------------------------
{
  *this = p;
}


// --------------------------------------------------------------------------
Pose::
    Pose( const carmen_point_t& p )
// --------------------------------------------------------------------------
{
  *this = (carmen_6d_point_t) { p.x, p.y, 0.0, 0.0, 0.0, p.theta };
}


// --------------------------------------------------------------------------
Pose::
    Pose( const Eigen::Isometry3d& m )
// --------------------------------------------------------------------------
{
    this->x       = m(0,3);
    this->y       = m(1,3);
    this->z       = m(2,3);
    this->theta_x = 0.0;
    this->theta_y = 0.0;
    this->theta_z = 0.0;

}


// --------------------------------------------------------------------------
Pose& Pose::
    operator =( carmen_6d_point_t p )
// --------------------------------------------------------------------------
{
  x = p.x;
  y = p.y;
  z = p.z;
  theta_x = p.theta_x;
  theta_y = p.theta_y;
  theta_z = p.theta_z;
  return *this;
}


// --------------------------------------------------------------------------
bool Pose::
    operator ==( const Pose& p ) const
// --------------------------------------------------------------------------
{
  return ( x == p.x
      && y == p.y
      && z == p.z
      && theta_x == p.theta_x
      && theta_y == p.theta_y
      && theta_z == p.theta_z );
}


// --------------------------------------------------------------------------
 Eigen::Isometry3d Pose::
    asMatrix() const
// --------------------------------------------------------------------------
{
  return HomogeneousTransformation( *this );
}


// --------------------------------------------------------------------------
Pose Pose::
    getMinimum( const Pose& a, const Pose& b )
// --------------------------------------------------------------------------
{
  Pose result;
  result.x = fmin( a.x, b.x );
  result.y = fmin( a.y, b.y );
  result.z = fmin( a.z, b.z );
  result.theta_x = fmin( a.theta_x, b.theta_x );
  result.theta_y = fmin( a.theta_y, b.theta_y );
  result.theta_z = fmin( a.theta_z, b.theta_z );
  return result;
}


// --------------------------------------------------------------------------
Pose Pose::
    getMaximum( const Pose& a, const Pose& b )
// --------------------------------------------------------------------------
{
  Pose result;
  result.x = fmax( a.x, b.x );
  result.y = fmax( a.y, b.y );
  result.z = fmax( a.z, b.z );
  result.theta_x = fmax( a.theta_x, b.theta_x );
  result.theta_y = fmax( a.theta_y, b.theta_y );
  result.theta_z = fmax( a.theta_z, b.theta_z );
  return result;
}


// --------------------------------------------------------------------------
void Pose::
    shiftXY( double dx, double dy )
// --------------------------------------------------------------------------
{
  this->x += dx;
  this->y += dy;
}


// --------------------------------------------------------------------------
void Pose::
    rotateZ( double thetaZ)
// --------------------------------------------------------------------------
{
  double tx = this->x;
  double ty = this->y;

  double _cos = cos( thetaZ );
  double _sin = sin( thetaZ );

  this->x = _cos*tx - _sin*ty;
  this->y = _sin*tx + _cos*ty;

  theta_z += thetaZ;
}


// --------------------------------------------------------------------------
Pose Pose::
    add( const Pose& by )
// --------------------------------------------------------------------------
{
  Pose p;
/*      \cos \alpha \cos \beta
      &   \cos \alpha \sin \beta \sin \gamma - \sin \alpha \cos \gamma
      &   \cos \alpha \sin \beta \cos \gamma + \sin \alpha \sin \gamma \\
      \sin \alpha \cos \beta
      &   \sin \alpha \sin \beta \sin \gamma + \cos \alpha \cos \gamma
      &   \sin \alpha \sin \beta \cos \gamma - \cos \alpha \sin \gamma \\
      - \sin \beta
      &   \cos \beta \sin \gamma
      &   \cos \beta \cos \gamma*/
  double _cos_x = cos( this->theta_x );
  double _sin_x = sin( this->theta_x );
  double _cos_y = cos( this->theta_y );
  double _sin_y = sin( this->theta_y );
  double _cos_z = cos( this->theta_z );
  double _sin_z = sin( this->theta_z );

  p.x = _cos_z * _cos_y                                 * by.x
      + ( _cos_z * _sin_y * _sin_x  - _sin_z * _cos_x ) * by.y
      + ( _cos_z * _sin_y * _cos_x  + _sin_z * _sin_x ) * by.z
      + this->x;

  p.y = _sin_z * _cos_y                                * by.x
      + ( _sin_z * _sin_y * _sin_z + _cos_z * _cos_x ) * by.y
      + ( _sin_z * _sin_y * _cos_z - _cos_z * _sin_x ) * by.z
      + this->y;

  p.z = -_sin_y                                        * by.x
      +          _cos_y * _sin_x                       * by.y
      +          _cos_y * _cos_x                       * by.z
      + this->z;

  p.theta_x = this->theta_x + by.theta_x;
  p.theta_y = this->theta_y + by.theta_y;
  p.theta_z = this->theta_z + by.theta_z;

  return p;
}


// --------------------------------------------------------------------------
void Pose::
    invalidate()
// --------------------------------------------------------------------------
{
  x = INFINITY;
}


// --------------------------------------------------------------------------
bool Pose::
    isValid() const
// --------------------------------------------------------------------------
{
  return !( isinf(x) || isinf(y) || isinf(z) );
}


// --------------------------------------------------------------------------
double Pose::
      get2DDistance( const Pose& p ) const
// --------------------------------------------------------------------------
{
  return sqrt( carmen_square( this->x - p.x ) + carmen_square( this->y - p.y ) );
}


// --------------------------------------------------------------------------
double Pose::
      get3DDistance( const Pose& p ) const
// --------------------------------------------------------------------------
{
  return sqrt( carmen_square( this->x - p.x ) + carmen_square( this->y - p.y ) + carmen_square( this->z - p.z ) );
}


// --------------------------------------------------------------------------
bool Pose::
    isApprox( const Pose& other, const double precision /* = 1e-16 */ ) const
// --------------------------------------------------------------------------
{
  return ( fabs( other.x - this->x ) <= precision &&
           fabs( other.y - this->y ) <= precision &&
           fabs( other.z - this->z ) <= precision &&
           fabs( other.theta_x - this->theta_x ) <= precision &&
           fabs( other.theta_y - this->theta_y ) <= precision &&
           fabs( other.theta_z - this->theta_z ) <= precision );
}

// --------------------------------------------------------------------------
Pose Pose::
    getInterpolated( const Pose& a, const Pose& b, const double fraction )
// --------------------------------------------------------------------------
{
  if( a == b || fraction == 0 || fraction == 1 )
    return a;
  Pose result;
  result.x = a.x + fraction * (b.x - a.x);
  result.y = a.y + fraction * (b.y - a.y);
  result.z = a.z + fraction * (b.z - a.z);
  result.theta_x = interpolate_heading( a.theta_x, b.theta_x, fraction );
  result.theta_y = interpolate_heading( a.theta_y, b.theta_y, fraction );
  result.theta_z = interpolate_heading( a.theta_z, b.theta_z, fraction );
  return result;
}

