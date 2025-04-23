#ifndef AMBISENSE_POSE_UTILS
#define AMBISENSE_POSE_UTILS
#include <Eigen/Eigen>


#include <iostream>
#include "point_extension.h"
#include "point_utils.h"



// --------------------------------------------------------------------------
class Pose : public carmen_6d_point_t
// --------------------------------------------------------------------------
{
  public:
    Pose(); ///< Create a 6D pose with 0.0 in all of its components
    Pose( const carmen_6d_point_t& p ); ///<  Create this pose from the given 6D point
    Pose( const carmen_point_t& p );    ///<  Create this pose from the given 2D point
    Pose( double x, double y, double z, double theta_x, double theta_y, double theta_z ); ///<  Create this pose from the given 6D point
    Pose( const Eigen::Isometry3d& m );    ///<  Create this pose from the given homogeneous vector (4x1 matrix) or homogeneous matrix (4x4)

    Pose& operator =( carmen_6d_point_t p ); ///< Assign this pose the given 6D point

    bool operator ==( const Pose& p ) const; ///< Check for equality in component-wise fashion

    friend std::ostream& operator <<( std::ostream& ostr, const Pose& p ); ///< Stream output


    inline carmen_point_t get2DPose() const ///< Return the projection to X,Y,THETA(z)
      { return (carmen_point_t) { x, y, {theta_z} }; }

    inline carmen_3d_point_t get3DPose() const ///< Return the projection to X,Y,Z,THETA(z)
      { return (carmen_3d_point_t) { x, y, z, {theta_z} }; }

    /** Return a homogeneous 4x4 matrix which represents this pose
     */
    Eigen::Isometry3d asMatrix() const;

    /** Add the given pose in a component-wise fashion */
    Pose add( const Pose& by );

    /** Shift the pose by the given displacements */
    void shiftXY( double dx, double dy );

    /** Rotate the pose around the z-axis, i.e., rotate its x/y-coordinates
     *  and update the orientation theta_z
     */
    void rotateZ( double thetaZ );

    static Pose getMinimum( const Pose& a, const Pose& b ); ///< Return the component-wise minimum of both poses
    static Pose getMaximum( const Pose& a, const Pose& b ); ///< Return the component-wise maximum of both poses
    /**
     * Compute interpolated pose between a at 0 and b at 1, with the linear interpolation factor fraction
     * @param a start pose (at 0)
     * @param b end pose (at 1)
     * @param fraction linear interpolation factor in [0,1]
     * @return linear interpolated pose at timestamp fraction
     */
    static Pose getInterpolated( const Pose& a, const Pose& b, const double fraction );

    /** Set the pose to invalid coordinates */
    void invalidate();

    /** Do the coordinates represent a valid pose? */
    bool isValid() const;

    /** Return the 2D Euclidean distance to the given pose */
    double get2DDistance( const Pose& p ) const;

    /** Return the 3D Euclidean distance to the given pose */
    double get3DDistance( const Pose& p ) const;

    /** Component-wise comparison with precision */
    bool isApprox( const Pose& other, const double precision = 1e-16 ) const;
};

#endif
