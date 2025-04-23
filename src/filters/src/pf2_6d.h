#ifndef AMBISENSE_SLAM__FILTERS__PF2_6D_H
#define AMBISENSE_SLAM__FILTERS__PF2_6D_H

#include "particle_filter_2.h"
#include "pose.h"


namespace pf2
{
  /** Specialized version of a particle for
   *  6D poses (3D position + 3D orientation)
   */
  // ------------------------------------------------------------------------
  class Particle6D : public Pose
  // ------------------------------------------------------------------------
  {
    public:
      Particle6D(); ///< Constructor
      Particle6D( const Particle6D& p ); ///< Constructor
      Particle6D( const carmen_3d_point_t& p ); ///< Create a particle with the given 3D pose
      Particle6D( const carmen_6d_point_t& p ); ///< Create a particle with the given 3D pose
      Particle6D( const Pose& p ); ///< Constructor
      //       virtual ~Particle6D(); ///< NOTE: We found that not using the destructor speeds up computations!
      void setPosition( const carmen_3d_point_t& p ); ///< Set the particle to the given 3D pose
      void setPosition( const carmen_6d_point_t& p ); ///< Set the particle to the given 3D pose
      void setPosition( const carmen_point_t&    p ); ///< Set the particle to the given 2D pose
      void setPosition( const Pose& p ); ///< Set the particle to the given pose

      friend std::ostream& operator <<( std::ostream& ostr, const Particle6D& p ); ///< Stream output


      /// Assignment operator is overloaded because assignments are
      /// significantly speeded up!
      Particle6D& operator = ( const Particle6D& p )
      {
#ifdef HAVE_PARTICLE_TRAJECTORY
//         if ( p.trajectory.size() > 0 )
//         {
//           trajectory.resize( p.trajectory.size() );
//           copy( p.trajectory.begin(), p.trajectory.end(), trajectory.begin() );
//         }
#endif

        x                  = p.x;
        y                  = p.y;
        z                  = p.z;
        theta_x            = p.theta_x;
        theta_y            = p.theta_y;
        theta_z            = p.theta_z;
        weight             = p.weight;
        parent             = p.parent;
  #ifdef HAVE_LOG_WEIGHTS
        log_weight         = p.log_weight;
  #endif
        return *this;
      }

    public:
      /// Importance weight of the particle
      double weight;

      /// Index of parent particle at previous time step
      int parent;

#ifdef HAVE_LOG_WEIGHTS
      /// Logarithm of the importance weight of the particle
      double log_weight;
#endif

#ifdef HAVE_PARTICLE_TRAJECTORY
      /// History of past poses
//       std::vector< Pose > trajectory;
#endif
  };


  /** Specialized version of a particle filter with 6D poses
   *  @author Philipp Vorst, Artur Koch
   */
  // ------------------------------------------------------------------------
  class ParticleFilter6D : public pf2::ParticleFilter< pf2::Particle6D >
  // ------------------------------------------------------------------------
  {
    public:
      /** Create a 6D particle filter for poses. */
      ParticleFilter6D();

      /** Compute mean and covariance matrix.
       *  @param cov Will be set to the zero matrix if the filter size is
       *             smaller than 2
       */
      void computeMeanAndCovariance( Pose& mean, double cov[36] );
      void computeUnweightedMeanAndCovariance( Pose& mean, double cov[36] );
      void computeMeanAndVariance2D( Pose& mean, Pose& variance, double& xy_covariance );
      void computeMean( Pose& mean );

      /** Compute weighted geometric median by an iterative solver (currently only for 2D-position)
       * see Weiszfeld's algorithm or Kuhn and Kuenne (1962): An efficient algorithm for the numerical solution of the generalized weber problem in spatial economics
       */
      void computeMedian( Pose& median );

     
      Particle6D getClosestParticle( const Particle6D& p );

      friend std::ostream & operator << ( std::ostream& ostr, const ParticleFilter6D& pf );

      /// TODO: test specialized KLD

      /** Prepapre KD sampling. Has to be called once in the beginning,
       *  before the first call to kdSample.
       */
      void kldInitialize( double mapMinX,  double mapMaxX,
                          double mapMinY,  double mapMaxY,
                          double mapMinZ,  double mapMaxZ,
                          double binWidth, double binAngle,
                          uint   minimalNumberOfParticles,
                          uint   maximalNumberOfParticles,
                          double delta = 0.99, double epsilon = 0.05
                        );

      /** specialized version */
      void kldSample( LikelihoodFunction likelihoodFunction, void* sensorData );

      double m_dKLDSamplingMinZ;
      int    m_iKLDSamplingMapDepth;

  };
}

#endif // AMBISENSE_SLAM__FILTERS__PF2_6D_H
