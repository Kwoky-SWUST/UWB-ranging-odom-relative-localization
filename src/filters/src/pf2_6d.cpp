#include "carmen.h"
#include "pf2_6d.h"
// #include <mcl_random/fast_gauss_rand.h>

#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;


typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6;

// --------------------------------------------------------------------------
void
pose2matrix( const Pose&p, Vector6d& M )
// --------------------------------------------------------------------------
{
  M(0,0) = p.x;
  M(1,0) = p.y;
  M(2,0) = p.z;
  M(3,0) = p.theta_x;
  M(4,0) = p.theta_y;
  M(5,0) = p.theta_z;
}


// --------------------------------------------------------------------------
std::ostream& pf2::operator <<( std::ostream& ostr, const pf2::Particle6D& p )
// --------------------------------------------------------------------------
{
  ostr << "(" << p.x << ", " << p.y << ", " << p.z << "; " << p.theta_x << ", " << p.theta_y << ", " << p.theta_z << ')' << " : " << p.weight;
  return ostr;
}


// --------------------------------------------------------------------------
pf2::Particle6D::
    Particle6D() : Pose()
// --------------------------------------------------------------------------
{
}


// // --------------------------------------------------------------------------
// pf2::Particle6D::
//     ~Particle6D()
// // --------------------------------------------------------------------------
// {
// #ifdef HAVE_PARTICLE_TRAJECTORY
//   trajectory.clear();
// #endif
// }


// --------------------------------------------------------------------------
pf2::Particle6D::
    Particle6D( const pf2::Particle6D& p ) : Pose()
// --------------------------------------------------------------------------
{
  *this = p;
}


// --------------------------------------------------------------------------
pf2::Particle6D::
    Particle6D( const carmen_3d_point_t& p ) : Pose()
// --------------------------------------------------------------------------
{
  setPosition( p );
}


// --------------------------------------------------------------------------
pf2::Particle6D::
    Particle6D( const carmen_6d_point_t& p ) : Pose()
// --------------------------------------------------------------------------
{
  setPosition( p );
}

// --------------------------------------------------------------------------
pf2::Particle6D::
    Particle6D( const Pose& p ) : Pose()
// --------------------------------------------------------------------------
{
  setPosition( p );
}



// --------------------------------------------------------------------------
void pf2::Particle6D::
    setPosition( const carmen_3d_point_t& p )
// --------------------------------------------------------------------------
{
  x = p.x;
  y = p.y;
  z = p.z;
  theta_x = 0.0;
  theta_y = 0.0;
  theta_z = p.theta;
}


// --------------------------------------------------------------------------
void pf2::Particle6D::
    setPosition( const carmen_point_t& p )
// --------------------------------------------------------------------------
{
  x = p.x;
  y = p.y;
  z = 0.0;
  theta_x = 0.0;
  theta_y = 0.0;
  theta_z = p.theta;
}


// --------------------------------------------------------------------------
void pf2::Particle6D::
    setPosition( const carmen_6d_point_t& p )
// --------------------------------------------------------------------------
{
  x = p.x;
  y = p.y;
  z = p.z;
  theta_x = p.theta_x;
  theta_y = p.theta_y;
  theta_z = p.theta_z;
}


// --------------------------------------------------------------------------
void pf2::Particle6D::
    setPosition( const Pose& p )
// --------------------------------------------------------------------------
{
  x = p.x;
  y = p.y;
  z = p.z;
  theta_x = p.theta_x;
  theta_y = p.theta_y;
  theta_z = p.theta_z;
}


// --------------------------------------------------------------------------
pf2::ParticleFilter6D::
    ParticleFilter6D()
  : pf2::ParticleFilter< Particle6D >()
// --------------------------------------------------------------------------
{
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    computeMeanAndVariance2D( Pose   & mean,
                              Pose   & variance,
                              double & xy_covariance )
// --------------------------------------------------------------------------
{
  mean = variance = Pose( 0,0,0,0,0,0 );
  xy_covariance = 0.0;
  double theta_x = 0.0, theta_y = 0.0;
  if ( size() == 0 )
    return;

  for ( std::vector< Particle6D >::const_iterator it = particles.begin(); it != particles.end(); ++it )
  {
    mean.x  += it->x * it->weight;
    mean.y  += it->y * it->weight;
    mean.z  += it->z * it->weight;
    theta_x += cos( it->theta_z ) * it->weight;
    theta_y += sin( it->theta_z ) * it->weight;
  }
  mean.theta_z = atan2( theta_y, theta_x );
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    computeMean( Pose& mean )
// --------------------------------------------------------------------------
{
  mean = Pose( 0,0,0,0,0,0 );
  if ( size() == 0 )
    return;

/*  Particle6D* p;
  for ( uint u = 0; u < particles.size(); ++u )
  {
    p = &( particles[u] );
    mean.x  += p->x * p->weight;
    mean.y  += p->y * p->weight;
    mean.z  += p->z * p->weight;
    theta_x += cos( p->theta_z ) * p->weight;
    theta_y += sin( p->theta_z ) * p->weight;
  }*/
  double theta_zx = 0.0, theta_zy = 0.0;
  for ( std::vector< Particle6D >::const_iterator it = particles.begin(); it != particles.end(); ++it )
  {
    mean.x  += it->x * it->weight;
    mean.y  += it->y * it->weight;
    mean.z  += it->z * it->weight;
    theta_zx += cos( it->theta_z ) * it->weight;
    theta_zy += sin( it->theta_z ) * it->weight;
  }
  mean.theta_z = atan2( theta_zy, theta_zx );
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    computeMeanAndCovariance( Pose& mean, double covariance[36] )
// --------------------------------------------------------------------------
{
  computeMean( mean );

  // No meaningful covariance if filter size is smaller than 2.
  if ( size() < 2 )
  {
    for ( int i = 0; i < 6; ++i )
      for ( int j = 0; j < 6; ++j )
        covariance[ i*6+j ] = 0;

    return;
  }

  Vector6d Mean;
  Vector6d X;
  Vector6d D;
  pose2matrix( mean, Mean );

  Matrix6 C=Matrix6::Zero();


  for ( std::vector< Particle6D >::const_iterator it = particles.begin(); it != particles.end(); ++it )
  {
    pose2matrix( *it, X );
    
    D  = X - Mean;
    D(3,0) = carmen_normalize_theta( D(3,0) );
    D(4,0) = carmen_normalize_theta( D(4,0) );
    D(5,0) = carmen_normalize_theta( D(5,0) );
    C += D * D.transpose() * it->weight;
  }
//   C *= 1.0 / particles.size();

  for ( int i = 0; i < 6; ++i )
    for ( int j = 0; j < 6; ++j )
      { 
        if(C(i,j) < 0.0000000001)
        covariance[ i*6+j ] = 0.0;
        else
        covariance[ i*6+j ] = C( i, j );
      }
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    computeUnweightedMeanAndCovariance( Pose& mean, double covariance[36] )
// --------------------------------------------------------------------------
{
  computeMean( mean );

  if ( size() == 0 )
    return;

  Vector6d Mean;
  Vector6d X;
  Vector6d D;
  pose2matrix( mean, Mean );

  Matrix6 C=Matrix6::Zero();


  for ( std::vector< Particle6D >::const_iterator it = particles.begin(); it != particles.end(); ++it )
  {
    pose2matrix( *it, X );
    D  = X - Mean;
    D(3,0) = carmen_normalize_theta( D(3,0) );
    D(4,0) = carmen_normalize_theta( D(4,0) );
    D(5,0) = carmen_normalize_theta( D(5,0) );
    C += D * D.transpose();
  }
  C *= 1.0 / particles.size();
  for ( int i = 0; i < 6; ++i )
    for ( int j = 0; j < 6; ++j )
    {
      if(C(i,j) < 0.0000000001)
        covariance[ i*6+j ] = 0.0;
      else
        covariance[ i*6+j ] = C( i, j );
    }
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    computeMedian(Pose & median)
// --------------------------------------------------------------------------
{
  // iterative solver for weighted geometric median (currently only for position)
  // see Weiszfeld's algorithm or Kuhn and Kuenne: An efficient algorithm for the numerical solution of the generalized weber problem in spatial economics, 1962

  median = Pose( 0,0,0,0,0,0 );
  if ( size() == 0 )
    return;

  // init estimate with the mean
  computeMean(median);

  // init loop conditions
  const double errorThreshold = 0.0001;
  const int maxIterations = 100;

  // init temporaries
  double error = errorThreshold + 1.;
  int iteration = 0;
  Pose newMedianEstimate(median);
  double normalizedWeight;

  // iterative solver
  while( error > errorThreshold && iteration < maxIterations ) {

    Pose nominator; // sum( w(i)x(i) / dist( x(i)-X )
    double denominator = 0.; // sum( w(i) / dist( x(i)-X )

    // loop over each particle
    for( std::vector< Particle6D >::const_iterator it = particles.begin(); it != particles.end(); ++it ) {

      // skip the particle if it matches the current estimate (the gradient is not defined at this position)
      if( median.x == it->x && median.y == it->y )
        continue;

      // w(i) / dist( x(i)-X )
      normalizedWeight = it->weight / it->get2DDistance( median );

      // sum( w(i)x(i) / dist( x(i)-X )
      nominator.x += normalizedWeight * it->x;
      nominator.y += normalizedWeight * it->y;

      // sum( w(i) / dist( x(i)-X )
      denominator += normalizedWeight;

    }

    // new weighted median estimate = nominator / denominator = sum( w(i)x(i) / dist( x(i)-X ) / sum( w(i) / dist( x(i)-X )
    newMedianEstimate.x = nominator.x / denominator;
    newMedianEstimate.y = nominator.y / denominator;

    // update error
    error = median.get2DDistance( newMedianEstimate );
    // update the estimate
    median = newMedianEstimate;
    // increment iteration counter
    ++iteration;
  }
}


// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
kldInitialize( double mapMinX,  double mapMaxX,
               double mapMinY,  double mapMaxY,
               double mapMinZ,  double mapMaxZ,
               double binWidth, double binAngle,
               uint   minimalNumberOfParticles,
               uint   maximalNumberOfParticles,
               double delta /*= 0.99*/, double epsilon /*= 0.05*/
               )
// --------------------------------------------------------------------------
{
//  std::cerr << "WARNING: kldInitialize NOT YET IMPLEMENTED IN PF 2 !!" << std::endl;
//  exit( EXIT_FAILURE );

  m_dKLDSamplingBinWidth = binWidth;
  m_dKLDSamplingBinAngle = binAngle;

  m_dKLDSamplingMinX = mapMinX;
  m_dKLDSamplingMinY = mapMinY;
  m_dKLDSamplingMinZ = mapMinZ;

  assert( mapMaxX > mapMinX );
  assert( mapMaxY > mapMinY );
  assert( mapMaxZ > mapMinZ );
  assert( m_dKLDSamplingBinWidth > 0.0 );
  assert( m_dKLDSamplingBinAngle > 0.0 );

  m_iKLDSamplingMapWidth     = (int)((mapMaxX - mapMinX) / m_dKLDSamplingBinWidth) + 1;
  m_iKLDSamplingMapHeight    = (int)((mapMaxY - mapMinY) / m_dKLDSamplingBinWidth) + 1;
  m_iKLDSamplingMapDepth     = (int)((mapMaxZ - mapMinZ) / m_dKLDSamplingBinWidth) + 1;
  m_iKLDSamplingOrientations = (int)(2*M_PI / m_dKLDSamplingBinAngle) + 1;

  m_uKLDMinNumberOfParticles = minimalNumberOfParticles;
  m_uKLDMaxNumberOfParticles = maximalNumberOfParticles;

  m_dKLDSamplingDelta = delta;
  m_dKLDEpsilon       = epsilon;

  assert( m_dKLDSamplingDelta > 0 );
  assert( m_dKLDEpsilon       > 0 );

  m_vKLDSamplingBins.resize( m_iKLDSamplingMapWidth * m_iKLDSamplingMapHeight * m_iKLDSamplingMapDepth * m_iKLDSamplingOrientations, false );

  // Find upper (1-delta) quantile of the standard normal N(0,1) distribution
  double p_accumulated = 0.0;
  double p_x;
  double f = 1.0 / ( sqrt(2*M_PI) * 1.0 /*=sigma*/ );
  double termination_p = (1 - m_dKLDSamplingDelta) / f;
  double integration_step = 0.05;

  // For integration, start with a large x far beyond the relevant
  // standard deviations
  {
    double x;
    for ( x = 100.0; p_accumulated < termination_p; x -= integration_step )
    {
      p_x = exp( - 0.5 * x*x ) * integration_step;
      p_accumulated += p_x;
    }
    m_dKLDSamplingUpperQuantile = x + integration_step/2;
  }

//   cout << "debug: m_dKLDSamplingUpperQuantile = " << m_dKLDSamplingUpperQuantile << endl;


  // Precompute KL bounds /////////////////////////////////
  if ( m_iKLBounds != NULL )
    delete m_iKLBounds;
  m_iKLBounds = new int[ m_uKLDMaxNumberOfParticles ];
  m_iKLBounds[0] = 0;

  double a, b;
  for ( int k = 1; k < m_uKLDMaxNumberOfParticles; ++k )
  {
    a = 2 / (9*k);
    b = 1.0 - a + sqrt(a) * m_dKLDSamplingUpperQuantile;
    b *= (b*b); // b^3
    m_iKLBounds[k] = (int) ( k/(2*m_dKLDEpsilon) * b );

    //std::cout << "debug: kl bound " << k << ": " << m_iKLBounds[k] << std::endl;
  }
#if 0
  // NOTE: original formula/computation was the following, but it does not
  //       make sense for k=1 after drawing the first sample!
    a = 2 / (9*(k-1));
    b = 1.0 - a + sqrt(a) * m_dKLDSamplingUpperQuantile;
    b *= (b*b); // b^3
    m_iKLBounds[k] = (int) ( (k-1)/(2*epsilon) * b );
#endif
  // //////////////////////////////////////////////////////
}




// --------------------------------------------------------------------------
void pf2::ParticleFilter6D::
    kldSample( LikelihoodFunction likelihoodFunction, void* sensorData )
// --------------------------------------------------------------------------
{
  int angle_multiplier;
  std::list< int > bins_with_support;

  int k = 1; // number of bins with support
  int n_samples = 0; // number of sampled particles so far
  int kl_bound = 1; // initial bound
  int bx, by, bz, btheta; // bin indices of new sample
  int bin_index;

  SampleSet kldParticlesTemp;

  register double accumulated_weight = 0;

  ++state.numberOfResamplings;

  assert( isNormalized() );
  assert( m_iKLDSamplingMapWidth     > 0 );
  assert( m_iKLDSamplingMapHeight    > 0 );
  assert( m_iKLDSamplingMapDepth     > 0 );
  assert( m_iKLDSamplingOrientations > 0 );

  // needs to be called for random sample drawing
  updateAccumulatedWeights();

  // Draw random samples and apply likelihood function until
  // KL bound is reached
  int samples_outside_bins = 0;
  do
  {
    // Draw new sample
    const Particle6D & whichParticle = getParticleRandomly();
    Particle6D newSample = whichParticle;

    // Evaluate likelihood function
    newSample.weight = likelihoodFunction( &newSample, sensorData );
#ifdef HAVE_LOG_WEIGHTS
    newSample.log_weight = log( newSample.weight );
#endif
    accumulated_weight += newSample.weight;

    // Normalize angle for computation of bin indices
    if ( newSample.theta_z < 0 || newSample.theta_z >= 2*M_PI )
    {
      angle_multiplier    = (int) (newSample.theta_z / (2*M_PI));

      // Correct ange multiplier for the case that newSample.theta_z < 0
      if ( newSample.theta_z < 0 )
        --angle_multiplier;

      newSample.theta_z -= ( angle_multiplier * (2 * M_PI) );
    }

    // Compute bin indices
    bx     = (int) ((newSample.x - m_dKLDSamplingMinX) / m_dKLDSamplingBinWidth );
    by     = (int) ((newSample.y - m_dKLDSamplingMinY) / m_dKLDSamplingBinWidth );
    bz     = (int) ((newSample.z - m_dKLDSamplingMinZ) / m_dKLDSamplingBinWidth );
    btheta = (int) ( newSample.theta_z / m_dKLDSamplingBinAngle );

    // Check whether new sample falls into empty bin
    if (    bx >= 0     && bx < m_iKLDSamplingMapWidth
         && by >= 0     && by < m_iKLDSamplingMapHeight
         && bz >= 0     && bz < m_iKLDSamplingMapDepth
         && btheta >= 0 && btheta < m_iKLDSamplingOrientations )
    {
      bin_index = bx +
                  by * m_iKLDSamplingMapWidth +
                  bz * ( m_iKLDSamplingMapWidth * m_iKLDSamplingMapHeight ) +
                  btheta * ( m_iKLDSamplingMapWidth * m_iKLDSamplingMapHeight * m_iKLDSamplingMapDepth );

      if ( ! m_vKLDSamplingBins[ bin_index ]
           && kl_bound < m_uKLDMaxNumberOfParticles )
      {
        m_vKLDSamplingBins[ bin_index ] = true;
        ++k;
        bins_with_support.push_back( bin_index );

        kl_bound = m_iKLBounds[ k ];

        if ( kl_bound > m_uKLDMaxNumberOfParticles )
        {
          kl_bound = m_uKLDMaxNumberOfParticles;
        }
        else if ( kl_bound < m_uKLDMinNumberOfParticles )
        {
          kl_bound = m_uKLDMinNumberOfParticles;
        }
      }

      ++n_samples;
    }
    else
    {
      ++samples_outside_bins;
    }

    kldParticlesTemp.push_back( newSample );

  } while ( n_samples < kl_bound
            && n_samples + samples_outside_bins < m_uKLDMaxNumberOfParticles );

  m_uKLDNumberOfBinsWithSupport = k;

  double duration = -carmen_get_time();
  // Reset sampling bins.
  // TODO: Potential optimization: memset with 0
  for ( std::list<int>::iterator it = bins_with_support.begin();
        it != bins_with_support.end();
        ++it )
  {
    m_vKLDSamplingBins[ *it ] = false;
  }
  duration += carmen_get_time();

  // Swap actual and temporary data
  assert( kldParticlesTemp.size() > 0 );
  particles.swap( kldParticlesTemp );
  if(m_vAccumulatedWeights.size() != particles.size())
    m_vAccumulatedWeights.resize( particles.size() );

  // Normalize ////////////////////////
  // (done here because norm. factor is known)
  double normalization_factor = 1.0 / accumulated_weight;
  particles[0].weight *= normalization_factor;
  m_vAccumulatedWeights[0] = particles[0].weight;
  for ( int i = 1; i < particles.size(); ++i )
  {
    particles[i].weight             *= normalization_factor;
#ifdef HAVE_LOG_WEIGHTS
    particles[i].log_weight         = log(particles[i].weight);
#endif
    m_vAccumulatedWeights[i]        = m_vAccumulatedWeights[i-1] + particles[i].weight;
  }
  state.sumOfWeights = m_vAccumulatedWeights[m_vAccumulatedWeights.size()-1];
  // //////////////////////////////////
  // Postconditions
  state.normalized       = true;

  state.effectiveSampleSize = particles.size();
  state.essComputed         = true;
}

// --------------------------------------------------------------------------
pf2::Particle6D pf2::ParticleFilter6D::
getClosestParticle( const pf2::Particle6D& p )
// --------------------------------------------------------------------------
{
  if( particles.empty() )
    return Particle6D();
  Particle6D& minParticle = particles[0];
  double minDist = minParticle.get3DDistance( p );
  for( int i = 1; i < particles.size(); ++i ) {
    const Particle6D& curParticle = particles[i];
    const double curDist = curParticle.get3DDistance( p );
    if( minDist > curDist ) {
      minDist = curDist;
      minParticle = curParticle;
    }
  }
  return minParticle;
}


// --------------------------------------------------------------------------
std::ostream & pf2::operator << ( std::ostream & ostr, const pf2::ParticleFilter6D & pf )
// --------------------------------------------------------------------------
{
  for ( std::vector< pf2::Particle6D >::const_iterator it = pf.particles.begin(); it != pf.particles.end(); ++it )
  {
    ostr << (*it) << "\n";
  }
  return ostr;
}
