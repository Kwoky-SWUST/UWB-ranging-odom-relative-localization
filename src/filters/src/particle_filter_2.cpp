#include <iostream>
#include <cmath>
#include <list>
#include <numeric>
#include <assert.h>
#include <Eigen/Eigen>
#include "particle_filter_2.h"
using namespace Eigen;
// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
pf2::ParticleFilter< PARTICLE_TYPE >::
    ParticleFilter()
// --------------------------------------------------------------------------
{
//test here
  Eigen::Isometry3d M=Eigen::Isometry3d::Identity();

  reset();
  m_bKLDSamplingRequired = true;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    copyFrom( const pf2::ParticleFilter< PARTICLE_TYPE >& src )
// --------------------------------------------------------------------------
{
  reset();
  particles.reserve( src.size() );
  const std::vector< PARTICLE_TYPE >& v = src.particles;
  particles.assign( v.begin(), v.end() );
  normalize();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    clear()
// --------------------------------------------------------------------------
{
  particles.clear();
  m_bKLDSamplingRequired = true;
  reset();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    reset()
// --------------------------------------------------------------------------
{
  state.normalized         = false;
  state.essComputed        = false;
  state.meanComputed       = false;

  state.mostLikelyParticle = -1; // invalidation as initialization
  state.sumOfWeights       = 0.0;
  state.maxWeight          = 0.0;
  state.minWeight          = INFINITY;

  m_vKLDSamplingBins.clear();
  m_vKLDSamplingBins.resize( 0, false );
  m_dKLDEpsilon = 0.05;
  m_iKLBounds = NULL;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline void pf2::ParticleFilter< PARTICLE_TYPE>::
    addParticle( PARTICLE_TYPE s )
// --------------------------------------------------------------------------
{
  s.parent = -1;
  particles.push_back( s );
  reset();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline void pf2::ParticleFilter< PARTICLE_TYPE>::
    save()
// --------------------------------------------------------------------------
{
  backupParticles = particles;
  backupState     = state;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline void pf2::ParticleFilter< PARTICLE_TYPE>::
    restore()
// --------------------------------------------------------------------------
{
  particles = backupParticles;
  state     = backupState;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
double pf2::ParticleFilter< PARTICLE_TYPE >::
    getCoefficientOfVariation()
// --------------------------------------------------------------------------
{
  getEffectiveSampleSize();
  return state.coefficientOfVariation;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
uint pf2::ParticleFilter< PARTICLE_TYPE >::
    getEffectiveSampleSize()
// --------------------------------------------------------------------------
{
  if ( size() < 2 )
    return size();

  assert( state.normalized );

  if ( ! state.essComputed )
  {
    double sum_of_sqr_weights = 0.0;
    double variation_sum      = 0.0;
    typedef typename std::vector< PARTICLE_TYPE >::const_iterator Iter;
    for ( Iter it = particles.begin(); it != particles.end(); ++it )
    {
      sum_of_sqr_weights += carmen_square( it->weight );
      variation_sum += carmen_square( ( size() * it->weight ) - 1.0 );
    }

    // NOTE: In the following line, the 0.5 is only required for rounding
    state.effectiveSampleSize = (uint) (1.0 / sum_of_sqr_weights + 0.5);
    state.coefficientOfVariation = variation_sum / size();
    state.essComputed = true;
  }

  return state.effectiveSampleSize;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline int pf2::ParticleFilter< PARTICLE_TYPE >::
    getMostLikelyParticle() const
// --------------------------------------------------------------------------
{
  assert( state.normalized );
  return state.mostLikelyParticle;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    determineMinimalAndMaximalParticleWeights( double& minWeight, double& maxWeight )
// --------------------------------------------------------------------------
{
  assert( state.normalized );
  minWeight = INFINITY;
  maxWeight = -INFINITY;
  for ( uint u = 0; u < size(); ++u )
  {
    if ( particles[u].weight < minWeight )
      minWeight = particles[u].weight;
    if ( particles[u].weight > maxWeight )
      maxWeight = particles[u].weight;
  }
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline bool pf2::ParticleFilter< PARTICLE_TYPE >::
    isNormalized() const
// --------------------------------------------------------------------------
{
  return state.normalized;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
uint pf2::ParticleFilter< PARTICLE_TYPE >::
    size() const
// --------------------------------------------------------------------------
{
  return particles.size();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
inline void pf2::ParticleFilter< PARTICLE_TYPE >::
    resample( const uint numberOfParticles,
              const ResamplingMethod method )
// --------------------------------------------------------------------------
{
  resample( numberOfParticles, 1.0/numberOfParticles, method );
  if ( ! isNormalized() )
    normalize();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    resample( const uint numberOfSamples, const double weight,
              const ResamplingMethod method
            )
// --------------------------------------------------------------------------
{
  assert( state.normalized );
  ++state.numberOfResamplings;

  if ( method == SYSTEMATIC_RESAMPLING )
    resampleUsingSystematicResampling( numberOfSamples, weight );
  else if ( method == RESIDUAL_RESAMPLING )
    resampleUsingResidualResampling( numberOfSamples, weight );
  else
    resampleUsingStratifiedResampling( numberOfSamples, weight );

  // Postconditions
  state.normalized          = ( weight*numberOfSamples == 1.0 ) ? true : false;
  state.effectiveSampleSize = numberOfSamples;
  state.essComputed         = true;
  state.meanComputed        = false;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
        resampleUsingStratifiedResampling( const uint   numberOfSamples,
                                           const double weight )
// --------------------------------------------------------------------------
{
  m_vSwapParticles.resize( numberOfSamples );

#ifdef HAVE_LOG_WEIGHTS
  double log_weight = log( weight );
#endif

  // Although the sample set is normalized, we have to use the last
  // accumulated weight because of floating point number problems
  // (accumulated weight may be close to, but unequal to 1.0)
  double weight_sum = 0.0;
  double accumulated_weight[ size() ];
  for ( uint u = 0; u < particles.size(); ++u )
  {
    weight_sum += particles[u].weight;
    accumulated_weight[u] = weight_sum;
  }

  register double position;
  register double interval_base_pos = 0.0;
  register double step_size = weight_sum / size();

  register int sampled_index = 0;

  // Draw num_particles random samples
  for ( uint i = 0; i < numberOfSamples; i++ )
  {
    position = interval_base_pos + carmen_uniform_random( 0.0, step_size );

    // Avoid marginally likely case that position exceeds
    // maximum accumulated weights
    if ( position > weight_sum )
      position = weight_sum;

    while ( position > accumulated_weight[ sampled_index ] )
      ++sampled_index;

    m_vSwapParticles[i] = particles[ sampled_index ];
    m_vSwapParticles[i].parent = sampled_index;
    m_vSwapParticles[i].weight = weight;
#ifdef HAVE_LOG_WEIGHTS
    m_vSwapParticles[i].log_weight = log_weight;
#endif

    interval_base_pos += step_size;
  }

  // Swap actual and temporary data
  particles.swap( m_vSwapParticles );
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    resampleUsingSystematicResampling( const uint   numberOfSamples,
                                       const double weight )
// --------------------------------------------------------------------------
{
  m_vSwapParticles.resize( numberOfSamples );

#ifdef HAVE_LOG_WEIGHTS
  double log_weight = log( weight );
#endif

  register int sampled_index = 0;

  // Although the sample set is normalized, we have to use the last
  // accumulated weight because of floating point number problems
  // (accumulated weight may be close to, but unequal to 1.0)
  double weight_sum = 0.0;
  double accumulated_weight[ size() ];
  for ( uint u = 0; u < particles.size(); ++u )
  {
    weight_sum += particles[u].weight;
    accumulated_weight[u] = weight_sum;
  }

  // Choose random starting position for low-variance walk
  register double position = carmen_uniform_random( 0, weight_sum );

  register double step_size = weight_sum / size();

  // Draw num_particles random samples
  for ( uint i = 0; i < numberOfSamples; i++ )
  {
    position += step_size;
    if ( position >= weight_sum )
    {
      position -= weight_sum;
      sampled_index = 0;
    }

    while ( position > accumulated_weight[ sampled_index ] )
      ++sampled_index;

    m_vSwapParticles[i] = particles[ sampled_index ];
    m_vSwapParticles[i].parent = sampled_index;
    m_vSwapParticles[i].weight = weight;
#ifdef HAVE_LOG_WEIGHTS
    m_vSwapParticles[i].log_weight = log_weight;
#endif
  }

  // Swap actual and temporary data
  particles.swap( m_vSwapParticles );
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    resampleUsingResidualResampling( const uint   numberOfSamples,
                                     const double weight )
// --------------------------------------------------------------------------
{
//   std::cout << "RESIDUAL_RESAMPLING" << std::endl;
  assert( size() > 0 );

  if ( m_vSwapParticles.size() < numberOfSamples )
    m_vSwapParticles.resize( numberOfSamples );
  if ( m_vAccumulatedWeights.size() < size() )
    m_vAccumulatedWeights.resize( size() );
  static std::vector<int> num_copies;
  if ( num_copies.size() < size() )
    num_copies.resize( size() );

#ifdef HAVE_LOG_WEIGHTS
  double log_weight = log( weight );
#endif

  // //////////////////////////////////////////////////////
  // Deterministic part: Resampling according to rounded proportion
  // of weight to required number of samples
  int    v, sum_copies = 0;
  ParticleIterator new_particle_iter = m_vSwapParticles.begin();
  for ( uint u = 0; u < particles.size(); ++u )
  {
//     num_copies = (int) floor( particles[u].weight * numberOfSamples );
    num_copies[u] = lrint( particles[u].weight * numberOfSamples - 0.5 ); // NOTE: faster!
    if ( num_copies[u] != 0 )
    {
//       std::cout << num_copies[u] << " copies of " << u << " @ " << particles[u].weight << std::endl;
      sum_copies += num_copies[u];
      for ( v = 0; v < num_copies[u]; ++v )
      {
#ifdef HAVE_PARTICLE_TRAJECTORY
        *new_particle_iter = particles[u]; // Note: Extensive copying should be avoided (expensive operation)
#else
        memcpy( &(*new_particle_iter), &particles[u], sizeof(PARTICLE_TYPE) ); // Note: Extensive copying should be avoided (expensive operation)
#endif
        new_particle_iter->parent = u;
        new_particle_iter->weight = weight;
#ifdef HAVE_LOG_WEIGHTS
        new_particle_iter->log_weight = log_weight;
#endif
        ++new_particle_iter;
      }
    }
  }
  // //////////////////////////////////////////////////////

  if ( sum_copies < (int)numberOfSamples )
  {
    // //////////////////////////////////////////////////////
    // Compute remainder weights. This is bit tricky, because we need to
    // deal with numerical problems
    double f = 1.0 / ( numberOfSamples - sum_copies );
    double weight_sum = 0.0;
    if ( sum_copies > 0 )
    {
  #ifdef HAVE_LOG_WEIGHTS
      double max_log_weight = -INFINITY;
  #endif
      assert( ! ::isnan( f ) );
      assert( ! ::isinf( f ) );
      for ( uint u = 0; u < particles.size(); ++u )
      {
        particles[u].weight = ( particles[u].weight*size() - num_copies[u] ) * f;
        assert( ! ::isnan( particles[u].weight ) );
        assert( ! ::isinf( particles[u].weight ) );

        // Now it may have happened that the new weight is slightly
        // below zero (numerical reasons)
        if ( particles[u].weight < 0 )
          particles[u].weight = 0;

  #ifdef HAVE_LOG_WEIGHTS
        particles[u].log_weight = log( particles[u].weight );
        if ( ( ! isinf( max_log_weight ) && ! isinf( particles[u].log_weight ) && particles[u].log_weight > max_log_weight ) // particle log weight is valid and greater than previous max. log weight
                || ( isinf( max_log_weight ) && ! isinf( particles[u].log_weight ) ) // particle log weight is first valid one
          )
        {
          max_log_weight = particles[u].log_weight;
        }
  #else
        weight_sum += particles[u].weight;
  #endif
      }

  #ifdef HAVE_LOG_WEIGHTS
      // Divide by maximal (log) weight for better numerical stability
      if ( ! isinf( max_log_weight ) )
      {
        for ( uint u = 0; u < particles.size(); ++u )
        {
          particles[u].log_weight -= max_log_weight;
          if ( particles[u].log_weight == -INFINITY ) {
            particles[u].weight = 0.0;
          } else {
            particles[u].weight = exp( particles[u].log_weight );
            assert( ! isnan( particles[u].weight ) );
          }
          weight_sum += particles[u].weight;
        }
      }
  #endif

      if ( weight_sum > 0 )
      {
        f = 1.0 / weight_sum;
        weight_sum = 0.0;
        for ( uint u = 0; u < particles.size(); ++u )
        {
          particles[u].weight *= f;
          weight_sum += particles[u].weight;
          m_vAccumulatedWeights[u] = weight_sum;
        }
      }
      else
      {
        // Reset particle weights in the case that they are all zero
        weight_sum = 0.0;
        f = 1.0 / particles.size();
        for ( uint u = 0; u < particles.size(); ++u )
        {
          particles[u].weight = f;
          weight_sum += particles[u].weight;
          m_vAccumulatedWeights[u] = weight_sum;
        }
      }
    }
    else // in this case: sum_copies == 0
    {
      weight_sum = 0.0;
      for ( uint u = 0; u < particles.size(); ++u )
      {
        weight_sum += particles[u].weight;
        m_vAccumulatedWeights[u] = weight_sum;
      }

      // Reset particle weights for the case that they are all zero
      if ( weight_sum == 0.0 )
      {
        weight_sum = 0.0;
        f = 1.0 / particles.size();
        for ( uint u = 0; u < particles.size(); ++u )
        {
          particles[u].weight = f;
          weight_sum += particles[u].weight;
          m_vAccumulatedWeights[u] = weight_sum;
        }
      }
    }
    // //////////////////////////////////////////////////////

    // Choose random starting position for low-variance walk
    register double position  = carmen_uniform_random( 0, weight_sum );
    register double step_size = weight_sum / size();

    // Draw num_particles random samples
    register int sampled_index = 0;
    while ( new_particle_iter != m_vSwapParticles.end() )
    {
      position += step_size;
      if ( position >= weight_sum )
      {
        position -= weight_sum;
        sampled_index = 0;
      }

      while ( position > m_vAccumulatedWeights[ sampled_index ] )
        ++sampled_index;

      *new_particle_iter = particles[ sampled_index ];
      new_particle_iter->parent = sampled_index;
      new_particle_iter->weight = weight;
  #ifdef HAVE_LOG_WEIGHTS
      new_particle_iter->log_weight = log_weight;
  #endif

      ++new_particle_iter;
    }
  }

  // Swap actual and temporary data
  particles.swap( m_vSwapParticles );
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    reweight( LikelihoodFunction lf, void* data )
// --------------------------------------------------------------------------
{
  double likelihood;
  typedef typename std::vector< PARTICLE_TYPE >::iterator Iter;
  for ( Iter it = particles.begin(); it != particles.end(); ++it )
  {
    likelihood = lf( &(*it), data );
    it->weight *= likelihood;
#ifdef HAVE_LOG_WEIGHTS
    it->log_weight += log(likelihood);
#endif
  }
}

// --------------------------------------------------------------------------
template< class PARTICLE_TYPE>
void pf2::ParticleFilter<PARTICLE_TYPE>::
    copyFrom( const std::vector<PARTICLE_TYPE> src )
// --------------------------------------------------------------------------
{
  reset();
  particles.reserve( src.size() );
  particles.assign( src.begin(), src.end() );
  normalize();
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    normalize()
// --------------------------------------------------------------------------
{
//   std::cout << "*** PSO::normalize" << std::endl;

  double _w, _max = 0.0;
  double sum_of_weights = 0.0;
  int    zeroCount; // number of particles with weight zero
  for ( uint u = 0; u < particles.size(); ++u )
  {
    _w = particles[u].weight;

    if ( _w > 0.0 )
    {
      if ( _w > _max )
      {
        _max = _w;
        state.mostLikelyParticle = u;
      }

      sum_of_weights += _w;
    }
    else
      ++zeroCount;

    if ( _w < state.minWeight )
      state.minWeight = _w;

  }

  // Remember accumulated belief for statistical purposes
  state.sumOfWeights = sum_of_weights;

  // Remember maximum of all beliefs for statistical purposes
  state.maxWeight = _max;

  if ( sum_of_weights > 0.0 )
  {
    double inverse_eta = 1.0 / sum_of_weights;
#ifdef HAVE_LOG_WEIGHTS
    double log_inverse_eta = log( inverse_eta );
#endif
    for ( ParticleIterator it = particles.begin(); it != particles.end(); ++it )
    {
      it->weight *= inverse_eta;
#ifdef HAVE_LOG_WEIGHTS
      it->log_weight += log_inverse_eta;
#endif
    }

    state.maxWeight *= inverse_eta;
    state.minWeight *= inverse_eta;
  }
  else
  {
    std::cerr << "ParticleFilter: WARNING: All particles accumulate to belief of zero. Setting all sample importance values 1/n." << std::endl;
    state.maxWeight = 1.0 / size();
    typedef typename std::vector< PARTICLE_TYPE >::iterator Iter;
    for ( Iter it = particles.begin(); it != particles.end(); ++it )
    {
      it->weight = state.maxWeight;
    }
  }

  state.normalized       = true;
  state.essComputed      = false;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    normalizeWithLogWeights()
// --------------------------------------------------------------------------
{
#ifdef HAVE_LOG_WEIGHTS
  double    _log_w, _max = -INFINITY;

  for ( uint u = 0; u < particles.size(); ++u )
  {
    _log_w = particles[u].log_weight;
    assert( ! isinf( _log_w ) );
    assert( ! isnan( _log_w ) );

    if ( _log_w > _max )
    {
      _max = _log_w;
    }
  }

  for ( uint u = 0; u < particles.size(); ++u )
  {
    particles[u].log_weight -= _max;
    particles[u].weight      = exp( particles[u].log_weight );
  }
  // NOTE: Further normalization is still required! The step
  // above is just for numerical stability
#endif

  normalize();
}

// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
int pf2::ParticleFilter< PARTICLE_TYPE >::
    getParticleIndexRandomly() const
// --------------------------------------------------------------------------
{
  // Get random number between 0.0 and 1.0
//   const double random = m_pUniformDistribution->getRandomNumber();
//   const double random = carmen_uniform_random( 0, 1 );

  assert( m_vAccumulatedWeights.size() == particles.size() );

  const double weight_sum = m_vAccumulatedWeights[ m_vAccumulatedWeights.size()-1 ];
  const double random = carmen_uniform_random( 0, weight_sum );

  const int    _size = size();
  register int min = 0, half;
  register int len = _size;

  // Binary search for sample whose accumulated probability fits the uniform random value
  while ( len > 0 )
  {
    half = len / 2;

    if ( m_vAccumulatedWeights[ min + half ] > random )
    {
      // step into left half of array
      len = half;
    }
    else
    {
      min +=  half+1;      // step into right half of array
      len -= (half+1);
    }
  }

  if ( min == _size )
    --min;

  return min;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
const PARTICLE_TYPE & pf2::ParticleFilter< PARTICLE_TYPE >::
    getParticleRandomly() const
// --------------------------------------------------------------------------
{
  return particles[ getParticleIndexRandomly() ];
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
   kldInitialize( double mapMinX,  double mapMaxX,
                  double mapMinY,  double mapMaxY,
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

  assert( mapMaxX > mapMinX );
  assert( mapMaxY > mapMinY );
  assert( m_dKLDSamplingBinWidth > 0.0 );
  assert( m_dKLDSamplingBinAngle > 0.0 );

  m_iKLDSamplingMapWidth     = (int)((mapMaxX - mapMinX) / m_dKLDSamplingBinWidth) + 1;
  m_iKLDSamplingMapHeight    = (int)((mapMaxY - mapMinY) / m_dKLDSamplingBinWidth) + 1;
  m_iKLDSamplingOrientations = (int)(2*M_PI / m_dKLDSamplingBinAngle) + 1;

  m_uKLDMinNumberOfParticles = minimalNumberOfParticles;
  m_uKLDMaxNumberOfParticles = maximalNumberOfParticles;

  m_dKLDSamplingDelta = delta;
  m_dKLDEpsilon       = epsilon;

  assert( m_dKLDSamplingDelta > 0 );
  assert( m_dKLDEpsilon       > 0 );

  m_vKLDSamplingBins.resize( m_iKLDSamplingMapWidth * m_iKLDSamplingMapHeight * m_iKLDSamplingOrientations, false );

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
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    kldSample( LikelihoodFunction likelihoodFunction, void* sensorData )
// --------------------------------------------------------------------------
{
//  std::cerr << "WARNING: kldSample NOT YET IMPLEMENTED IN PF 2 !!" << std::endl;
//  exit( EXIT_FAILURE );

  int angle_multiplier;
  std::list< int > bins_with_support;

  int k = 1; // number of bins with support
  int n_samples = 0; // number of sampled particles so far
  int kl_bound = 1; // initial bound
  int bx, by, btheta; // bin indices of new sample
  int bin_index;

  SampleSet kldParticlesTemp;

  register double accumulated_weight = 0;

  ++state.numberOfResamplings;

  assert( isNormalized() );
  assert( m_iKLDSamplingMapWidth     > 0 );
  assert( m_iKLDSamplingMapHeight    > 0 );
  assert( m_iKLDSamplingOrientations > 0 );

  // needs to be called for random sample drawing
  updateAccumulatedWeights();

  // Draw random samples and apply likelihood function until
  // KL bound is reached
  int samples_outside_bins = 0;
  do
  {
    // Draw new sample
    const PARTICLE_TYPE & whichParticle = getParticleRandomly();
    PARTICLE_TYPE newSample = whichParticle;

    // Evaluate likelihood function
    newSample.weight = likelihoodFunction( &newSample, sensorData );

#ifdef HAVE_LOG_WEIGHTS
    newSample.log_weight = log( newSample.weight );
#endif
    accumulated_weight += newSample.weight;

    // Normalize angle for computation of bin indices
    if ( newSample.theta < 0 || newSample.theta >= 2*M_PI )
    {
      angle_multiplier    = (int) (newSample.theta / (2*M_PI));

      // Correct ange multiplier for the case that newSample->theta < 0
      if ( newSample.theta < 0 )
        --angle_multiplier;

      newSample.theta -= ( angle_multiplier * (2 * M_PI) );
    }

    // Compute bin indices
    bx     = (int) ((newSample.x - m_dKLDSamplingMinX) / m_dKLDSamplingBinWidth );
    by     = (int) ((newSample.y - m_dKLDSamplingMinY) / m_dKLDSamplingBinWidth );
    btheta = (int) ( newSample.theta / m_dKLDSamplingBinAngle );

    // Check whether new sample falls into empty bin
    if (    bx >= 0     && bx < m_iKLDSamplingMapWidth
         && by >= 0     && by < m_iKLDSamplingMapHeight
         && btheta >= 0 && btheta < m_iKLDSamplingOrientations )
    {
      bin_index = bx + by*m_iKLDSamplingMapWidth + btheta * ( m_iKLDSamplingMapWidth * m_iKLDSamplingMapHeight );

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
template< class PARTICLE_TYPE >
inline uint pf2::ParticleFilter< PARTICLE_TYPE >::
    getKLDNumberOfBinsWithSupport() const
// --------------------------------------------------------------------------
{
  return m_uKLDNumberOfBinsWithSupport;
}


// --------------------------------------------------------------------------
template< class PARTICLE_TYPE >
void pf2::ParticleFilter< PARTICLE_TYPE >::
    updateAccumulatedWeights()
// --------------------------------------------------------------------------
{
  const unsigned num_particles = particles.size();
  if(num_particles == 0) return;

  // resize vector if necessary
  if( m_vAccumulatedWeights.size() != num_particles ) {
    m_vAccumulatedWeights.resize( num_particles );
  }

  // compute accumulated weights
  m_vAccumulatedWeights[0] = particles[0].weight;
  for(int i=1; i<particles.size(); ++i)
    m_vAccumulatedWeights[i] = m_vAccumulatedWeights[i-1] + particles[i].weight;

//  // normalize accumulated weights with weight sum
//  for(int i=0; i<particles.size(); ++i)
//    m_vAccumulatedWeights[i] /= m_vAccumulatedWeights[num_particles-1];
}
