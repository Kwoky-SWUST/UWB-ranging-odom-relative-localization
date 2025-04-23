#ifndef AMBISENSE_SLAM__FILTERS__PARTICLE_FILTER_2_H
#define AMBISENSE_SLAM__FILTERS__PARTICLE_FILTER_2_H

#include <vector>
#include <ros/ros.h>
#include "carmen.h"

namespace pf2
{

  /** Statistics about the current state of the particle filter
   *  @author Philipp Vorst
   */
  // ------------------------------------------------------------------------
  struct ParticleFilterState
  // ------------------------------------------------------------------------
  {
    /// Has the particle filter been normalized since the last reset?
    bool normalized;

    /// The effective particle filter size
    /// @see getEffectiveSampleSize, m_bESSComputed
    uint effectiveSampleSize;

    /// Has the effective particle filter size been computed since the last reset?
    /// @see getEffectiveParticleSetSize
    bool essComputed;

    /// Coefficient of variation by Liu et al.
    /// @see getCoefficientOfVariation
    double coefficientOfVariation;

    /// Has the mean been computed since the last update?
    bool   meanComputed;

    /// The index of the most likely particle within the array of particles
    int    mostLikelyParticle;

    /// The recent accumulated belief for all particles in this set
    double sumOfWeights;

    /// The maximum of all beliefs of the particles in this set
    double maxWeight;

    /// The minimum of all beliefs of the particles in this set
    double minWeight;

    /// Resampling counter
    uint numberOfResamplings;
  };

  /** Particle filter.
   *  PARTICLE_TYPE can be any data structure containing a double-valued
   *  field 'weight'.
   *  @author Philipp Vorst
   */
  // ------------------------------------------------------------------------
  template< class PARTICLE_TYPE >
  class ParticleFilter
  // ------------------------------------------------------------------------
  {
    public:
      typedef std::vector< PARTICLE_TYPE >                    SampleSet;
      typedef typename std::vector< PARTICLE_TYPE >::iterator ParticleIterator;
      typedef double (*LikelihoodFunction)( PARTICLE_TYPE*, void* );

      /** Resampling techniques. A comparison is given in Douc/Cappé/Moulines:
       *  "Comparison of Resampling Schemes for Particle Filtering"
       *  (in ISPA 2005)
       *  @see resample, resampleFraction, resampleUsingResidualResampling,
       *       resampleUsingStratifiedResampling, resampleUsingSystematicResampling
       */
      enum ResamplingMethod {
        SYSTEMATIC_RESAMPLING = 0,  ///< Faster method (approx. by factor 2 for small structs)
        RESIDUAL_RESAMPLING   = 1,  ///< Potentially best method
        STRATIFIED_RESAMPLING = 2   ///< Slower than SYSTEMATIC_RESAMPLING, but better independence of samples
      };

#define PF2_DEFAULT_RESAMPLING_METHOD  SYSTEMATIC_RESAMPLING

    public:
      /** Constructor */
      ParticleFilter< PARTICLE_TYPE >();

      /** Copy all particles from the given particle filter. This will override
       *  the current samples of this particle filter.
       *   @param src The particle filter to copy the particles from.
       */
      void copyFrom( const std::vector< PARTICLE_TYPE > src );

      /** Remove all particles from the current particle filter */
      void clear();

      /** Insert a sample to the particle filter
       *  @note No referencing!, always 'const PARTICLE_TYPE s'
       */
      inline void addParticle( PARTICLE_TYPE s );

      /** Returns i-th sample. Note: risky access!! */
      PARTICLE_TYPE & operator [] ( const int i )
        { return particles[i]; }

      /** Returns i-th sample (const accessor!). Note: risky access!! */
      const PARTICLE_TYPE & operator [] ( const int i ) const
        { return particles[i]; }

      /** Returns a pointer to the i-th sample. Note: risky access!! */
      PARTICLE_TYPE *at( int i )
        { return &particles[i]; }

      /** Normalize particle filter, i.e. make sum of probs accumulate to 1.
       *  Use the normal weights for this purpose (not log weights).
       *  @see normalizeWithLogWeights
       */
      void normalize();

      /** Normalize particle filter, i.e. make sum of probs accumulate to 1.
       *  Use the logarithmic weights for this purpose.
       *  @see normalize
       */
      void normalizeWithLogWeights();

      /** Have the sample weights been normalized? */
      inline bool isNormalized() const;

      /** Copy all particles from the given particle filter. This will override
         *  the current samples of this particle filter.
         *   @param src The particle filter to copy the particles from.
         */
      void copyFrom( const pf2::ParticleFilter< PARTICLE_TYPE >& src );

      /** Get the number of particles */
      uint size() const;

      /** Re-sample the particle filter.
       *  Precondition: The particle filter has been normalized.
       */
      void resample( const uint numberOfParticles, const double weight,
                     const ResamplingMethod method = PF2_DEFAULT_RESAMPLING_METHOD
                   );

      /** Re-sample particle filter, using residual resampling.
       *  Precondition: The particle filter has been normalized.
       *  @see RESIDUAL_RESAMPLING
       */
      void resampleUsingResidualResampling( const uint numberOfParticles, const double weight );

      /** Re-sample particle filter, using stratified resampling.
       *  Precondition: The particle filter has been normalized.
       *  @see STRATIFIED_RESAMPLING
       */
      void resampleUsingStratifiedResampling( const uint numberOfParticles, const double weight );

      /** Re-sample particle filter, using systematic resampling.
       *  Precondition: The particle filter has been normalized.
       *  @see    SYSTEMATIC_RESAMPLING
       *  @author The implementation goes back to the CARMEN resampling
       *          method used in localizecore.c:carmen_localize_resample(...)
       */
      void resampleUsingSystematicResampling( const uint numberOfParticles, const double weight );

      /** Re-sample particle filter. Precondition: The particle filter has been normalized.
       */
      inline void resample( const uint numberOfParticles,
                            const ResamplingMethod method = PF2_DEFAULT_RESAMPLING_METHOD );

      /** Re-sample specific fraction of current particle filter.
       */
      inline void resampleFraction( const double fractionOfOldParticles,
                                    const ResamplingMethod method = PF2_DEFAULT_RESAMPLING_METHOD )
        { resample( (uint) ( fractionOfOldParticles*size(), 1.0/size() ) ); }

      /** Resample the entire sample set
       */
      inline void resample( const ResamplingMethod method = PF2_DEFAULT_RESAMPLING_METHOD )
        { resample( size(), method ); }

      /** Return the number of particles which effectively contribute to the
       *  probability distribution. This is a measure of the degeneracy of the
       *  particle filter. */
      uint getEffectiveSampleSize();

      /** Return the coefficient of variation, introduced by Liu et al.,
       *  "A theoretical framework...".
       *  Essentially, this value is greater than 1.0 if the ESS drops
       *  below (number of particles)/2, and zero if the ESS equals the
       *  number of particles.
       */
      double getCoefficientOfVariation();

      /** Get the index of the most likely particle */
      inline int getMostLikelyParticle() const;

      /** Determine the smallest and the largest importance weight among
       *  all particles
       */
      void determineMinimalAndMaximalParticleWeights( double& minWeight, double& maxWeight );

      /** Reweight all particles, using the given likelihood function and the given
       *  sensor data */
      void reweight( LikelihoodFunction lf, void* data );

      /** Save a copy of the current particle filter state/ its sample set
       *  @see restore
       */
      void save();

      /** Roll back the recent changes. The particle filter will use the previously
       *  saved sample set.
       *  @see save   */
      void restore();

      /** Return the raw data array. */
      inline SampleSet& getParticles()
        { return particles; }

      /** Draw a sample randomly from particle filter (according to its
       *  weight) and return its index. NOTE: accumulated weights need
       *  to be updated before if the weights changed!
       *  @see updateAccumulatedWeights
       *  @see m_vAccumulatedWeights */
      int getParticleIndexRandomly() const;
      const PARTICLE_TYPE & getParticleRandomly() const;

      /** Perform KD-sampling on the particle filter. */
      void kldSample( LikelihoodFunction fct, void* sensorData );

      /** Prepapre KD sampling. Has to be called once in the beginning,
       *  before the first call to kdSample.
       */
      void kldInitialize( double mapMinX,  double mapMaxX,
                          double mapMinY,  double mapMaxY,
                          double binWidth, double binAngle,
                          uint   minimalNumberOfParticles,
                          uint   maximalNumberOfParticles,
                          double delta = 0.99, double epsilon = 0.05
                        );

      /** Get the number of bins with support.
       *  @see m_iKLDNumberOfBinsWithSupport
       */
      inline uint getKLDNumberOfBinsWithSupport() const;

      /** Update the accumulated weights of the particles
       *  @see m_vAccumulatedWeights
       */
      void updateAccumulatedWeights();


    public:
      SampleSet                     particles;
      ParticleFilterState           state;

      SampleSet                     backupParticles;
      ParticleFilterState           backupState;
      bool                          m_bKLDSamplingRequired;

    protected:
      /** Reset all internal flags */
      void reset();

    protected:
      SampleSet                     m_vSwapParticles;
      std::vector< double >         m_vAccumulatedWeights;

      // KLD sampling parameters ////////////////////////////////

      /// Minimal number of particles
      uint   m_uKLDMinNumberOfParticles;

      /// Maximal number of particles
      uint   m_uKLDMaxNumberOfParticles;

      /// Width of bins in KD sampling
      double m_dKLDSamplingBinWidth;

      /// Angle tolerance for bins in KD sampling
      double m_dKLDSamplingBinAngle;

      double m_dKLDSamplingMinX;
      double m_dKLDSamplingMinY;

      int    m_iKLDSamplingMapWidth;
      int    m_iKLDSamplingMapHeight;
      int    m_iKLDSamplingOrientations;

      double m_dKLDSamplingDelta;
      double m_dKLDEpsilon;

      /** Upper (1-m_dKLDSamplingDelta) quantile of the standard normal
      *  N(0,1) distribution. Precomputed in kldInitialize
      *  @see kldInitialize
      */
      double m_dKLDSamplingUpperQuantile;

      /** Recent number of bins with support */
      uint   m_uKLDNumberOfBinsWithSupport;

      /** Array of flags which state whether or not a bin has support.
      *  @todo A tree would represent a more sophisticated data
      *        structure for this.
      */
      std::vector< bool > m_vKLDSamplingBins;

      /** Precomputed KL bounds */
      int * m_iKLBounds;

  };

}; // end namespace pf2


#include "particle_filter_2.cpp"

#endif // AMBISENSE_SLAM__FILTERS__PARTICLE_FILTER_2_H
