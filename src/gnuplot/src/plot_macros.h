#ifndef CROCO__UTILS__GNUPLOT_MACROS
#define CROCO__UTILS__GNUPLOT_MACROS

#include <vector>
#include <string>
#include <map>
#include <iostream>

// forward decls
class GnuplotInterface;

// container type for plot commands
class GnuPlotCommandTriplet {
  public:
    GnuPlotCommandTriplet () {}
    GnuPlotCommandTriplet( std::string _pre, std::string _plot, std::string _data ) : preAmble(_pre), plotCommand(_plot), data(_data) {}
    GnuPlotCommandTriplet( const GnuPlotCommandTriplet& other ) : preAmble( other.preAmble ), plotCommand( other.plotCommand ), data( other.data ) {}

    // concatenate 2 plots
    const GnuPlotCommandTriplet operator+( const GnuPlotCommandTriplet& other ) const;
    // assignment
    GnuPlotCommandTriplet& operator=( const GnuPlotCommandTriplet& other ) {
      if( this != &other ) {
        this->preAmble = other.preAmble;
        this->plotCommand = other.plotCommand;
        this->data = other.data;
      }
      return *this;
    }
    GnuPlotCommandTriplet& operator+=(const GnuPlotCommandTriplet& other) { *this = *this + other; return *this; }

    std::string getCommandString( const std::string& header = std::string() ) const;

    void plot( const std::string& header = std::string(), GnuplotInterface* plotter = NULL ) const;

    std::string preAmble;
    std::string plotCommand;
    std::string data;
};


void plot_doubles( const std::vector< double >& values, const std::string title = std::string("") );
GnuPlotCommandTriplet get_doubles_plot_triplet( const std::vector< double >& values, const std::string title = std::string("") );

void plot_pairs( const std::vector< std::pair< double, double > >& values, const std::string title = std::string("") );

/// plot a histogram for 1D-data
void plot_histogram( const std::vector< double >& values, unsigned int numBins = 100 );
void plot_cdf( const std::vector< double >& values, unsigned int numBins = 100 );

/// vectors (i.e. Vec2Types) are considered to represent points here!
template< class Vec2Type >
std::string get_vec2_data_string( const std::vector< Vec2Type >& values );
template< class Vec2Type >
std::string get_vec2_plot_string( const std::vector< Vec2Type >& values, std::string title = std::string("") );
template< class Vec2Type >
GnuPlotCommandTriplet get_vec2_plot_triplet( const std::vector< Vec2Type >& values, std::string title = std::string("") );
template< class Vec2Type >
void plot_vec2( const std::vector< Vec2Type >& values, std::string title = std::string("") );
/// vectors (i.e. Vec3Types) are considered to represent points here!
template< class Vec3Type >
std::string get_vec3_data_string( const std::vector< Vec3Type >& values );
template< class Vec3Type >
std::string get_vec3_plot_string( const std::vector< Vec3Type >& values, std::string title = std::string("") );
template< class Vec3Type >
GnuPlotCommandTriplet get_vec3_plot_triplet( const std::vector< Vec3Type >& values, std::string title = std::string("") );
template< class Vec3Type >
void plot_vec3( const std::vector< Vec3Type >& values, std::string title = std::string("") );
template< class Vec3Type >
GnuPlotCommandTriplet get_vec3_contour_plot_triplet( const std::vector< Vec3Type >& values, std::string title = std::string("") );
/// poses supplying (x,y,theta_z) fields, represented as vector field!
template< class Pose3Type >
std::string get_vectorfield_data_string( const std::vector< Pose3Type >& values, const double arrowSize = 0.1 );
template< class Pose3Type >
std::string get_vectorfield_plot_string( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );
template< class Pose3Type >
GnuPlotCommandTriplet get_vectorfield_plot_triplet( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );
template< class Pose3Type >
void plot_vectorfield( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );
/// poses supplying (x,y,theta_z) fields, represented as trajectory
template< class Pose3Type >
std::string get_trajectory_data_string( const std::vector< Pose3Type >& values, const double arrowSize = 0.1 );
template< class Pose3Type >
std::string get_trajectory_plot_string( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );
template< class Pose3Type >
GnuPlotCommandTriplet get_trajectory_plot_triplet( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );
template< class Pose3Type >
void plot_trajectory( const std::vector< Pose3Type >& values, std::string title = std::string(""), const double arrowSize = 0.1 );


typedef std::map< std::string, std::vector< std::pair< double, double > > > PlotSeriesMap;
void plot_several_vectors_of_pairs( PlotSeriesMap & values, GnuplotInterface * gi = NULL );

void plot_several_vectors_of_pairs( std::vector< std::vector< std::pair< double, double > > > values );

/// plot gaussian distribution
std::string get_guassian_data_string( const double mu = 0.,
                                      const double sigma = 1.,
                                      double xmin = -1,
                                      double xmax = -1,
                                      const size_t num_steps = 100 );
std::string get_gaussian_plot_string( const double mu = 0., const double sigma = 1. );
void plot_gaussian( const double mu = 0., const double sigma = 1. );


/// plot binomial distribution
std::string get_binomial_data_string( const unsigned int numTrials,
                                      const double probability = 0.5,
                                      const int offset = 0 );
std::string get_binomial_data_string( const unsigned int numTrials,
                                      const double mu,
                                      const double sigma );
std::string get_binomial_plot_string( const unsigned int numTrials,
                                      const double probability = 0.5,
                                      const int offset = 0 );
std::string get_binomial_plot_string( const unsigned int numTrials,
                                      const double mu,
                                      const double sigma );
void plot_binomial( const unsigned int numTrials,
                    const double probability = 0.5,
                    const int offset = 0 );
void plot_binomial( const unsigned int numTrials,
                    const double mu,
                    const double sigma );

/** plot multiple (i.e. 2) plots (vertically aligned)
 * @param command_string1 commands for the first plot
 * @param command_string2 commands for the second plot
 * @param gi the plotter to be used, a new plotter is created if (gi == NULL)
 */
void plot_multiplot( const std::string& command_string1, const std::string& command_string2, GnuplotInterface* gi = NULL );

#endif // CROCO__UTILS__GNUPLOT_MACROS
