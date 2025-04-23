#include <algorithm>
#include <sstream>
#include <math.h>
//
#include <utils/gnuplot/gnuplot.h>
#include <utils/text/to_string.h>
#include <carmenx/utils/geometry/vector2d.h>
#include <carmenx/utils/geometry/vector3d.h>
#include <carmenx/ambisense_slam/ambisense_slam_config.h>
#include <carmenx/ambisense_slam/filters/pf2_6d.h>
//
#include <boost/math/distributions/binomial.hpp>
//
#include "plot_macros.h"

using namespace std;

// --------------------------------------------------------------------------
const GnuPlotCommandTriplet GnuPlotCommandTriplet::
operator+( const GnuPlotCommandTriplet& other ) const
// --------------------------------------------------------------------------
{
  std::string plotCommandConcatenated;
  // does this command-string already contain a "plot "-command? (or splot)
  if( this->plotCommand.size() > 0 && this->plotCommand.find( "plot " ) != std::string::npos ) {
    // remove "plot " (or splot) inside the other plot command
    unsigned plot_pos = other.plotCommand.find( "plot " );
    if( plot_pos != std::string::npos ) {
      plotCommandConcatenated = this->plotCommand + ", " + other.plotCommand.substr( plot_pos + 5 );
    } else {
      plotCommandConcatenated = this->plotCommand + ", " + other.plotCommand;
    }
  } else {
    if( this->plotCommand.empty() )
      plotCommandConcatenated = other.plotCommand;
    else {
      // something wrong here, this plot command does not have a plot-command, we assume its invalid and overwrite it with the other
      return GnuPlotCommandTriplet( other );
    }
  }
  // concatenate
  return GnuPlotCommandTriplet( this->preAmble + other.preAmble, plotCommandConcatenated, this->data + other.data );
}


// --------------------------------------------------------------------------
std::string GnuPlotCommandTriplet::
getCommandString( const std::string& header /* = std::string() */ ) const
// --------------------------------------------------------------------------
{
  std::string plotCmd;
  if( ! header.empty() ) {
    plotCmd += header;
  } else {
    plotCmd += this->preAmble;
  }
  if( this->plotCommand[ plotCommand.size() -1 ] != '\n' )
    plotCmd += this->plotCommand + '\n';
  else
    plotCmd += this->plotCommand;
  plotCmd += this->data;
  return plotCmd;
}


// --------------------------------------------------------------------------
void GnuPlotCommandTriplet::plot( const std::string& header /* = std::string() */,
                                  GnuplotInterface* plotter /* = NULL*/ ) const
// --------------------------------------------------------------------------
{
  bool bFree = false;
  if( plotter == NULL ) {
    plotter = new GnuplotInterface();
    bFree = true;
  }

  plotter->commandStr( getCommandString( header ) );

  if( bFree ) {
    delete plotter;
    plotter = NULL;
  }
}


// --------------------------------------------------------------------------
void plot_doubles( const std::vector< double >& values,
                   const std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  if( !title.empty() )
    gi.commandStr( std::string("set title '" + title + "'\n")  );
  gi.commandStr( "set grid\nset mouse\nplot '-' w p\n" );

  string gnuplot_command;

  // Raw samples
  for ( uint u = 0; u < values.size(); ++u )
  {
    stringstream s;
    s << u << ' ' << values[u] << '\n';
    gnuplot_command += s.str();
  }
  gnuplot_command += "e\n";

  gi.commandStr( gnuplot_command );
}


// --------------------------------------------------------------------------
GnuPlotCommandTriplet get_doubles_plot_triplet( const std::vector<double>& values,
                                                const std::string title )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet ret;

  // preamble
  ret.preAmble = std::string( "set size square\nset grid\nset mouse\nset title '" + title + "'\n" );
  // plot command
  ret.plotCommand = "plot '-' w p";
  // sample data
  for ( uint u = 0; u < values.size(); ++u ) {
    stringstream s;
    s << u << ' ' << values[u] << '\n';
    ret.data += s.str();
  }
  ret.data += "e\n";

  return ret;
}


// --------------------------------------------------------------------------
void plot_pairs( const std::vector< std::pair< double, double > >& values,
                 const std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  if( !title.empty() )
    gi.commandStr( std::string("set title '" + title + "'\n")  );
  gi.commandStr( "set size square\nset grid\nset mouse\nplot '-' w p\n" );

  string gnuplot_command;

  // Raw samples
  for ( uint u = 0; u < values.size(); ++u )
  {
    stringstream s;
    s << values[u].first << ' ' << values[u].second << '\n';
    gnuplot_command += s.str();
  }
  gnuplot_command += "e\n";

  gi.commandStr( gnuplot_command );
}


// --------------------------------------------------------------------------
void plot_histogram( const std::vector< double >& values,
                     unsigned int numBins /* = 100 */ )
// --------------------------------------------------------------------------
{
  if( values.size() <= 1 )
    return;
  GnuplotInterface * gi = new GnuplotInterface();
  std::string gnuplot_command( "set grid\nset mouse\nset title 'Histogram'\n");
  // first analyze data
  double min = values[0];
  double max = values[0];
  for(size_t i=1; i<values.size(); ++i) {
    min = std::min( min, values[i] );
    max = std::max( max, values[i] );
  }
  const double bin_width = (max - min) / (double)numBins;
  gnuplot_command += "rint(x)=(x-int(x)>0.9999)?int(x)+1:int(x)\n";
  gnuplot_command += "binwidth=" + toString( bin_width ) + '\n';
  gnuplot_command += "set boxwidth binwidth\nset style fill solid 0.5\n";
  gnuplot_command += "bin(x,width)=width*rint(x/width)+binwidth/2.0\n";
  gnuplot_command += "plot '-' using (bin($1,binwidth)):(1.0) smooth freq with boxes lc rgb\"green\" noti\n";
  for( size_t i=0; i<values.size(); ++i ) {
    stringstream s;
    s << values[i] << '\n';
    gnuplot_command += s.str();
  }
  gnuplot_command += "e\n";

  gi->commandStr( gnuplot_command );
}


// --------------------------------------------------------------------------
void plot_cdf( const std::vector< double >& values,
               unsigned int numBins /* = 100 */ )
// --------------------------------------------------------------------------
{
  if( values.size() <= 1 )
    return;
  GnuplotInterface * gi = new GnuplotInterface();
  std::string gnuplot_command( "set grid\nset mouse\nset title 'CDF'\n");
  // first analyze data
  double min = values[0];
  double max = values[0];
  for(size_t i=1; i<values.size(); ++i) {
    min = std::min( min, values[i] );
    max = std::max( max, values[i] );
  }
  const double bin_width = (max - min) / (double)numBins;
  std::vector< size_t > bins( numBins, 0 );
  for( size_t i=0; i<values.size(); ++i ) {
  	bins[ (values[i] - min) / bin_width ]++;
  }
  for( size_t i=1; i<numBins; ++i ) {
  	bins[i] += bins[i-1];
  }
  gnuplot_command += "plot '-' with lines lc rgb\"green\" noti\n";
  stringstream s;
  s << min << ' ' << 0 << '\n';
  for( size_t i=0; i<numBins; ++i ) {
    s << min + (i+1) * bin_width << ' ' << (double)bins[i] / (double)values.size() << '\n';
  }
  gnuplot_command += s.str();
  gnuplot_command += "e\n";
  gi->commandStr( gnuplot_command );
}



// --------------------------------------------------------------------------
template<class Vec2Type>
std::string get_vec2_data_string( const std::vector< Vec2Type >& values )
// --------------------------------------------------------------------------
{
  std::string plot_data;
  // Raw samples
  for ( uint u = 0; u < values.size(); ++u )
  {
    stringstream s;
    s << values[u].x << ' ' << values[u].y << '\n';
    plot_data += s.str();
  }
  plot_data += "e\n";
  return plot_data;
}


// --------------------------------------------------------------------------
template< class Vec2Type >
GnuPlotCommandTriplet get_vec2_plot_triplet( const std::vector< Vec2Type >& values,
                                             std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet triplet;
  triplet.preAmble = std::string( "set size square\nset grid\nset mouse\nset title '" + title + "'\n" );
  triplet.plotCommand = "plot '-' w p";
  triplet.data = get_vec2_data_string( values );
  return triplet;
}

// --------------------------------------------------------------------------
template<class Vec2Type>
std::string get_vec2_plot_string( const std::vector< Vec2Type >& values,
                                  std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  return get_vec2_plot_triplet( values, title ).getCommandString();
}

// --------------------------------------------------------------------------
template<class Vec2Type>
void plot_vec2( const std::vector< Vec2Type >& values,
                std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_vec2_plot_string( values, title ) );
}


// --------------------------------------------------------------------------
template< class Vec3Type >
std::string get_vec3_data_string( const std::vector< Vec3Type >& values )
// --------------------------------------------------------------------------
{
  std::string plot_data;
  // Raw samples
  for ( uint u = 0; u < values.size(); ++u )
  {
    stringstream s;
    s << values[u].x << ' ' << values[u].y << ' ' << values[u].z << '\n';
    plot_data += s.str();
  }
  plot_data += "e\n";
  return plot_data;
}


// --------------------------------------------------------------------------
template< class Vec3Type >
GnuPlotCommandTriplet get_vec3_plot_triplet( const std::vector< Vec3Type >& values,
                                             std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet triplet;
  triplet.preAmble = std::string( "set size square\nset grid\nset mouse\nset title '" + title + "'\n" );
  triplet.plotCommand = "splot '-' w impulses lt 0, '-' w p";
  triplet.data = get_vec3_data_string( values ) + get_vec3_data_string( values );
  return triplet;
}


// --------------------------------------------------------------------------
template< class Vec3Type >
GnuPlotCommandTriplet get_vec3_contour_plot_triplet( const std::vector< Vec3Type >& values, std::string title = std::string("") )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet triplet;
  std::string preamble = "set terminal wxt enhanced\n";
  preamble += "set title '" + title + "'\n";
  preamble += "set pm3d map\n";
  preamble += "set size ratio -1\nset border -1\nset mouse\nset grid front\nshow grid\nset dgrid3d 100,100,2\n";
  preamble += "set xtics 10\n";
  preamble += "set ytics 10\n";
  preamble += "set palette rgbformulae 33,13,10\n";
  triplet.preAmble = preamble;
  triplet.plotCommand = "splot '-' u 1:2:3 w pm3d at b";
  triplet.data = get_vec3_data_string( values );
  return triplet;
}


// --------------------------------------------------------------------------
template< class Vec3Type >
std::string get_vec3_plot_string( const std::vector< Vec3Type >& values,
                                  std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  return get_vec3_plot_triplet( values, title ).getCommandString();
}

// --------------------------------------------------------------------------
template< class Vec3Type >
void plot_vec3( const std::vector< Vec3Type >& values,
                std::string title /* = std::string("") */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_vec3_plot_string( values, title ) );
}


// --------------------------------------------------------------------------
template< class Pose3Type >
std::string get_vectorfield_data_string( const std::vector< Pose3Type >& values,
                                         const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  std::string plot_data;
  // Raw samples
  for ( uint u = 0; u < values.size(); ++u )
  {
    stringstream s;
    s << values[u].x << ' ' << values[u].y << ' ' << arrowSize * cos( values[u].theta_z ) << ' ' << arrowSize * sin( values[u].theta_z ) << '\n';
    plot_data += s.str();
  }
  plot_data += "e\n";
  return plot_data;
}


// --------------------------------------------------------------------------
template< class Pose3Type >
GnuPlotCommandTriplet get_vectorfield_plot_triplet( const std::vector< Pose3Type >& values,
                                                    std::string title /* = std::string("") */,
                                                    const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet triplet;
  triplet.preAmble = std::string( "set size square\nset grid\nset mouse\nset title '" + title + "'\n" );
  triplet.plotCommand = "plot '-' using 1:2:3:4 w vectors lt 2";
  triplet.data = get_vectorfield_data_string( values, arrowSize );
  return triplet;
}

// --------------------------------------------------------------------------
template< class Pose3Type >
std::string get_vectorfield_plot_string( const std::vector< Pose3Type >& values,
                                         std::string title /* = std::string("") */,
                                         const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  return get_vectorfield_plot_triplet( values, title, arrowSize ).getCommandString();
}

// --------------------------------------------------------------------------
template< class Pose3Type >
void plot_vectorfield( const std::vector< Pose3Type >& values,
                       std::string title /* = std::string("") */,
                       const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_vectorfield_plot_string( values, title, arrowSize ) );
}


// --------------------------------------------------------------------------
template< class Pose3Type >
std::string get_trajectory_data_string( const std::vector< Pose3Type >& values,
                                        const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  return get_vectorfield_data_string( values, arrowSize );
}


// --------------------------------------------------------------------------
template< class Pose3Type >
GnuPlotCommandTriplet get_trajectory_plot_triplet( const std::vector< Pose3Type >& values,
                                                   std::string title /* = std::string("") */,
                                                   const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  GnuPlotCommandTriplet triplet;
  triplet.preAmble = std::string( "set size square\nset grid\nset mouse\nset title '" + title + "'\n" );
  triplet.plotCommand = "plot '-' using 1:2:3:4 w vectors lt 2 noti, '-' using 1:2 w lines lt 2";
  const std::string data = get_trajectory_data_string( values, arrowSize );
  triplet.data = data + data;
  return triplet;
}


// --------------------------------------------------------------------------
template< class Pose3Type >
std::string get_trajectory_plot_string( const std::vector< Pose3Type >& values,
                                        std::string title /* = std::string("") */,
                                        const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  return get_trajectory_plot_triplet( values, title, arrowSize ).getCommandString();
}

// --------------------------------------------------------------------------
template< class Pose3Type >
void plot_trajectory( const std::vector< Pose3Type >& values,
                      std::string title /* = std::string("") */,
                      const double arrowSize /* = 0.1 */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_trajectory_plot_string( values, title, arrowSize ) );
}



typedef std::map< std::string, std::vector< std::pair< double, double > > > PlotSeriesMap;
// --------------------------------------------------------------------------
void plot_several_vectors_of_pairs( PlotSeriesMap & values, GnuplotInterface * gi )
// --------------------------------------------------------------------------
{
  bool local_gi = false;
  if ( ! gi )
  {
    gi = new GnuplotInterface();
    local_gi = true;
  }

  gi->commandStr( "set grid\nset mouse\n" );
  string gnuplot_command = "plot " ;
  for ( PlotSeriesMap::iterator it = values.begin(); it != values.end(); ++it )
  {
    if ( it != values.begin() )
      gnuplot_command += ", ";
    gnuplot_command += "'-' w l ti '" + it->first + "'";
  }
  gnuplot_command += "\n";

  // Raw samples
  for ( PlotSeriesMap::iterator it = values.begin(); it != values.end(); ++it )
  {
    for ( uint u = 0; u < it->second.size(); ++u )
    {
      stringstream s;
      s << it->second[u].first << ' ' << it->second[u].second << '\n';
      gnuplot_command += s.str();
    }
    gnuplot_command += "e\n";
  }

  gi->commandStr( gnuplot_command );

  if ( local_gi )
  {
    string s;
    cin >> s;
    delete gi;
  }
}


// --------------------------------------------------------------------------
void plot_several_vectors_of_pairs( std::vector< std::vector< std::pair< double, double > > > values )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( "set grid\nset mouse\n" );
  string gnuplot_command = "plot '-' w p" ;
  for ( uint u = 1; u < values.size(); ++u )
    gnuplot_command += ", '-' w p";
  gnuplot_command += "\n";

  // Raw samples
  for ( uint v = 0; v < values.size(); ++v )
  {
    for ( uint u = 0; u < values[v].size(); ++u )
    {
      stringstream s;
      s << values[v][u].first << ' ' << values[v][u].second << '\n';
      gnuplot_command += s.str();
    }
    gnuplot_command += "e\n";

    gi.commandStr( gnuplot_command );
  }
}


// --------------------------------------------------------------------------
double
get_gaussian( double x, double mu, double sigma )
// --------------------------------------------------------------------------
{
  if (sigma < 1e-9 || isnan(sigma)) {
    if (fabs(x-mu) < 1e-9)
      return 1.0;
    else
      return 0.0;
  }
  return( (1./sqrt(2.0*M_PI*sigma*sigma)) * exp(-(((x-mu)*(x-mu))/(2.*sigma*sigma))) );
}


// --------------------------------------------------------------------------
std::string get_guassian_data_string( const double mu /* = 0. */,
                                      const double sigma /* = 1. */,
                                      double xmin /* = -1 */,
                                      double xmax /* = -1 */,
                                      const size_t num_steps /* = 100*/ )
// --------------------------------------------------------------------------
{
  if( xmin == xmax ) {
    xmin = mu - 3 * sigma;
    xmax = mu + 3 * sigma;
  }
  const double step_size = ( xmax - xmin ) / (double)num_steps;

  std::string plot_data;
  for ( double x = xmin; x <= xmax; x += step_size ) {
    stringstream s;
    double d = get_gaussian( x, mu, sigma );
    s << x << ' ' << d << '\n';
    plot_data += s.str();
  }
  plot_data += "e\n";
  return plot_data;
}


// --------------------------------------------------------------------------
std::string get_gaussian_plot_string( const double mu /* = 0. */,
                                      const double sigma /* = 1. */ )
// --------------------------------------------------------------------------
{
  return std::string( "plot '-' w l ti 'Gaussian'\n" + get_guassian_data_string( mu, sigma ) );
}


// --------------------------------------------------------------------------
void plot_gaussian( const double mu /* = 0. */,
                    const double sigma /* = 1. */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_gaussian_plot_string( mu, sigma ) );
}



// --------------------------------------------------------------------------
std::string get_binomial_data_string( const unsigned int numTrials,
                                      const double probability /* = 0.5 */,
                                      const int offset /* = 0 */ )
// --------------------------------------------------------------------------
{
  boost::math::binomial bin( numTrials, probability );

  std::string plot_data;
  for ( size_t x = 0; x <= numTrials; ++x ) {
    stringstream s;
    double d = boost::math::pdf( bin, x );
    s << (x + offset) << ' ' << d << '\n';
    plot_data += s.str();
  }
  plot_data += "e\n";
  return plot_data;
}


// --------------------------------------------------------------------------
std::string get_binomial_data_string( const unsigned int numTrials,
                                      const double mu,
                                      const double sigma )
// --------------------------------------------------------------------------
{
  return get_binomial_data_string( numTrials, (1.-sigma), (int)rint(mu) );
}


// --------------------------------------------------------------------------
std::string get_binomial_plot_string( const unsigned int numTrials,
                                      const double probability /* = 0.5 */,
                                      const int offset /* = 0 */ )
// --------------------------------------------------------------------------
{
  return std::string( "plot '-' w l ti 'Binomial(n=" + toString(numTrials) + ",p=" + toString(probability) + ")'\n" + get_binomial_data_string( numTrials, probability, offset ) );
}


// --------------------------------------------------------------------------
std::string get_binomial_plot_string( const unsigned int numTrials,
                                      const double mu,
                                      const double sigma )
// --------------------------------------------------------------------------
{
  return get_binomial_plot_string( numTrials, (1.-sigma), (int)rint(mu) );
}


// --------------------------------------------------------------------------
void plot_binomial( const unsigned int numTrials,
                    const double probability /* = 0.5 */,
                    const int offset /* = 0 */ )
// --------------------------------------------------------------------------
{
  GnuplotInterface gi;
  gi.commandStr( get_binomial_plot_string( numTrials, probability, offset ) );
}
// --------------------------------------------------------------------------
void plot_binomial( const unsigned int numTrials,
                    const double mu,
                    const double sigma )
// --------------------------------------------------------------------------
{
  plot_binomial( numTrials, (1.-sigma), (int)rint(mu) );
}



// --------------------------------------------------------------------------
void plot_multiplot( const std::string& command_string1,
                     const std::string& command_string2,
                     GnuplotInterface* gi /* = NULL */ )
// --------------------------------------------------------------------------
{
  if(gi == NULL) {
    gi = new GnuplotInterface;
  }

  std::string multiplot_header("set multiplot layout 2, 1\n");
  gi->commandStr( multiplot_header + command_string1 + command_string2 );
  /** removed "unset multiplot"-command, since this makes the interactive autoscale focus on the first
   *  plot only and removes the second plot, because gnuplot is not in multiplot mode any more
   *  undesired drawback: the plot seems to be replotted again on exit ...
   */
  // gi->commandStr( multiplot_header + command_string1 + command_string2 + "unset multiplot\n" );
}


// --------------------------------------------------------------------------
// explicit instantiation
// --------------------------------------------------------------------------

#define EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( VECTOR2_TYPE ) \
    template void plot_vec2<VECTOR2_TYPE>( const std::vector< VECTOR2_TYPE >&, std::string ); \
    template GnuPlotCommandTriplet get_vec2_plot_triplet<VECTOR2_TYPE>( const std::vector< VECTOR2_TYPE >&, std::string ); \
    template std::string get_vec2_plot_string<VECTOR2_TYPE>( const std::vector< VECTOR2_TYPE >&, std::string ); \
    template std::string get_vec2_data_string<VECTOR2_TYPE>( const std::vector< VECTOR2_TYPE >& );

EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector2d )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector2f )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector2i )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector2u )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector3d )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector3f )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector3i )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Vector3u )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( carmen_point_t )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( carmen_6d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( carmen_3d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Pose )
EXPLICIT_INSTANTIATE_PLOT_VEC2_FOR( Particle )


#define EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( VECTOR3_TYPE ) \
    template void plot_vec3<VECTOR3_TYPE>( const std::vector< VECTOR3_TYPE >&, std::string ); \
    template GnuPlotCommandTriplet get_vec3_plot_triplet<VECTOR3_TYPE>( const std::vector< VECTOR3_TYPE >&, std::string ); \
    template std::string get_vec3_plot_string<VECTOR3_TYPE>( const std::vector< VECTOR3_TYPE >&, std::string ); \
    template std::string get_vec3_data_string<VECTOR3_TYPE>( const std::vector< VECTOR3_TYPE >& ); \
    template GnuPlotCommandTriplet get_vec3_contour_plot_triplet<VECTOR3_TYPE>( const std::vector< VECTOR3_TYPE >&, std::string );

EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Vector3d )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Vector3f )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Vector3i )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Vector3u )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( carmen_6d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( carmen_3d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Pose )
EXPLICIT_INSTANTIATE_PLOT_VEC3_FOR( Particle )


#define EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR( POSE3_TYPE ) \
    template void plot_vectorfield<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template GnuPlotCommandTriplet get_vectorfield_plot_triplet<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template std::string get_vectorfield_plot_string<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template std::string get_vectorfield_data_string<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, const double ); \
    template void plot_trajectory<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template GnuPlotCommandTriplet get_trajectory_plot_triplet<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template std::string get_trajectory_plot_string<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, std::string, const double ); \
    template std::string get_trajectory_data_string<POSE3_TYPE>( const std::vector< POSE3_TYPE >&, const double );

EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR( carmen_point_t )
EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR( carmen_6d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR( carmen_3d_point_t )
EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR( Pose )
EXPLICIT_INSTANTIATE_PLOT_VECTORFIELD_FOR(Particle)

