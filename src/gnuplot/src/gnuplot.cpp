#include "gnuplot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
//
#ifndef ENABLE_OUTPUT_STYLING
  #define ENABLE_OUTPUT_STYLING
#endif
#include "text/colors.h"
#include "text/to_string.h"

using namespace std;


/** Maximal size of a gnuplot command */
#define GP_CMD_SIZE       32768
/** Maximal size of a plot title */
#define GP_TITLE_SIZE   	80
/** Maximal size for an equation */
#define GP_EQ_SIZE      	512
/** Maximal size of a name in the PATH */
#define GP_PATH_MAXNAMESZ       4096

/** Maximal number of simultaneous temporary files */
#define GP_MAX_TMP_FILES    64
/** Maximal size of a temporary file name */
#define GP_TMP_NAME_SIZE    512


/** Define P_tmpdir if not defined (this is normally a POSIX symbol) */
#ifndef P_tmpdir
#define P_tmpdir "."
#endif

/** Uncomment the following definition if all commands sent to
 ** Gnuplot should also be printed on stdout
 **/
// #define GNUPLOT_INTERFACE_PRINT_TRANSCRIPT

/**
  @internal
  @typedef	GnuplotHandle
  @brief	gnuplot session handle (opaque type).

  This structure holds all necessary information to talk to a gnuplot
  session. It is built and returned by gnuplot_init() and later used
  by all functions in this module to communicate with the session, then
  meant to be closed by gnuplot_close().

  This structure is meant to remain opaque, you normally do not need
  to know what is contained in there.
 */
typedef struct GnuplotHandle
{
  /** Pipe to gnuplot process */
  FILE    * gnucmd;

  /** Name of temporary files */
  char      to_delete[ GP_MAX_TMP_FILES ][ GP_TMP_NAME_SIZE ];
  /** Number of temporary files */
  int       ntmp;
};



// --------------------------------------------------------------------------
GnuplotInterface::
    GnuplotInterface( const bool quiet,
                      string fileName,
                      OutputMode mode,
                      Settings * settings )
// --------------------------------------------------------------------------
{
  m_tSettings.quiet = quiet;
  if( settings != NULL )
    settings->quiet = quiet;
  initializeInterface( fileName, mode, settings );
}


// --------------------------------------------------------------------------
GnuplotInterface::
    GnuplotInterface( string fileName,
                      OutputMode mode,
                      Settings * settings )
// --------------------------------------------------------------------------
{
  initializeInterface( fileName, mode, settings );
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    initializeInterface( std::string fileName,
                        OutputMode mode,
                        Settings* settings )
// --------------------------------------------------------------------------
{
  m_sFileName  = fileName;
  m_iPlots     = 0;
  m_sPlotStyle = "points";
  m_tOutputMode = mode;
  if ( settings != NULL )
    m_tSettings = *settings;
  m_sCmdArgs    = "";

  if ( settings != NULL )
  {
    m_sCmdArgs += ( ( settings->raiseWindow ) ? " -raise" : " -noraise" );
    m_sCmdArgs += ( ( settings->persist ) ? " -persist" : "" );

    m_sCmdArgs += " -xrm 'gnuplot*background:"  + getGnuplotColorString( settings->background ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*textColor:"   + getGnuplotColorString( settings->text ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*borderColor:" + getGnuplotColorString( settings->border ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*axisColor:"   + getGnuplotColorString( settings->axis ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line1Color:" + getGnuplotColorString( settings->line1Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line2Color:" + getGnuplotColorString( settings->line2Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line3Color:" + getGnuplotColorString( settings->line3Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line4Color:" + getGnuplotColorString( settings->line4Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line5Color:" + getGnuplotColorString( settings->line5Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line6Color:" + getGnuplotColorString( settings->line6Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line7Color:" + getGnuplotColorString( settings->line7Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*line8Color:" + getGnuplotColorString( settings->line8Color ) + "'";
    m_sCmdArgs += " -xrm 'gnuplot*font: " + settings->font + "'";
    m_sCmdArgs += " -xrm 'gnuplot*title:" + settings->windowTitle + "'";
  }

  if( m_tSettings.quiet )
    m_sCmdArgs += " > /dev/null 2>&1";

  if ( m_sFileName != "" )
  {
    string gpFileName = m_sFileName + ".gnuplot";
    m_pOutputStream = new ofstream( gpFileName.c_str(), ios::out );

    if ( m_pOutputStream == NULL || !m_pOutputStream->is_open() )
    {
      cerr << "Error opening file " << fileName << endl;
      exit(1);
    }

    (*m_pOutputStream) << "#!/usr/bin/gnuplot -persist" << endl;

    switch ( m_tOutputMode )
    {
      case POSTSCRIPT:
      {
        (*m_pOutputStream) << "set terminal postscript eps enhanced color solid\n";
        (*m_pOutputStream) << "set output \"" << fileName << ".ps\"" << endl;
        break;
      }

      case FIG:
      {
        (*m_pOutputStream) << "set terminal fig monochrome small pointsmax 1000\n";
        (*m_pOutputStream) << "set output \"" << fileName << ".fig\"" << endl;
        break;
      }

      case PNG:
      {
        // Transparent background, font size medium (7x13 pixels, bold), crop
        // unused pixels on image border
        (*m_pOutputStream) << "set terminal png transparent medium crop\n";
        (*m_pOutputStream) << "set output \"" << fileName << ".png\"" << endl;
        break;
      }

      default:
        break;
    }

    m_pGnuplotHandle = NULL;
  }
  else // gnuplot_i stuff
  {
    m_pOutputStream = NULL;
    m_pGnuplotHandle = init();
    if ( m_pGnuplotHandle == NULL )
      delete this;
  }
}


// --------------------------------------------------------------------------
GnuplotInterface::
    ~GnuplotInterface()
// --------------------------------------------------------------------------
{
  if ( m_pOutputStream != NULL )
  {
    if ( m_tOutputMode == WINDOW )
    {
      // Wait for carriage return s.t. window is not closed immediately
      (*m_pOutputStream) << "pause -1" << endl;
    }
    m_pOutputStream->flush();
    m_pOutputStream->close();
    delete m_pOutputStream;
  }
  else // gnuplot_i stuff
  {
    close( );
  }
}


// --------------------------------------------------------------------------
char * GnuplotInterface::
    getProgramPath(char * pname)
// --------------------------------------------------------------------------
{
  int         i, j, lg;
  char    *   path;
  static char buf[GP_PATH_MAXNAMESZ];

  /* Trivial case: try in CWD */
  sprintf(buf, "./%s", pname);
  if (access(buf, X_OK)==0) {
      sprintf(buf, ".");
      return buf ;
  }
  /* Try out in all paths given in the PATH variable */
  buf[0] = 0;
  path = getenv("PATH");
  if (path!=NULL) {
      for (i=0; path[i]; ) {
          for (j=i ; (path[j]) && (path[j]!=':'); j++);
          lg = j - i;
          strncpy(buf, path + i, lg);
          if (lg == 0) buf[lg++] = '.';
          buf[lg++] = '/';
          strcpy(buf + lg, pname);
          if (access(buf, X_OK) == 0) {
              /* Found it! */
              break ;
          }
          buf[0] = 0;
          i = j;
          if (path[i] == ':') i++ ;
      }
  } else {
              fprintf(stderr, "PATH variable not set\n");
      }
  /* If the buffer is still empty, the command was not found */
  if (buf[0] == 0) return NULL ;
  /* Otherwise truncate the command name to yield path only */
  lg = strlen(buf) - 1 ;
  while (buf[lg]!='/') {
      buf[lg]=0 ;
      lg -- ;
  }
  buf[lg] = 0;
  return buf ;
}


// --------------------------------------------------------------------------
GnuplotHandle * GnuplotInterface::
    init()
// --------------------------------------------------------------------------
{
  GnuplotHandle * handle;

  if ( getenv("DISPLAY") == NULL )
  {
    cerr << "cannot find DISPLAY variable: is it set?" << endl;
  }

  if ( getProgramPath( "gnuplot" )==NULL )
  {
    cerr << "Cannot find gnuplot in your PATH" << endl;
    return NULL ;
  }

  /*
    * Structure initialization:
    */
  handle = (GnuplotHandle*) malloc(sizeof(GnuplotHandle));
  setStyle( "points" );
  handle->ntmp = 0 ;

  string open_cmd = "gnuplot" + m_sCmdArgs;
  handle->gnucmd = popen( open_cmd.c_str(), "w" );
  if ( handle->gnucmd == NULL )
  {
    fprintf(stderr, "error starting gnuplot\n");
    free(handle);
    return NULL ;
  }

  return handle;
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    close()
// --------------------------------------------------------------------------
{
  if ( pclose(m_pGnuplotHandle->gnucmd) == -1 )
  {
    fprintf( stderr, "problem closing communication to gnuplot\n" );
    return;
  }
  if ( m_pGnuplotHandle->ntmp )
  {
    for ( int i = 0; i < m_pGnuplotHandle->ntmp; ++i )
    {
      remove( m_pGnuplotHandle->to_delete[i]);
    }
  }
  free(m_pGnuplotHandle);
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    commandStr( const string & cmd )
// --------------------------------------------------------------------------
{
  string::size_type first = 0;
  while ( first < cmd.length() )
  {
    string::size_type i = cmd.find_last_of( "\n", first + 500 );
    if ( i == string::npos )
      i = cmd.find_last_of( "\\", first + 500 );

    if ( i != string::npos && i > first )
    {
      if ( i-first+1 >= GP_CMD_SIZE )
      {
        cerr << "WARNING: Too long Gnuplot command" << endl;
        return;
      }

      string sub( cmd.substr( first, i-first+1 ) );
      command( sub.c_str() );
      first = i+1;
    }
    else
    {
      if ( cmd.length()-first+1 >= GP_CMD_SIZE )
      {
        cerr << "WARNING: Too long Gnuplot command" << endl;
        return;
      }

      string sub( cmd.substr( first ) );
      command( sub.c_str() );
      first = cmd.length();
      break;
    }
  }
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    command( const char * cmd, ... )
// --------------------------------------------------------------------------
{
#ifdef GNUPLOT_INTERFACE_PRINT_TRANSCRIPT
  cout << color::blue << cmd << color::normal << endl;
#endif

  va_list ap;
  static char local_cmd[ GP_CMD_SIZE ];

  va_start( ap, cmd );
  int length = vsprintf( local_cmd, cmd, ap );
  va_end( ap );

  if( length == 0 || local_cmd[length-1] != '\n' )
    strcat( local_cmd, "\n" );

  if ( m_pOutputStream != NULL )
  {
    (*m_pOutputStream) << local_cmd;
  }
  else // gnuplot_i stuff
  {
    fputs( local_cmd, m_pGnuplotHandle->gnucmd );
    fflush( m_pGnuplotHandle->gnucmd );
  }
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    setStyle( char * plot_style )
// --------------------------------------------------------------------------
{
  if (strcmp(plot_style, "lines") &&
      strcmp(plot_style, "points") &&
      strcmp(plot_style, "linespoints") &&
      strcmp(plot_style, "impulses") &&
      strcmp(plot_style, "dots") &&
      strcmp(plot_style, "steps") &&
      strcmp(plot_style, "errorbars") &&
      strcmp(plot_style, "boxes") &&
      strcmp(plot_style, "boxerrorbars"))
  {
    cerr << "warning: unknown requested style: using points" << endl;
    m_sPlotStyle = "points";
  }
  else
  {
    m_sPlotStyle = plot_style;
  }
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    plotEquation( char * equation, char * title )
// --------------------------------------------------------------------------
{
    char    cmd[GP_CMD_SIZE];
    char    plot_str[GP_EQ_SIZE] ;
    char    title_str[GP_TITLE_SIZE] ;

    if (title == NULL) {
        strcpy(title_str, "no title");
    } else {
        strcpy(title_str, title);
    }
    if (m_iPlots > 0) {
        strcpy(plot_str, "replot");
    } else {
        strcpy(plot_str, "plot");
    }

    sprintf(cmd, "%s %s title \"%s\" with %s",
                  plot_str, equation, title_str, m_sPlotStyle.c_str() );
    command( cmd );
    m_iPlots++ ;
}


// --------------------------------------------------------------------------
string GnuplotInterface::
    getGnuplotColorString( Color color )
// --------------------------------------------------------------------------
{
  switch ( color )
  {
    case black:      return "black";
    case white:      return "white";
    case gray:       return "gray";
    case lightgray:  return "white,0.75";
    case red:        return "red";
    case green:      return "green";
    case blue:       return "blue";
    case cyan:       return "cyan";
    case sienna:     return "sienna";
    case magenta:    return "magenta";
    case orange:     return "orange";
    case coral:      return "coral";

    default:
      break;
  }
  return "";
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    setGrid( bool enable )
// --------------------------------------------------------------------------
{
  if ( enable )
    command( "set grid" );
  else
    command( "unset grid" );
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    setDataGrid3D( bool enable )
// --------------------------------------------------------------------------
{
  if ( enable )
    command( "set dgrid3d" );
  else
    command( "unset dgrid3d" );
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    setDataGrid3DDimensions( int x, int y, float weight )
// --------------------------------------------------------------------------
{
  commandStr( "set dgrid3d " + toString(x) + ","
                          + toString(y) + "," + toString(weight) );
}


// --------------------------------------------------------------------------
void GnuplotInterface::
    setView( float rotX, float rotY )
// ------------------------------------------------------------------------
{
  commandStr( "set view " + toString(rotX) + "," + toString(rotY) );
}


// ------------------------------------------------------------------------
void GnuplotInterface::
    setSize( float width, float height )
// ------------------------------------------------------------------------
{
  commandStr( "set size " + toString(width) + "," + toString(height) );
}


// ------------------------------------------------------------------------
void GnuplotInterface::
    setXLabel( const std::string & label )
// ------------------------------------------------------------------------
{
  commandStr( "set xlabel \"" + label + "\"" );
}


// ------------------------------------------------------------------------
void GnuplotInterface::
    setYLabel( const std::string & label )
// ------------------------------------------------------------------------
{
  commandStr( "set ylabel \"" + label + "\"" );
}


// ------------------------------------------------------------------------
void GnuplotInterface::
    setZLabel( const std::string & label )
// ------------------------------------------------------------------------
{
  commandStr( "set zlabel \"" + label + "\"" );
}

