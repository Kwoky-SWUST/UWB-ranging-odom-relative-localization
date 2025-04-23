#ifndef CROCO__UTILS__GNUPLOT_H
#define CROCO__UTILS__GNUPLOT_H

// Forward declarations
class GnuplotHandle;

#include <string>
#include <iostream>
#include <fstream>

/** C++ interface to Gnuplot.
  * Output is either directly displayed in a window or
  * written to a file.
  *
  * Based on the "gnuplot_i" interface by N. Devillard
  * (http://ndevilla.free.fr/gnuplot/)
  * See file "LICENSE" in this directory for license details.
  *
  * Example usage:
  *
  * @code
  *   GnuplotInterface * gi = new GnuplotInterface( false );
  *   gi->command( "plot sin(x)" );
  *   sleep( 10 );
  *   delete gi;
  * @endcode
  *
  *
  * Another example:
  *
  * Somewhere in your constructor:
  * @code
  *   // Open the interface such that outputs are displayed in a window:
  *   GnuplotInterface * m_pGnuplotInterface = new GnuplotInterface( "", GnuplotInterface::WINDOW );
  * @endcode
  *
  *
  *
  * In your main loop, which successively writes data e.g. to a file 'my_experiment.dat':
  * (The file consists of lines with white-space separated columns. Let us assume that in
  *  the first column you place the current time, then X, Y, and Z coordinates. For instance,
  *  my_experiment.dat may look like this:
  * @code
  *   # This is a comment: Columns 1=time, 2=X, 3=Y, 4=Z
  *   0 0 0 0
  *   1 0 1 0.5
  *   2 0 4 1.0
  *   3 0 9 1.5
  *   4 0 16 2.0
  *   # and so on....
  * @endcode
  * Then the following code displays the current data as a 3D plot:
  * @code
  *   // Drawing
  *   string cmd = "splot 'my_experiment.dat' using ($1):($3):($4) title 'Funny data' with vectors";
  *   m_pGnuplotInterface->command( (char*) cmd.c_str() );
  *   //
  *   m_pGnuplotInterface->setView( 72.0, 0.0 );
  * @endcode
  *
  * This interface does NOT exhaustively provide methods for all Gnuplot features. If you
  * want to see what Gnuplot offers, please start 'gnuplot' in your console and enter
  * 'help'. Gnuplot provides an extensive textual help function with topic-based search.
  *
  * @author Philipp Vorst (DA)
  */
// --------------------------------------------------------------------------
class GnuplotInterface
// --------------------------------------------------------------------------
{
  public:
    /** Different output types. WINDOW means live window output, the other
     *  formats require streaming the data to a file. */
    enum OutputMode { WINDOW, FIG, PNG, POSTSCRIPT };

    enum Color { white, black, gray, red, green, blue, cyan, sienna, magenta, orange, coral, lightgray };

    class Settings
    {
      public:
        Settings() :  background(white), text(black), border(black), axis(black),
                      line1Color(red), line2Color(green), line3Color(blue), line4Color(magenta),
                      line5Color(cyan), line6Color(sienna), line7Color(orange), line8Color(coral),
                      raiseWindow(true), persist(false), windowTitle("Gnuplot"), font(""), quiet(1) {}

        Color background;
        Color text;
        Color border;
        Color axis;
        Color line1Color;
        Color line2Color;
        Color line3Color;
        Color line4Color;
        Color line5Color;
        Color line6Color;
        Color line7Color;
        Color line8Color;
        bool  raiseWindow;
        bool  persist;
        bool  quiet;
        std::string windowTitle;
        std::string font;
    };

  public:
    /** Create a Gnuplot interface. If fileName has non-zero length, the
      * file with the specified name will be opened for output rather than
      * opening a Gnuplot window.
      *   @param postscript If true, the Gnuplot commands will automatically
      *                     realize the desired plots in a document of the
      *                     given type
      *   @param fileName   The name of the file to which all commands are
      *                     written if you do not want to use Gnuplot now. In
      *                     case of postscript conversion, this should be the
      *                     name of the postscript file.
      *  @see OutputMode
      */
    GnuplotInterface( std::string      fileName,
                      OutputMode       mode = WINDOW,
                      Settings       * colors = NULL
                    );

    /** See standard constructor + quiet flag
     * @param quiet pipes all output to /dev/null
     */
    GnuplotInterface( const bool       quiet = true,
                      std::string      fileName = "",
                      OutputMode       mode = WINDOW,
                      Settings       * colors = NULL
                    );

    /** Standard destructor */
    virtual ~GnuplotInterface();

    /** basic initialization, called from constructors ...
     */
    void initializeInterface( std::string      fileName,
                             OutputMode       mode,
                             Settings       * colors );


    /**
      @brief	Send a command to an active gnuplot session.
      @param	cmd    Command to send, same as a printf statement.

      This sends a string to an active gnuplot session, to be executed.
      There is strictly no way to know if the command has been
      successfully executed or not.
      The command syntax is the same as printf.

      Examples:

      @code
        command( "plot %d*x", 23.0);
        command( "plot %g * cos(%g * x)", 32.0, -3.0);
      @endcode

      Since the communication to the gnuplot process is run through
      a standard Unix pipe, it is only unidirectional. This means that
      it is not possible for this interface to query an error status
      back from gnuplot.

      */
    void command( const char * cmd, ... );

    /** Send a command to an active gnuplot session */
    void commandStr( const std::string & cmd );


    /**
     *  @brief	Change the plotting style of a gnuplot session.
     *  @param	plot_style Plotting-style to use (character string)
     *
     *  The provided plotting style is a character string. It must be one of
     *  the following:
     *
     *  - lines
     *  - points
     *  - linespoints
     *  - impulses
     *  - dots
     *  - steps
     *  - errorbars
     *  - boxes
     *  - boxeserrorbars
     *
     */
    void setStyle( char * plot_style );

    /**
      @brief	Plot a curve of given equation y=f(x).
      @param	equation	Equation to plot.
      @param	title		Title of the plot.

      Plots out a curve of given equation. The general form of the
      equation is y=f(x), you only provide the f(x) side of the equation.

      Example:

      @code
            gnuplot_ctrl    *h ;
            char            eq[80] ;

            h = gnuplot_init() ;
            strcpy(eq, "sin(x) * cos(2*x)") ;
            gnuplot_plot_equation(h, eq, "sine wave", normal) ;
            gnuplot_close(h) ;
      @endcode
      */
    void plotEquation( char * equation, char * title );


    // ------------------------------------------------------------------------
    // ----  C O M M A N D   S H O R T C U T S  -------------------------------
    // ------------------------------------------------------------------------

    /** Activate/ deactivate pm3d style. This style is very nice if one wants
      * to 'splot' 3D data. */
    void setPM3D( bool enable ) { if (enable) command( "set pm3d" );
                                  else command("unset pm3d");
                                }

    /** Activate/ deactivate the normal 2D grid. */
    void setGrid( bool enable );

    /** Activate/ deactivate the 3D data grid. When enabled, 3D data read from
      * a file are always treated as a scattered data set. */
    void setDataGrid3D( bool enable );

    /** Set the grid cell numbers and cel weighting for 3D grids. Automatically
      * enables the 3D data grid. */
    void setDataGrid3DDimensions( int x, int y, float weight );

    /** Use/ do not use colored output */
    void setColor( bool color ) { if (color) command("set palette color");
                                  else command("set palette gray");
                                }

    /** Set the X axis label */
    void setXLabel( const std::string & label );

    /** Set the Y axis label */
    void setYLabel( const std::string & label );

    /** Set the Z axis label */
    void setZLabel( const std::string & label );

    /** Set the view rotations in X and Y direction */
    void setView( float rotX, float rotY );

    /** Set the size of the output. For mode PNG, this is the number of pixels
     *  of the output image */
    void setSize( float width, float height );

    /** Return the settings with which this interface was created */
    const Settings & getSettings() const { return m_tSettings; }


  protected:
    /**
      @brief	Find out where a command lives in your PATH.
      @param	pname Name of the program to look for.
      @return	pointer to statically allocated character string.

      This is the C equivalent to the 'which' command in Unix. It parses
      out your PATH environment variable to find out where a command
      lives. The returned character string is statically allocated within
      this function, i.e. there is no need to free it. Beware that the
      contents of this string will change from one call to the next,
      though (as all static variables in a function).

      The input character string must be the name of a command without
      prefixing path of any kind, i.e. only the command name. The returned
      string is the path in which a command matching the same name was
      found.

      Examples (assuming there is a prog named 'hello' in the cwd):

      @verbatim
        getProgramPath("hello")   returns "."
        getProgramPath("ls")      returns "/bin"
        getProgramPath("csh")     returns "/usr/bin"
        getProgramPath("/bin/ls") returns NULL
      @endverbatim

      */
    char * getProgramPath( char * pname );

    /** Turn the given color to a color string */
    std::string getGnuplotColorString( Color color );

    /**
      @brief	Opens up a gnuplot session, ready to receive commands.
      @return	Newly allocated gnuplot control structure.

      This opens up a new gnuplot session, ready for input. The struct
      controlling a gnuplot session should remain opaque and only be
      accessed through the provided functions.

      The session must be closed using close().
      */
    GnuplotHandle * init();

    /**
      * @brief	Closes a gnuplot session previously opened by init()
      *
      * Kills the child PID and deletes all opened temporary files.
      * It is mandatory to call this function to close the handle, otherwise
      * temporary files are not cleaned and child process might survive.
      */
    void close();

    /** Handle to the output file if required */
    std::ofstream * m_pOutputStream;

    /** Name of the output file if required */
    std::string     m_sFileName;

    /** The encapsulated Gnuplot handle via which all commands are sent. */
    GnuplotHandle * m_pGnuplotHandle;

    /** Number of currently active plots */
    int             m_iPlots;

    /** Current plotting style */
    std::string     m_sPlotStyle;

    /** Output mode: Normal window, postscript etc.
     *  @see OutputMode */
    OutputMode      m_tOutputMode;

    /** Command line arguments handed over to Gnuplot at startup */
    std::string     m_sCmdArgs;

    /** The settings with which this interface was created */
    Settings        m_tSettings;
};

#endif // CROCO__UTILS__GNUPLOT_H
