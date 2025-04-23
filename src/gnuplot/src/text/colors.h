#ifndef CROCO__UTILS__COLORS_H
#define CROCO__UTILS__COLORS_H

#include <string>


/** Collection of ANSI escape sequences which set the text output foreground
 *  color. */
namespace color
{

#ifdef ENABLE_OUTPUT_STYLING

  static std::string black(      "\033[0;30m");
  static std::string darkgray(   "\033[1;30m");
  static std::string red(        "\033[0;31m");
  static std::string lightred(   "\033[1;31m");
  static std::string green(      "\033[0;32m");
  static std::string lightgreen( "\033[1;32m");
  static std::string brown(      "\033[0;33m");
  static std::string yellow(     "\033[1;33m");
  static std::string blue(       "\033[0;34m");
  static std::string lightblue(  "\033[1;34m");
  static std::string purple(     "\033[0;35m");
  static std::string lightpurple("\033[1;35m");
  static std::string cyan(       "\033[0;36m");
  static std::string lightcyan(  "\033[1;36m");
  static std::string lightgray(  "\033[0;37m");
  static std::string white(      "\033[1;37m");
  static std::string normal(     "\033[0;39m");

#else

  static std::string black      = "";
  static std::string darkgray   = "";
  static std::string red(        "" );
  static std::string lightred(   "" );
  static std::string green = "";
  static std::string lightgreen( "" );
  static std::string brown(      "" );
  static std::string yellow(     "" );
  static std::string blue(       "" );
  static std::string lightblue(  "" );
  static std::string purple(     "" );
  static std::string lightpurple("" );
  static std::string cyan(       "" );
  static std::string lightcyan(  "" );
  static std::string lightgray(  "" );
  static std::string white(      "" );
  static std::string normal(     "" );

#endif // #ifdef ENABLE_OUTPUT_STYLING

}


/** Collection of ANSI escape sequences which set the text output background
 *  color. */
namespace bgcolor
{

#ifdef ENABLE_OUTPUT_STYLING

  static std::string black(      "\033[40m");
  static std::string red(        "\033[41m");
  static std::string green(      "\033[42m");
  static std::string yellow(     "\033[43m");
  static std::string blue(       "\033[44m");
  static std::string purple(     "\033[45m");
  static std::string cyan(       "\033[46m");
  static std::string white(      "\033[47m");
  static std::string normal(     "\033[49m");

#else

  static std::string black  = "";
  static std::string red    = "";
  static std::string green  = "";
  static std::string yellow = "";
  static std::string blue   = "";
  static std::string purple = "";
  static std::string cyan   = "";
  static std::string white  = "";
  static std::string normal = "";

#endif // #ifdef ENABLE_OUTPUT_STYLING
}

#endif // CROCO__UTILS__COLORS_H
