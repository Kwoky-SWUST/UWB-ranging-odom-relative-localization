#ifndef CROCO__UTILS__FONT_STYLE_H
#define CROCO__UTILS__FONT_STYLE_H

#include <string>


namespace fontstyle {

#ifdef ENABLE_OUTPUT_STYLING

  static std::string normal    = "\033[0m";
  static std::string bold      = "\033[6m";
  static std::string blink     = "\033[5m";
  static std::string underline = "\033[4m"; // see http://board.gulli.com/thread/48794-c-escape---sequenzen---liste-bzw-erluterung-/
  static std::string invert    = "\033[7m";

#else

  static std::string normal    = "";
  static std::string bold      = "";
  static std::string invert    = "";

#endif // ENABLE_OUTPUT_STYLING

}

#endif // CROCO__UTILS__FONT_STYLE_H
