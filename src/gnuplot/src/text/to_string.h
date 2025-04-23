#ifndef CROCO__UTILS__TO_STRING_H
#define CROCO__UTILS__TO_STRING_H

#include <cmath>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>


inline void setPrecision( std::ostream & ostr,
                          const unsigned int precision )
{
  ostr.setf( std::ios::fixed, std::ios::floatfield );
  ostr.precision( precision );
}


inline void resetPrecision( std::ostream & ostr )
{
  ostr.setf( std::ios::floatfield );
}


inline std::string toString( const bool b )
{
  return ( b ? "true" : "false" );
}


inline std::string toString( const int a )
{
  std::stringstream oss;

  if( std::isnan( a ))
  {
    oss << "nan";
  }
  else if( std::isinf( a ))
  {
    oss << ( a < 0 ? "-inf" : "inf" );
  }
  else
  {
    oss << a;
  }

  return oss.str();
}

/** Convert a long int to a string */
inline std::string toString( const long int a )
{
  std::stringstream oss;

  if( std::isnan( a )) {
    oss << "nan";
  } else if( std::isinf( a )) {
    oss << ( a < 0 ? "-inf" : "inf" );
  } else {
    oss << a;
  }

  return oss.str();
}

/** Convert an unsigned int to a string */
inline std::string toString( const unsigned int a )
{
  std::stringstream oss;

  if( std::isnan( a )) {
    oss << "nan";
  } else if( std::isinf( a )) {
    oss << "inf";
  } else {
    oss << a;
  }

  return oss.str();
}


/** Convert a long unsigned int to a string */
inline std::string toString( const unsigned long int a )
{
  std::stringstream oss;

  if( std::isnan( a )) {
    oss << "nan";
  } else if( std::isinf( a )) {
    oss << "inf";
  } else {
    oss << a;
  }

  return oss.str();
}


inline std::string toString( const float a,
                             const int decimals = -1 )
{
  std::stringstream oss;

  if( std::isnan( a ))
  {
    oss << "nan";
  }
  else if( std::isinf( a ))
  {
    oss << ( a < 0 ? "-inf" : "inf" );
  }
  else
  {
    float f = a;
    if( decimals > -1 )
    {
      int r = (int) ( f * (float) pow( 10.0, decimals ));

      f = (float) r / (float) pow( 10.0, decimals );
    }
    oss << f;
  }

  return oss.str();
}


inline std::string toString( const double a,
                             const int decimals = 0 )
{
  std::stringstream oss;

  if ( std::isnan( a ) )
  {
    oss << "nan";
  }
  else if ( std::isinf( a ) )
  {
    oss << ( a < 0 ? "-inf" : "inf" );
  }
  else
  {
    double d = a;
    if( decimals > 0 )
    {
      setPrecision( oss, decimals );
/*      int r = (int) ( d * (double) pow( 10.0, decimals ));
      d = (double) r / (double) pow( 10.0, decimals );*/
    }
    oss << d;
  }

  return oss.str();
}


/**
 * These will store different data types in
 * a char buffer. sizeof( type ) chars (bytes)
 * will be used.
 */

inline unsigned int bool2CharBuffer( bool a,
                                     char * buffer )
{
  for( unsigned int i = 0; i < sizeof( bool ); i++ )
  {
    buffer[i] = ((char *) &a)[i];
  }

  return sizeof( bool );
}

inline unsigned int int2CharBuffer( int a,
                                    char * buffer )
{
  for( unsigned int i = 0; i < sizeof( int ); i++ )
  {
    buffer[i] = ((char *) &a)[i];
  }

  return sizeof( int );
}

inline unsigned int float2CharBuffer( float a,
                                      char * buffer )
{
  for( unsigned int i = 0; i < sizeof( int ); i++ )
  {
    buffer[i] = ((char *) &a)[i];
  }

  return sizeof( float );
}

inline unsigned int double2CharBuffer( double a,
                                       char * buffer )
{
  for( unsigned int i = 0; i < sizeof( double ); i++ )
  {
    buffer[i] = ((char *) &a)[i];
  }

  return sizeof( double );
}

inline unsigned int long2CharBuffer( long a,
                                     char * buffer )
{
  for( unsigned int i = 0; i < sizeof( long ); i++ )
  {
    buffer[i] = ((char *) &a)[i];
  }

  return sizeof( long );
}

inline unsigned int string2CharBuffer( std::string a,
                                       char * buffer )
{
  for( unsigned int i = 0; i < a.length(); i++ )
  {
    buffer[i] = a[i];
  }

  return a.length();
}

inline unsigned int charBuffer2Bool( bool & result,
                                     const char * buffer )
{
  for( unsigned int i = 0; i < sizeof( bool ); i++ )
  {
    ((char *) &result)[i] = buffer[i];
  }

  return sizeof( bool );
}

inline unsigned int charBuffer2Int( int & result,
                                    const char * buffer )
{
  for( unsigned int i = 0; i < sizeof( int ); i++ )
  {
    ((char *) &result)[i] = buffer[i];
  }

  return sizeof( int );
}

inline unsigned int charBuffer2Float( float & result,
                                      const char * buffer )
{
  for( unsigned int i = 0; i < sizeof( float ); i++ )
  {
    ((char *) &result)[i] = buffer[i];
  }

  return sizeof( float );
}

inline unsigned int charBuffer2Double( double & result,
                                       const char * buffer )
{
  for( unsigned int i = 0; i < sizeof( double ); i++ )
  {
    ((char *) &result)[i] = buffer[i];
  }

  return sizeof( double );
}

inline unsigned int charBuffer2Long( long & result,
                                     const char * buffer )
{
  for( unsigned int i = 0; i < sizeof( long ); i++ )
  {
    ((char *) &result)[i] = buffer[i];
  }

  return sizeof( long );
}


#endif // CROCO__UTILS__TO_STRING_H
