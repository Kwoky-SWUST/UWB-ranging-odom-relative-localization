#ifndef CROCO__UTILS__DEC_HEX
#define CROCO__UTILS__DEC_HEX

#include <string>
#include <iostream>
#include <iomanip>

// --------------------------------------------------------------------------
inline unsigned char dec2hex( const unsigned char& i )
// --------------------------------------------------------------------------
{
  return ( i < 10 ) ? i+'0' : (i-10)+'A';
}


/** Converts a character representing the
 *  textual representation of a hexadecimal number into the represented
 *  integer number.
 */
// --------------------------------------------------------------------------
inline int hex2dec( const unsigned char& c )
// --------------------------------------------------------------------------
{
  int i;
  if ( c >= '0' && c <= '9' )
    i = c - '0';
  else if ( c >= 'a' && c <= 'f' )
    i = (c - 'a') + 10;
  else if ( c >= 'A' && c <= 'F' )
    i = (c - 'A') + 10;
  else
    i = -1;

  return i;
}

// --------------------------------------------------------------------------
inline std::string hexChar2String( const unsigned char& value )
// --------------------------------------------------------------------------
{
  std::string return_value( "--h" );
  return_value[0] = dec2hex( (value & 0xF0) / 16);
  return_value[1] = dec2hex( value & 0x0F );
  return return_value;
}


// --------------------------------------------------------------------------
inline std::string byteStringToHexString( const std::string& raw_byte_str )
// --------------------------------------------------------------------------
{
  std::string ret;
  for(int i=0; i<int(raw_byte_str.length()); ++i) {
    const unsigned char& curByte = raw_byte_str[i];
    ret.push_back( dec2hex((curByte >> 4) & 0x0F ) );
    ret.push_back( dec2hex( curByte       & 0x0F ) );
  }
  return ret;
}


/** Checks whether the input string consists of hex characters only
 *  @param sIn textual representation of a hexadecimal number
 *  @return bool value representing the result of the test
 */
// --------------------------------------------------------------------------
inline bool
isHexOnlyString( const std::string& sIn )
// --------------------------------------------------------------------------
{
  for( unsigned int iCnt=0; iCnt<sIn.length(); iCnt++)
  {
    if(! ((sIn[iCnt] >= '0' && sIn[iCnt] <= '9') ||
        (sIn[iCnt] >= 'a' && sIn[iCnt] <= 'f') ||
        (sIn[iCnt] >= 'A' && sIn[iCnt] <= 'F') ) )
    {
      return false;
    }
  }
  return true;
}



/* pos: 0=High Nibble, 1=Low Nibble */
inline void
bin2nibble(unsigned char in, unsigned char* out, unsigned int pos)
{
  unsigned char mask0 = 0xF0;
  unsigned char mask1 = 0x0F;
  if (pos == 0) {
    *out &= mask1; // clear High Nibble
    *out |= in<<4;
  }
  else {
    *out &= mask0; // clear Low Nibble
    *out |= in;
  }
}


inline unsigned char
ascii2bin(unsigned char c)
{
  if (c >= '0' && c <= '9')
    c -= '0';
  else if (c >= 'A' && c <= 'F')
    c -= 'A'-10;
  else if (c >= 'a' && c <= 'f')
    c -= 'a'-10;
  return c;
}


inline std::string
asciiString2binString(const std::string& in)
{
  unsigned char c = 0;
  int pos = 1;
  std::string out;

  for (unsigned int i = 0; i < in.size(); i++) {
    pos = i % 2;
    bin2nibble(ascii2bin(in[i]), &c, pos);
    if (pos)
      out += c;
  }
  if (pos == 0) {
    bin2nibble(0, &c, 1);
    out += c;
  }
  return out;
}



//--------------------------------------------
// Output
//--------------------------------------------


// --------------------------------------------------------------------------
inline void
printbin_char(const unsigned char& c, std::ostream& os )
// --------------------------------------------------------------------------
{
  // number of bits
  int size = sizeof(char) * 8;
  unsigned char mask = 0x80;

  for (int i = 0; i < size; i++) {
    if ((i % 4) == 0)
      os << " ";
    os << ((c & mask>>i) ? "1" : "0");
  }
}

// --------------------------------------------------------------------------
inline void
printbin_char(const unsigned char& c)
{
  printbin_char(c, std::cout );
}

// --------------------------------------------------------------------------
inline void
printbin_string(const unsigned char* c, int len, std::ostream& os)
// --------------------------------------------------------------------------
{
  for (int i = 0; i < len; i++) {
    if ((i % 8) == 0)
      os << std::endl;
    printbin_char(c[i]);
  }
  os << std::endl;
}

// --------------------------------------------------------------------------
inline void
printbin_string(const unsigned char* c, int len)
{
  printbin_string(c, len, std::cout);
}

// --------------------------------------------------------------------------
inline void
printbin_string(const std::string& str, std::ostream& os)
{
  printbin_string((const unsigned char*)str.c_str(), str.size(), os);
}

// --------------------------------------------------------------------------
inline void
printbin_string(const std::string& str)
{
  printbin_string(str, std::cout);
}


// --------------------------------------------------------------------------
inline void
printhex_string(const unsigned char* c, int len, std::ostream& os )
// --------------------------------------------------------------------------
{
  os << std::endl;
  int hi = 1;
  for (int i = 0; i < len; ) {
    char cc;
    if (hi) {
      cc = c[i] >> 4;
      os << std::hex << (int)cc;
      hi = 0;
    }
    else {
      cc = c[i] & 0x0F ;
      hi = 1;
      os << std::hex << (int)cc;
      os << " ";
      i++;
    }
  }
  os << std::dec << std::endl;
}

// --------------------------------------------------------------------------
inline void
printhex_string(const unsigned char* c, int len )
{
  printhex_string( c, len, std::cout );
}


// --------------------------------------------------------------------------
inline void
printhex_string(const std::string& str, std::ostream& os)
{
  printhex_string( (const unsigned char*)str.c_str(), str.length(), os );
}

// --------------------------------------------------------------------------
inline void
printhex_string(const std::string& str)
{
  printhex_string(str, std::cout);
}



#endif // CROCO__UTILS__DEC_HEX
