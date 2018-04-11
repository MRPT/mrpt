#ifndef __RGB8_H
#define __RGB8_H

namespace CVD {

/// @ingroup gImage

/// A 32-bit colour.
/// @b Deprecated The templated Rgba should be used instead (Rgba<byte> is exactly equivalent).
struct Rgb8
{
   unsigned char red; ///< The red component
   unsigned char green; ///< The green component
   unsigned char blue; ///< The blue component
   unsigned char dummy; ///< The 4th byte, usually either ignored or used to represent the alpha value

   /// Default constructor. Sets all elements to zero.
   explicit Rgb8() : red(0), green(0), blue(0), dummy(0) {}

   /// Construct an Rgb8 as specified
   /// @param r The red component
   /// @param g The green component
   /// @param b The blue component
   /// @param a The dummy byte (defaults to zero)
   explicit Rgb8(unsigned char r, unsigned char g, unsigned char b, unsigned char a=0)
   : red(r), green(g), blue(b), dummy(a) {}
};

} // end namespace 
#endif
