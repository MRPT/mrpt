#ifndef CVD_INCLUDE_BGRX_H
#define CVD_INCLUDE_BGRX_H

#include <iostream>

namespace CVD {


//////////////////////////////
// CVD::Bgrx
/// A colour consisting of red, green, blue and dummy components,
/// in the order bgr dummy in memory.
/// @param T The datatype of each component
/// @ingroup gImage
template <typename T>
class Bgrx
{
public:
	/// Default constructor. Sets everything to 0.
	explicit Bgrx() : blue(0), green(0), red(0), dummy(0) {}

	/// Constructs a colour as specified
	/// @param r The red component
	/// @param g The green component
	/// @param b The blue component
	explicit Bgrx(T b, T g, T r) : blue(b), green(g), red(r), dummy(0) {}

   T blue; ///< The blue component
   T green; ///< The green component
   T red; ///< The red component
   T dummy; ///< The dummy

	/// Assignment operator between two different storage types, using the standard casts as necessary
	/// @param c The colour to copy from
   template <typename T2>
     Bgrx<T>& operator=(const Bgrx<T2>& c){
     blue = static_cast<T>(c.blue); 
     green = static_cast<T>(c.green); 
     red = static_cast<T>(c.red);
     return *this;
   }

	/// Logical equals operator. Returns true if each component is the same.
	/// @param c Bgrx to compare with
	bool operator==(const Bgrx<T>& c) const
      {return red == c.red && green == c.green && blue == c.blue;}

	/// Logical not-equals operator. Returns true unless each component is the same.
	/// @param c Bgrx to compare with
	bool operator!=(const Bgrx<T>& c) const
      {return red != c.red || green != c.green || blue != c.blue;}

//   T to_grey() const {return 0.3*red + 0.6*green + 0.1*blue;}
};

/// Write the colour to a stream in the format "(blue,green,red)"
/// @param os The stream
/// @param x The colour object
/// @relates Bgrx
template <typename T>
std::ostream& operator <<(std::ostream& os, const Bgrx<T>& x)
{
   return os << "(" << x.blue << ","
             << x.green << "," << x.red << ")";
}

/// Write the colour to a stream in the format "(blue,green,red)"
/// @param os The stream
/// @param x The colour object
/// @relates Bgrx
inline std::ostream& operator <<(std::ostream& os, const Bgrx<unsigned char>& x)
{
   return os << "(" 
             << static_cast<unsigned int>(x.blue) << ")"
             << static_cast<unsigned int>(x.green) << ","
             << static_cast<unsigned int>(x.red) << ",";
}


} // end namespace 
#endif

