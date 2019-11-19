#ifndef CVD_RGBA_H
#define CVD_RGBA_H

#include <iostream>

namespace CVD {


//////////////////////////////
// CVD::Rgba
// Template class to represent red, green, blue and alpha components
//
/// A colour consisting of red, green, blue and alpha components
/// @param T The datatype of each component
/// @ingroup gImage
template <typename T>
class Rgba
{
public:
	/// Default constructor. Sets everything to 0.
	explicit Rgba() : red(0), green(0), blue(0), alpha(0) {}

	/// Constructs a colour as specified
	/// @param r The red component
	/// @param g The green component
	/// @param b The blue component
	/// @param a The alpha component
	inline explicit Rgba(T r, T g, T b, T a) : red(r), green(g), blue(b), alpha(a) {}
	template <class S> inline explicit Rgba(const Rgba<S>& rgba) : red(static_cast<T>(rgba.red)), green(static_cast<T>(rgba.green)), blue(static_cast<T>(rgba.blue)), alpha(static_cast<T>(rgba.alpha)) {}

   T red; ///< The red component
   T green; ///< The green component
   T blue; ///< The blue component
   T alpha; ///< The alpha component

	/// Assignment operator between two different storage types, using the standard casts as necessary
	/// @param c The colour to copy from
   template <typename T2>
     Rgba<T>& operator=(const Rgba<T2>& c){
     red = static_cast<T>(c.red);
     green = static_cast<T>(c.green); 
     blue = static_cast<T>(c.blue); 
     alpha = static_cast<T>(c.alpha); 
     return *this;
   }

	/// Logical equals operator. Returns true if each component is the same.
	/// @param c Rgba to compare with
	bool operator==(const Rgba<T>& c) const
      {return red == c.red && green == c.green && blue == c.blue && alpha == c.alpha;}

	/// Logical not-equals operator. Returns true unless each component is the same.
	/// @param c Rgba to compare with
	bool operator!=(const Rgba<T>& c) const
      {return red != c.red || green != c.green || blue != c.blue || alpha != c.alpha;}

//   T to_grey() const {return 0.3*red + 0.6*green + 0.1*blue;}
};

/// Write the colour to a stream in the format "(red,green,blue,alpha)"
/// @param os The stream
/// @param x The colour object
/// @relates Rgba
template <typename T>
std::ostream& operator <<(std::ostream& os, const Rgba<T>& x)
{
   return os << "(" << x.red << "," << x.green << ","
             << x.blue << "," << x.alpha << ")";
}

/// Write the colour to a stream in the format "(red,green,blue,alpha)"
/// @param os The stream
/// @param x The colour object
/// @relates Rgba
inline std::ostream& operator <<(std::ostream& os, const Rgba<unsigned char>& x)
{
   return os << "(" << static_cast<unsigned int>(x.red) << ","
             << static_cast<unsigned int>(x.green) << ","
             << static_cast<unsigned int>(x.blue) << ","
             << static_cast<unsigned int>(x.alpha) << ")";
}


} // end namespace 
#endif

