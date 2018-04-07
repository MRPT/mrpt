#ifndef CVD_VECTOR_IMAGE_REF_H
#define CVD_VECTOR_IMAGE_REF_H

#include <TooN/TooN.h>
#include <cvd/image_ref.h>

namespace CVD
{
	/// Convert an image co-ordinate into a Vector
	/// @param ir The ImageRef to convert
	/// @ingroup gImage
	inline TooN::Vector<2> vec(const ImageRef& ir)
	{
		TooN::Vector<2> r;
		r[0] = ir.x;
		r[1] = ir.y;
		return r;
	}

	/// Convert a Vector into an image co-ordinate. Numbers are truncated, not rounded
	/// @param v The Vector to convert
	/// @ingroup gImage
	inline ImageRef ir(const TooN::Vector<2>& v)
	{
		return ImageRef((int)v[0], (int)v[1]);
	}

	/// Convert a Vector into an image co-ordinate. Numbers are rounded
	/// @param v The Vector to convert
	/// @ingroup gImage
	inline ImageRef ir_rounded(const TooN::Vector<2>& v)
	{
		return ImageRef(
      static_cast<int>(v[0] > 0.0 ? v[0] + 0.5 : v[0] - 0.5),
      static_cast<int>(v[1] > 0.0 ? v[1] + 0.5 : v[1] - 0.5));
	}

	/// Rescale an ImageRef by a scaling factor
	/// @param v The Vector to convert
	/// @ingroup gImage
	inline ImageRef ir_rescale_rounded(const ImageRef &v, double rescaling_factor)
	{
	  return ir_rounded(vec(v) * rescaling_factor);
	}

}


#endif
