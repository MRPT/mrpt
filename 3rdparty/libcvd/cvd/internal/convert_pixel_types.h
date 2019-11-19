#ifndef CVD_CONVERT_PIXEL_TYPES_H
#define CVD_CONVERT_PIXEL_TYPES_H

#include <math.h>
#include <type_traits>
#include <cstring>
#include <cvd/internal/scalar_convert.h>
#include <cvd/internal/builtin_components.h>
#include <cvd/internal/rgb_components.h>
#include <limits>

namespace CVD{namespace Pixel
{
  //All "conversion" classes must have a "void convert_pixel(from, to)"
  //nonstatic templated member.
  
  //This conversion knows how to convert from any size pixel to any other.
  
  template <class From, class To, int CF=Pixel::Component<From>::count, int CT=Pixel::Component<To>::count> struct GenericConversion;

  template <class From, class To> struct GenericConversion<From,To,1,1> {
    static inline void convert(const From& from, To& to) { 
      to = scalar_convert<To,From>(from);
    }
  };
  
  template <class From, class To, int N> struct GenericConversion<From,To,N,N> {
    typedef typename Pixel::Component<From>::type FromS;
    typedef typename Pixel::Component<To>::type ToS;
    static inline void convert(const From& from, To& to) { 
      for (int i=0; i<N; i++)
	Pixel::Component<To>::get(to,i) = scalar_convert<ToS,FromS>(Pixel::Component<From>::get(from,i));
    }
  };

  template <class T, int N> struct GenericConversion<T,T,N,N> { 
    static inline void convert(const T& from, T& to) { 
      to = from;
    }
  };

  template <class T, int N> struct GenericConversion<T[N],T[N], N, N> { 
    typedef T array[N];
    static inline void convert(const array& from, array& to) { 
      for (int i=0; i<N; i++)
	to[i] = from[i];
    }
  };

  template <class Rgbish, class Scalar> struct CIE {
    static inline void convert(const Rgbish& from, Scalar& to) {
      // "Correct" values are:
	  // const double wr=0.299, wg=0.587, wb=0.114;
	  // Since these are not representable as floating point values
	  // They do not sum exactly to 1.
	  // Instead, convert them using  floor(x *  0x10000 + 0.5)/0x10000
	  // So that they use exactly 16 bits of precision.
	  // This should be OK for pretty mych everything
	  // 0.299 ~= 19595/0x10000  == 0x4c8bp-16  == 0.2989959716796875
	  // 0.587 ~= 38470/0x10000  == 0x9646p-16  == 0.587005615234375
	  // 0.114 ~= 7471/0x10000   == 0x1d2fp-16  == 0.1139984130859375
	  // These will sum exactly to 1. C++ does not yet allow hex floating point
	  // literals
	  const double wr=0.2989959716796875, wg=0.587005615234375, wb=0.1139984130859375;
	  
	  /* 
      The old method does not work properly because the conversion to double from int_type
	  is inexact. Ttuncation of this causes errors. However, it is unnecessary to convert
	  to the double pixel type first: the arithmetic merely needs to be done with doubles.
	  
      to = scalar_convert<Scalar,double>(wr*scalar_convert<double,typename Pixel::Component<Rgbish>::type>(from.red) + 
					 wg*scalar_convert<double,typename Pixel::Component<Rgbish>::type>(from.green) + 
					 wb*scalar_convert<double,typename Pixel::Component<Rgbish>::type>(from.blue));
					 
	  Fortunately, we have forseen this eventuality, and scalar_convert can convert from
	  type A, to type B when pixel type B is held in a variable of type C
	  */
	  to = scalar_convert<Scalar,typename Pixel::Component<Rgbish>::type,double>(wr*from.red + wg*from.green + wb*from.blue);

	  /* The following method could be used (for speed):
	  
	  to = scalar_convert<Scalar,typename Pixel::Component<Rgbish>::type>(
				static_cast<typename Pixel::Component<Rgbish>::type>(wr*from.red + wg*from.green + wb*from.blue));

	  but this looses precision and may produce odd results for rgb(x,y,z) (x!=y!=z). Does this matter? Bear
	  in mind that wr, wg, wb are only approximations of "average" human eye response anyway.
	  */
	
    }
  };

  template <class P, class Scalar> struct Uniform {
    static inline void convert(const P& from, Scalar& to) {
      typedef typename Pixel::Component<P>::type T;
      typename traits<T>::wider_type sum = Pixel::Component<P>::get(from,0);
      for (unsigned int i=1; i<Pixel::Component<P>::count; i++)
	sum += Pixel::Component<P>::get(from,i);
      to = scalar_convert<Scalar,T>(sum/Pixel::Component<P>::count);
    }
  };

  template <class P, class Scalar> struct RMS {
    static inline void convert(const P& from, Scalar& to) {
      typedef typename Pixel::Component<P>::type T;
      double sum = Pixel::Component<P>::get(from,0);
      sum *= sum;
      for (unsigned int i=1; i<Pixel::Component<P>::count; i++) {
	double w = Pixel::Component<P>::get(from,i);
	sum += w*w;
      }
      to = scalar_convert<Scalar,T,double>(sqrt(sum/(Pixel::Component<P>::count)));
    }
  };  

  template <class P, class Scalar> struct L2Norm {
    static inline void convert(const P& from, Scalar& to) {
      typedef typename Pixel::Component<P>::type T;
      double sum = Pixel::Component<P>::get(from,0);
      sum *= sum;
      for (unsigned int i=1; i<Pixel::Component<P>::count; i++) {
	double w = Pixel::Component<P>::get(from,i);
	sum += w*w;
      }
      to = scalar_convert<Scalar,T,double>(sqrt(sum));
    }
  };  

  template <class P, class Scalar> struct SumOfSquares {
    static inline void convert(const P& from, Scalar& to) {
      typedef typename Pixel::Component<P>::type T;
      double sum = Pixel::Component<P>::get(from,0);
      sum *= sum;
      for (unsigned int i=1; i<Pixel::Component<P>::count; i++) {
    double w = Pixel::Component<P>::get(from,i);
    sum += w*w;
      }
      to = scalar_convert<Scalar,T,double>(sum);
    }
  };

  template <class Scalar, class Vec> struct Replicate {
    static inline void convert(const Scalar& from, Vec& to) {
      typedef typename Pixel::Component<Vec>::type T;
      Pixel::Component<Vec>::get(to,0) = scalar_convert<T, Scalar>(from);
      for (size_t i=1; i<Pixel::Component<Vec>::count; i++) 
	Pixel::Component<Vec>::get(to,i) = Pixel::Component<Vec>::get(to,0);
    }
  };
  
  template <class Scalar, class T> struct GreyToRgba {
    static inline void convert(const Scalar& from, Rgba<T>& to) {
      to.red = to.green = to.blue = scalar_convert<T, Scalar>(from);
      to.alpha = traits<T>::max_intensity;
    }
  };

  template <class A, class B> inline void RgbToRgb(const A& from, B& to) {
    typedef typename Pixel::Component<A>::type T;
    typedef typename Pixel::Component<B>::type S;
    to.red = scalar_convert<S,T>(from.red);
    to.green = scalar_convert<S,T>(from.green);
    to.blue = scalar_convert<S,T>(from.blue);
  }
  
  template <class A, class B> struct RgbishToRgbish {
    static inline void convert(const A& from, B& to) {
      RgbToRgb(from,to);
    }
  };
  
  template <class A,class T> struct RgbishToRgbish<A,Rgba<T> > {
    static inline void convert(const A& from, Rgba<T>& to) {
      RgbToRgb(from,to);
      to.alpha = traits<T>::max_intensity;
    }
  };
  
  template <class A,class T> struct RgbishToRgbish<A,Argb<T> > {
    static inline void convert(const A& from, Argb<T>& to) {
      RgbToRgb(from,to);
      to.alpha = traits<T>::max_intensity;
    }
  };

  template <class S,class T> struct RgbishToRgbish<Rgba<S>,Rgba<T> > {
    static inline void convert(const Rgba<S>& from, Rgba<T>& to) {
      RgbToRgb(from,to);
      to.alpha = scalar_convert<T,S>(from.alpha);
    }
  };
  
  template <class S,class T> struct RgbishToRgbish<Argb<S>,Argb<T> > {
    static inline void convert(const Argb<S>& from, Argb<T>& to) {
      RgbToRgb(from,to);
      to.alpha = scalar_convert<T,S>(from.alpha);
    }
  };
  
  

  // Default conversions

  template <class From, class To, int FN=Pixel::Component<From>::count, int TN=Pixel::Component<To>::count> struct DefaultConversion { 
    typedef GenericConversion<From,To> type; 
  };

  // Scalar to X
  template <class S, class T> struct DefaultConversion<S,Rgb<T>,1,3> { typedef Replicate<S,Rgb<T> > type; };
  template <class S> struct DefaultConversion<S,Rgb8,1,3> { typedef Replicate<S,Rgb8> type; };
  template <class S, class T> struct DefaultConversion<S,Bgrx<T>,1,3> { typedef Replicate<S,Bgrx<T> > type; };
  template <class S, class T> struct DefaultConversion<S,Rgba<T>,1,4> { typedef GreyToRgba<S,T> type; };

  // Rgb<T> to X
  template <class T, class S> struct DefaultConversion<Rgb<T>,S,3,1> { typedef CIE<Rgb<T>,S> type; };
  template <class T> struct DefaultConversion<Rgb<T>,Rgb8, 3,3> { typedef RgbishToRgbish<Rgb<T>, Rgb8> type; };
  template <class T, class S> struct DefaultConversion<Rgb<T>,Rgba<S>,3,4> { typedef RgbishToRgbish<Rgb<T>, Rgba<S> > type; };
  template <class T, class S> struct DefaultConversion<Rgb<T>,Rgb<S>,3,3> { typedef RgbishToRgbish<Rgb<T>, Rgb<S> > type; };
  template <class T> struct DefaultConversion<Rgb<T>,Rgb<T>,3,3> { typedef GenericConversion<Rgb<T>, Rgb<T> > type; };
  template <class T> struct DefaultConversion<Rgb<T>,Bgrx<T>,3,3> { typedef RgbishToRgbish<Rgb<T>, Bgrx<T> > type; };

  // Bgrx<T> to X
  template <class T, class S> struct DefaultConversion<Bgrx<T>,S,3,1> { typedef CIE<Bgrx<T>,S> type; };
  template <class T> struct DefaultConversion<Bgrx<T>,Rgb8, 3,3> { typedef RgbishToRgbish<Bgrx<T>, Rgb8> type; };
  template <class T, class S> struct DefaultConversion<Bgrx<T>,Rgba<S>,3,4> { typedef RgbishToRgbish<Bgrx<T>, Rgba<S> > type; };
  template <class T, class S> struct DefaultConversion<Bgrx<T>,Rgb<S>,3,3> { typedef RgbishToRgbish<Bgrx<T>, Rgb<S> > type; };
  template <class T> struct DefaultConversion<Bgrx<T>,Rgb<T>,3,3> { typedef GenericConversion<Bgrx<T>, Rgb<T> > type; };
  template <class T> struct DefaultConversion<Bgrx<T>,Bgrx<T>,3,3> { typedef RgbishToRgbish<Bgrx<T>, Bgrx<T> > type; };

  // Rgb8 to X
  template <class S> struct DefaultConversion<Rgb8,S,3,1> { typedef CIE<Rgb8,S> type; };
  template <class S> struct DefaultConversion<Rgb8,Rgb<S>,3,3> { typedef RgbishToRgbish<Rgb8, Rgb<S> > type; };
  template <class S> struct DefaultConversion<Rgb8,Rgba<S>,3,4> { typedef RgbishToRgbish<Rgb8, Rgba<S> > type; };
  template <class S> struct DefaultConversion<Rgb8,Bgrx<S>,3,4> { typedef RgbishToRgbish<Rgb8, Bgrx<S> > type; };
  template <> struct DefaultConversion<Rgb8,Rgb8,3,3> { typedef GenericConversion<Rgb8, Rgb8> type; };

  // Rgba<T> to X
  template <class T, class S> struct DefaultConversion<Rgba<T>,S,4,1> { typedef CIE<Rgba<T>,S> type; };
  template <class T, class S> struct DefaultConversion<Rgba<T>,Rgb<S>,4,3> { typedef RgbishToRgbish<Rgba<T>, Rgb<S> > type; };
  template <class T, class S> struct DefaultConversion<Rgba<T>,Bgrx<S>,4,3> { typedef RgbishToRgbish<Rgba<T>, Bgrx<S> > type; };
  template <class T> struct DefaultConversion<Rgba<T>,Rgb8,4,3> { typedef RgbishToRgbish<Rgba<T>, Rgb8> type; };
  template <class T, class S> struct DefaultConversion<Rgba<T>,Rgba<S>,4,4> { typedef RgbishToRgbish<Rgba<T>, Rgba<S> > type; };
  template <class T, class S> struct DefaultConversion<Rgba<T>,Argb<S>,4,4> { typedef RgbishToRgbish<Rgba<T>, Argb<S> > type; };
  template <class T> struct DefaultConversion<Rgba<T>,Rgba<T>,4,4> { typedef GenericConversion<Rgba<T>, Rgba<T> > type; };
  

  // Argb<T> to X
  template <class T, class S> struct DefaultConversion<Argb<T>,S,4,1> { typedef CIE<Argb<T>,S> type; };
  template <class T, class S> struct DefaultConversion<Argb<T>,Rgb<S>,4,3> { typedef RgbishToRgbish<Argb<T>, Rgb<S> > type; };
  template <class T> struct DefaultConversion<Argb<T>,Rgb8,4,3> { typedef RgbishToRgbish<Argb<T>, Rgb8> type; };
  template <class T, class S> struct DefaultConversion<Argb<T>,Rgba<S>,4,4> { typedef RgbishToRgbish<Argb<T>, Rgba<S> > type; };
  template <class T> struct DefaultConversion<Argb<T>,Argb<T>,4,4> { typedef GenericConversion<Argb<T>, Argb<T> > type; };

  
  template <class From, class To, class Conv=typename DefaultConversion<From,To>::type, 
    bool both_pod=std::is_trivially_copyable<From>::value && std::is_trivially_copyable<To>::value> struct ConvertPixels {
    static inline void convert(const From* from, To* to, size_t count) {
      for (size_t i=0; i<count; i++) 
	Conv::convert(from[i], to[i]);
    }
  };

  template <class T> struct ConvertPixels<T,T,GenericConversion<T,T>,true> {
    static inline void convert(const T* from, T* to, size_t count) {
      std::memcpy(to, from, count*sizeof(T));
    }
  };
	
  ///All pixel types which are DefaultConvertible can be converted freely from one
  ///to another. This struct indicated which types are DefaultConvertible. All 
  ///bulitin numeric types, Rgb of them, Rgba and Rgb8 etc fall in to this class.
  /// @ingroup gImageIO
  template<class C> struct DefaultConvertible
  {
  	static const int is = std::numeric_limits<C>::is_specialized;
  };

  template<class C> struct DefaultConvertible<Rgb<C> >
  {
  	static const int is = std::numeric_limits<C>::is_specialized;
  };

  template<class C> struct DefaultConvertible<Argb<C> >
  {
  	static const int is = std::numeric_limits<C>::is_specialized;
  };

  template<class C> struct DefaultConvertible<Rgba<C> >
  {
  	static const int is = std::numeric_limits<C>::is_specialized;
  };

  template<class C> struct DefaultConvertible<Bgrx<C> >
  {
  	static const int is = std::numeric_limits<C>::is_specialized;
  };

  template<> struct DefaultConvertible<Rgb8>
  {
  	static const int is = 1;
  };
  
}}
#endif
