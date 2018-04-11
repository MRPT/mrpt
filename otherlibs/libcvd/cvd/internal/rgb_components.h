#ifndef CVD_RGB_TRAITS_H
#define CVD_RGB_TRAITS_H

#include <cvd/rgb.h>
#include <cvd/rgba.h>
#include <cvd/rgb8.h>
#include <cvd/argb.h>
#include <cvd/bgrx.h>
#include <cvd/la.h>
#include <cvd/internal/builtin_components.h>
#include <cvd/internal/pixel_traits.h>

namespace CVD
{
	namespace Pixel
	{
		
		template<class P> struct Component<Rgb<P> >
		{
			typedef P type;
			static const size_t count = 3;

			
			//This version is much faster, with -funroll-loops
			static const P& get(const Rgb<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<const P*>(&pixel)+i);
				//return i == 0 ? pixel.red : (i==1 ? pixel.green : pixel.blue);
			}

			static P& get(Rgb<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<P*>(&pixel)+i);
				// return i == 0 ? pixel.red : (i==1 ? pixel.green : pixel.blue);
			}
		};

		template<class P> struct Component<Bgrx<P> >
		{
			typedef P type;
			static const size_t count = 3;

			
			//This version is much faster, with -funroll-loops
			static const P& get(const Bgrx<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<const P*>(&pixel)+i);
				//return i == 0 ? pixel.blue : (i==1 ? pixel.green : pixel.red);
			}

			static P& get(Bgrx<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<P*>(&pixel)+i);
				// return i == 0 ? pixel.blue : (i==1 ? pixel.green : pixel.red);
			}
		};


		template<> struct Component<Rgb8>
		{
			typedef unsigned char type;
			static const size_t count = 3;

			static const type& get(const Rgb8& pixel, size_t i)
			{
				return *(reinterpret_cast<const unsigned char*>(&pixel)+i);
				//return i == 0 ? pixel.red : (i==1 ? pixel.green : pixel.blue);
			}

			static type& get(Rgb8& pixel, size_t i)
			{
				return *(reinterpret_cast<unsigned char*>(&pixel)+i);
				//return i == 0 ? pixel.red : (i==1 ? pixel.green : pixel.blue);
			}
		};

		template<class P> struct Component<Rgba<P> >
		{
			typedef P type;
			static const size_t count = 4;

			static const P& get(const Rgba<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<const P*>(&pixel)+i);
				//return i == 0 ? pixel.red : (i==1 ? pixel.green : (i==2 ?pixel.blue: pixel.alpha));
			}

			static P& get(Rgba<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<P*>(&pixel)+i);
				//return i == 0 ? pixel.red : (i==1 ? pixel.green : (i==2 ?pixel.blue: pixel.alpha));
			}
		};

		template<class P> struct Component<La<P> >
		{
			typedef P type;
			static const size_t count = 2;

			static const P& get(const La<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<const P*>(&pixel)+i);
			}

			static P& get(La<P>& pixel, size_t i)
			{
				return *(reinterpret_cast<P*>(&pixel)+i);
			}
		};

		template<class P> struct Component<Argb<P> >
		{
			typedef P type;
			static const size_t count = 4;

			static const P& get(const Argb<P>& pixel, size_t i)
			{
				//return *(reinterpret_cast<const P*>(&pixel)+i);
				return i == 0 ? pixel.red : (i==1 ? pixel.green : (i==2 ?pixel.blue: pixel.alpha));
			}

			static P& get(Argb<P>& pixel, size_t i)
			{
				//return *(reinterpret_cast<P*>(&pixel)+i);
				return i == 0 ? pixel.red : (i==1 ? pixel.green : (i==2 ?pixel.blue: pixel.alpha));
			}
		};

		template <class T> struct is_Rgb { enum { value = 0 }; };
		template <class T> struct is_Rgb<Rgb<T> > { enum { value = 1 }; };
		template <> struct is_Rgb<Rgb8> { enum { value = 1 }; };
		template <class T> struct is_Rgb<Rgba<T> > { enum { value = 1 }; };
		template <class T> struct is_Rgb<Argb<T> > { enum { value = 1 }; };

		template<class T, int LIFT> struct traits<Rgb<T>, LIFT> 
		{ 
		  typedef Rgb<typename Pixel::traits<T>::wider_type> wider_type; 
		  typedef Rgb<typename Pixel::traits<T>::float_type> float_type;
		};

		template<class T, int LIFT> struct traits<Rgba<T>, LIFT> 
		{ 
		  typedef Rgba<typename Pixel::traits<T>::wider_type> wider_type; 
		  typedef Rgba<typename Pixel::traits<T>::float_type> float_type;
		};

		template<class T, int LIFT> struct traits<La<T>, LIFT> 
		{ 
		  typedef La<typename Pixel::traits<T>::wider_type> wider_type; 
		  typedef La<typename Pixel::traits<T>::float_type> float_type;
		};
  
		template<int LIFT> struct traits<Rgb8, LIFT> 
		{ 
		  typedef Rgb<int> wider_type; 
		  typedef Rgb<float> float_type;
		};

	}
		template <class T> struct Rgb_ops {
		  template <class S> static inline T sum(const T& a, const S& b) { return T(a.red+b.red, a.green+b.green, a.blue+b.blue); }
		  template <class S> static inline void add(T& a, const S& b) { a.red+=b.red; a.green+=b.green; a.blue+=b.blue; }
		  template <class S> static inline T diff(const T& a, const S& b) { return T(a.red-b.red, a.green-b.green, a.blue-b.blue); }
		  template <class S> static inline void sub(T& a, const S& b) { a.red-=b.red; a.green-=b.green; a.blue-=b.blue; }
		  template <class S> static inline T prod(const T& a, const S& b) { return T(a.red*b, a.green*b, a.blue*b); }
		  template <class S> static inline void mul(T& a, const S& b) { a.red*=b; a.green*=b; a.blue*=b; }
		  template <class S> static inline T quot(const T& a, const S& b) { return T(a.red/b, a.green/b, a.blue/b); }
		  template <class S> static inline void div(T& a, const S& b) { a.red/=b; a.green/=b; a.blue/=b; }
		  template <class S> static inline void assign(T& a, const S& b) { a.red=b.red; a.green=b.green; a.blue=b.blue; }
		};

		template <class T, class S> inline Rgb<T> operator+(const Rgb<T>& a, const Rgb<S>& b) { return Rgb_ops<Rgb<T> >::sum(a,b); }
		template <class T, class S> inline Rgb<T>& operator+=(Rgb<T>& a, const Rgb<S>& b) { Rgb_ops<Rgb<T> >::add(a,b); return a; }
		template <class T, class S> inline Rgb<T> operator-(const Rgb<T>& a, const Rgb<S>& b) { return Rgb_ops<Rgb<T> >::diff(a,b); }
		template <class T, class S> inline Rgb<T>& operator-=(Rgb<T>& a, const Rgb<S>& b) { Rgb_ops<Rgb<T> >::sub(a,b); return a; }
		template <class T, class S> inline Rgb<T> operator*(const Rgb<T>& a, const S& b) { return Rgb_ops<Rgb<T> >::prod(a,b); }
		template <class T, class S> inline Rgb<T> operator*(const S& b, const Rgb<T>& a) { return Rgb_ops<Rgb<T> >::prod(a,b); }
		template <class T, class S> inline Rgb<T>& operator*=(Rgb<T>& a, const S& b) { Rgb_ops<Rgb<T> >::mul(a,b); return a; }
		template <class T, class S> inline Rgb<T> operator/(const Rgb<T>& a, const S& b) { return Rgb_ops<Rgb<T> >::quot(a,b); }
		template <class T, class S> inline Rgb<T> operator/(const S& b, const Rgb<T>& a) { return Rgb_ops<Rgb<T> >::quot(a,b); }
		template <class T, class S> inline Rgb<T>& operator/=(Rgb<T>& a, const S& b) { Rgb_ops<Rgb<T> >::div(a,b); return a; }


		template <class T> struct Rgba_ops {
		  template <class S> static inline T sum(const T& a, const S& b) { return T(a.red+b.red, a.green+b.green, a.blue+b.blue, a.alpha+b.alpha); }
		  template <class S> static inline void add(T& a, const S& b) { a.red+=b.red; a.green+=b.green; a.blue+=b.blue; a.alpha+=b.alpha;}
		  template <class S> static inline T diff(const T& a, const S& b) { return T(a.red-b.red, a.green-b.green, a.blue-b.blue, a.alpha-b.alpha); }
		  template <class S> static inline void sub(T& a, const S& b) { a.red-=b.red; a.green-=b.green; a.blue-=b.blue; a.alpha-=b.alpha; }
		  template <class S> static inline T prod(const T& a, const S& b) { return T(a.red*b, a.green*b, a.blue*b, a.alpha*b); }
		  template <class S> static inline void mul(T& a, const S& b) { a.red*=b; a.green*=b; a.blue*=b; a.alpha*=b; }
		  template <class S> static inline T quot(const T& a, const S& b) { return T(a.red/b, a.green/b, a.blue/b, a.alpha/b); }
		  template <class S> static inline void div(T& a, const S& b) { a.red/=b; a.green/=b; a.blue/=b; a.alpha/=b;}
		  template <class S> static inline void assign(T& a, const S& b) { a.red=b.red; a.green=b.green; a.blue=b.blue; a.alpha=b.alpha; }
		};

		template <class T, class S> inline Rgba<T> operator+(const Rgba<T>& a, const Rgba<S>& b) { return Rgba_ops<Rgba<T> >::sum(a,b); }
		template <class T, class S> inline Rgba<T>& operator+=(Rgba<T>& a, const Rgba<S>& b) { Rgba_ops<Rgba<T> >::add(a,b); return a; }
		template <class T, class S> inline Rgba<T> operator-(const Rgba<T>& a, const Rgba<S>& b) { return Rgba_ops<Rgba<T> >::diff(a,b); }
		template <class T, class S> inline Rgba<T>& operator-=(Rgba<T>& a, const Rgba<S>& b) { Rgba_ops<Rgba<T> >::sub(a,b); return a; }
		template <class T, class S> inline Rgba<T> operator*(const Rgba<T>& a, const S& b) { return Rgba_ops<Rgba<T> >::prod(a,b); }
		template <class T, class S> inline Rgba<T> operator*(const S& b, const Rgba<T>& a) { return Rgba_ops<Rgba<T> >::prod(a,b); }
		template <class T, class S> inline Rgba<T>& operator*=(Rgba<T>& a, const S& b) { Rgba_ops<Rgba<T> >::mul(a,b); return a; }
		template <class T, class S> inline Rgba<T> operator/(const Rgba<T>& a, const S& b) { return Rgba_ops<Rgba<T> >::quot(a,b); }
		template <class T, class S> inline Rgba<T> operator/(const S& b, const Rgba<T>& a) { return Rgba_ops<Rgba<T> >::quot(a,b); }
		template <class T, class S> inline Rgba<T>& operator/=(Rgba<T>& a, const S& b) { Rgba_ops<Rgba<T> >::div(a,b); return a; }

		template <class T> struct La_ops {
		  template <class S> static inline T sum(const T& a, const S& b) { return T(a.luminance+b.luminance, a.alpha+b.alpha); }
		  template <class S> static inline void add(T& a, const S& b) { a.luminance+=b.luminance; a.alpha+=b.alpha;}
		  template <class S> static inline T diff(const T& a, const S& b) { return T(a.luminance-b.luminance, a.alpha-b.alpha); }
		  template <class S> static inline void sub(T& a, const S& b) { a.luminance-=b.luminance; a.alpha-=b.alpha; }
		  template <class S> static inline T prod(const T& a, const S& b) { return T(a.luminance*b, a.alpha*b); }
		  template <class S> static inline void mul(T& a, const S& b) { a.luminance*=b; a.alpha*=b; }
		  template <class S> static inline T quot(const T& a, const S& b) { return T(a.luminance/b, a.alpha/b); }
		  template <class S> static inline void div(T& a, const S& b) { a.luminance/=b; a.alpha/=b;}
		  template <class S> static inline void assign(T& a, const S& b) { a.luminance=b.luminance; a.alpha=b.alpha; }
		};

		template <class T, class S> inline La<T> operator+(const La<T>& a, const La<S>& b) { return La_ops<La<T> >::sum(a,b); }
		template <class T, class S> inline La<T>& operator+=(La<T>& a, const La<S>& b) { La_ops<La<T> >::add(a,b); return a; }
		template <class T, class S> inline La<T> operator-(const La<T>& a, const La<S>& b) { return La_ops<La<T> >::diff(a,b); }
		template <class T, class S> inline La<T>& operator-=(La<T>& a, const La<S>& b) { La_ops<La<T> >::sub(a,b); return a; }
		template <class T, class S> inline La<T> operator*(const La<T>& a, const S& b) { return La_ops<La<T> >::prod(a,b); }
		template <class T, class S> inline La<T> operator*(const S& b, const La<T>& a) { return La_ops<La<T> >::prod(a,b); }
		template <class T, class S> inline La<T>& operator*=(La<T>& a, const S& b) { La_ops<La<T> >::mul(a,b); return a; }
		template <class T, class S> inline La<T> operator/(const La<T>& a, const S& b) { return La_ops<La<T> >::quot(a,b); }
		template <class T, class S> inline La<T> operator/(const S& b, const La<T>& a) { return La_ops<La<T> >::quot(a,b); }
		template <class T, class S> inline La<T>& operator/=(La<T>& a, const S& b) { La_ops<La<T> >::div(a,b); return a; }

		
	
}
#endif
