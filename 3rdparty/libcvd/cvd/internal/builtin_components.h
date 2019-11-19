#ifndef CVD_BUILTIN_TRAITS_H_
#define CVD_BUILTIN_TRAITS_H_

#include <limits>
#include <cstddef>

#if defined (CVD_HAVE_TOON)
#include <TooN/TooN.h>
#endif




namespace CVD
{
  namespace Pixel
  {

    //The "Component" class gives us information about the pixel components, 
    //ie how many components there are and whay the type is.
		
    //We use a 2 layer thing here so that Component is only properly defined
    //for builtin types, unless explicitly overridden.

    template<class P, int spp> struct component_base {
      template<bool x> struct component_base_only_for_basic_types;
      static const int fail = sizeof(component_base_only_for_basic_types<false>);
    };
	  
    template<class P> struct component_base<P, 1>
    {
    };

    //template <class P, int primitive = std::numeric_limits<P>::is_specialized> struct Component;

    template<class P> struct Component
    {
      typedef P type;
      static const size_t count = 1;
		  
      static const P& get(const P& pixel, size_t)
      {
	return pixel;
      }

      static P& get(P& pixel, size_t)
      {
	return pixel;
      }
			
    };

    template<class P, int I> struct Component<P[I]>
    {
      typedef P type;
      static const size_t count=I;
      
      static const P& get(const P pixel[I], size_t i)
      {
	return pixel[i];
      }

      static inline P& get(P pixel[I], size_t i)
      {
	return pixel[i];
      }
    };

#if defined (CVD_HAVE_TOON)
    template<int N, typename P> struct Component<TooN::Vector<N, P> >
    {
      typedef P type;
      static const size_t count=N;
      
      static inline const P & get(const TooN::Vector<N, P>& pixel, size_t i)
      {
          return pixel[i];
      }

      static inline P& get(TooN::Vector<N, P>& pixel, size_t i)
      {
	return pixel[i];
      }
    };

    template<int N, int M, typename P> struct Component<TooN::Matrix<N,M, P> >
    {
      typedef P type;
      static const size_t count=N*M;
      
      static const P& get(const TooN::Matrix<N,M,P>& pixel, size_t i)
      {
	return pixel[i/M][i%M];
      }

      static inline P& get(TooN::Matrix<N,M,P>& pixel, size_t i)
      {
	return pixel[i/M][i%M];
      }
    };
#endif
		
  }
}

#endif
