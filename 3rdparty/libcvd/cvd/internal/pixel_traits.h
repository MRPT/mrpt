#ifndef CVD_PIXEL_TRAITS_H_
#define CVD_PIXEL_TRAITS_H_

#include <limits>

#if defined (CVD_HAVE_TOON)
#include <TooN/TooN.h>
#endif

namespace CVD {
namespace Pixel {
	
	//This is required for MIPSPro, since it is able to deduce more than gcc 3.3
	//before a template is instantiated
	template<class T> struct traits_error
	{
	};
	
	// LIFT is a dummy parameter to lift the partial specialisations of traits 
	// on various types into templates again. Then the visibility of static const
	// members is sorted out by the compiler correctly.
	template<class T, int LIFT=0> struct traits: public traits_error<T>
	{
		static const bool integral=traits_error<T>::Error_trait_not_defined_for_this_class;
	};

	template<int LIFT> struct traits<unsigned char, LIFT> 
	{ 
		typedef int wider_type;
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 8;
		static const unsigned char max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<char, LIFT> 
	{ 
		typedef int wider_type;
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = std::numeric_limits<char>::is_signed;
		static const int bits_used = std::numeric_limits<char>::digits;
		static const char max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<signed char, LIFT> 
	{ 
		typedef int wider_type;
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 7;
		static const signed char max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<short, LIFT> 
	{ 
		typedef int wider_type;
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = true;
		static const int bits_used = 15;
		static const short max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<unsigned short, LIFT> 
	{ 
		typedef int wider_type;
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 16;
		static const unsigned short max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<int, LIFT> 
	{ 
		typedef int wider_type; 
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = true;
		static const int bits_used = 16;
		static const int max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<unsigned int, LIFT> 
	{ 
		typedef unsigned int wider_type; 
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 16;
		static const unsigned int max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<long, LIFT> 
	{ 
		typedef int wider_type; 
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = true;
		static const int bits_used = 16;
		static const long max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<long long, LIFT> 
	{ 
		typedef long long wider_type; 
		typedef double float_type;
		static const bool integral = true;
		static const bool is_signed = true;
		static const int bits_used = 31;
		static const long long max_intensity=(1ll << bits_used) - 1ll; 
	};

	template<int LIFT> struct traits<unsigned long long, LIFT> 
	{ 
		typedef unsigned long long wider_type; 
		typedef double float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 31;
		static const unsigned long long max_intensity=(1ull << bits_used) - 1ull; 
	};

	template<int LIFT> struct traits<unsigned long, LIFT> 
	{ 
		typedef unsigned int wider_type; 
		typedef float float_type;
		static const bool integral = true;
		static const bool is_signed = false;
		static const int bits_used = 16;
		static const long max_intensity=(1 << bits_used) - 1; 
	};

	template<int LIFT> struct traits<float, LIFT> 
	{ 
		typedef float wider_type; 
		typedef float float_type;
		static const bool integral = false;
		static const bool is_signed = true;
		static const float max_intensity; 
	};

    template<int LIFT> const float traits<float, LIFT>::max_intensity = 1.0f;

	template<int LIFT> struct traits<double, LIFT> 
	{ 
		typedef double wider_type; 
		typedef double float_type;
		static const bool integral = false;
		static const bool is_signed = true;
		static const double max_intensity; 
	};
	
	template<int LIFT> const double traits<double, LIFT>::max_intensity = 1.0;

	template<int LIFT> struct traits<long double, LIFT> 
	{ 
		typedef long double wider_type; 
		typedef long double float_type;
		static const bool integral = false;
		static const bool is_signed = true;
		static const long double max_intensity; 
	};

	template<int LIFT> struct traits<bool, LIFT>
	{
	    typedef int wider_type;  // int holds a sum of many bools
	    typedef float float_type; // which floating point type can hold them?
	    static const bool integral = true; // bool is integral
	    static const bool is_signed = false; // bool is unsigned
	    static const int bits_used = 1; // only one bit
	    static const bool max_intensity= true;  // the 'high' value
	};


#if defined (CVD_HAVE_TOON)
    template<int N> struct traits<TooN::Vector<N> >
    {
	typedef TooN::Vector<N> wider_type;
	typedef TooN::Vector<N> float_type;
	static const bool integral = false;
	static const bool is_signed = true;
	static const TooN::Vector<N> max_intensity;
    };
    template <int N> const TooN::Vector<N> traits<TooN::Vector<N> >::max_intensity = TooN::Vector<N>(1.0);

    template<int N, int M> struct traits<TooN::Matrix<N,M> >
    {
      typedef TooN::Matrix<N,M> wider_type;
      typedef TooN::Matrix<N,M> float_type;
      static const bool integral = false;
      static const bool is_signed = true;
    };

#endif


    template<int LIFT> const long double traits<long double, LIFT>::max_intensity = 1.0;

	template<class C> struct indirect_type
	{
		typedef C type;
	};

	template<class C, int N, int LIFT> struct traits<C[N], LIFT>
	{
		typedef typename indirect_type<typename traits<C>::wider_type[N]>::type  wider_type;
		typedef typename indirect_type<typename traits<C>::float_type[N]>::type  float_type;
	};
}
}

#endif
