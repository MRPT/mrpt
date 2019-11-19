#ifndef CVD_SCALAR_CONVERT_H
#define CVD_SCALAR_CONVERT_H

#include <cvd/byte.h>
#include <cvd/internal/pixel_traits.h>

namespace CVD
{
namespace Pixel
{

	namespace Internal
	{

		//When we convert, we want the following thing to happen:
		//conv(max_low_precision_number) == max_high_precision_number
		//Since max is all ones, rightshifting truncates, resulting in all ones
		//Left shifting is more tricky.
		//All remaining empty bits need to be filled with the higest bits of the low precision number
		//That applies recursively (consider converting byte to ulong)
		//As an illustration, imagine converting the 4 bit umber 1011 to  a 10 bit number. The
		//result should be:
		//1011 -> 1011 1011 10
		//
		// In other words, it's:
		// truncate( 1011 * 1000100010.00100010001000100010001000...
		// 
		// Which is equal to multiplying by (high_precision_max + 1)/(low_precision_max)
		// Where strict rruncation occurs, ie 1.1111111... trucates to 1
		//
		template<class To, class From> struct int_info {
		  //Difference in number of bits used
		  static const int diff=traits<To>::bits_used - traits<From>::bits_used;

		  //Extra bits required to fill the space bits
		  //Number of complete copies required
		  static const int chunks=traits<To>::bits_used / traits<From>::bits_used;
		  //Number of extra bits
		  static const int extra_bits  =traits<To>::bits_used % traits<From>::bits_used;
		  //Right shift required to leave extra bits behind:
		  static const int final_rshift = traits<From>::bits_used - extra_bits;
		  
		  //Which way do we need to shift
		  static const int shift_dir =   (diff == 0)?0:((diff > 0)?1:-1);
		};
		
		template<class To, int num, int shift, int bits, int r_shift> struct upshift
		{
			static To aggregate(To i)
			{
				return i << shift | upshift<To,num-1,shift-bits,bits, r_shift>::aggregate(i);
			}
		};
		template<class To, int shift, int bits, int r_shift> struct upshift<To,0,shift,bits,r_shift>
		{
			static To aggregate(To i)
			{
				return i >> r_shift;
			}
		};
	
		template<class To, class From, int i=int_info<To,From>::shift_dir> struct shift_convert {
		  template <class D> static To from(D f) {
		    return static_cast<To>(f);
		  }		  
		};

		template<class To, class From> struct shift_convert<To, From, 1> 
		{
		  typedef int_info<To,From> info;
		  template <class D> static To from(D f) {
		    //return static_cast<To>(f) << int_info<To,From>::diff;
			return upshift<To,info::chunks, info::diff, traits<From>::bits_used, info::final_rshift>::aggregate(static_cast<To>(f));
		  }
		};
		
		template<class To, class From> struct shift_convert<To, From,-1> {	
		  template <class D> static To from(D f)  {
		    return static_cast<To>(f >> -int_info<To,From>::diff);
		  }
		};
		
		namespace Internal
		{
			//Trivial constexpr array since std::array isn't.
			template<class T, int N>
			struct trivial_array
			{
				T elems[N];

				constexpr T& operator[](size_t i)
				{
					return elems[i];
				}

				constexpr const T& operator[](size_t i) const
				{
					return elems[i];
				}
			};

			template <class S> 
			constexpr trivial_array<S, 512> buildLookupTable()
			{
				trivial_array<S, 512> table =  {};
				for (int i=0; i<=511; i++)
					table[i] = (S)((i-255)/255.0);    

				return table;
			}

			constexpr static trivial_array<float, 512> float_for_byte = buildLookupTable<float>();
			constexpr static trivial_array<double, 512> double_for_byte = buildLookupTable<double>();
		}

		
		inline float byte_to_float(int b) { return Internal::float_for_byte[b+255]; }
		inline double byte_to_double(int b) { return Internal::double_for_byte[b+255]; }
	
		//Convert a "D" to "To" scaled as if we are converting "From" type to a "To" type.
		//Special code is invoked if both D and To are integral.
		//FIXME: why is the test on "From", not "D"??
		template <class From, class To, class D=From, bool int1 = traits<To>::integral && traits<From>::integral, bool int2 =traits<D>::integral> struct ScalarConvert {
		    static inline To from(const D& from) {
			static const double factor = double(traits<To>::max_intensity)/traits<From>::max_intensity; 
			auto s = from * factor;
			return static_cast<To>(s);
		    }
		};
	    
		//If the input and output are integral, then use integer only scaling code.	
		template <class From, class To, class D> struct ScalarConvert<From,To,D,true, true> {
		    static inline To from(const D& f) {
			return shift_convert<To, From, int_info<To,From>::shift_dir>::from(f);
		    }
		};
		
		//If the destination is bool, then use != 0.
		//Note two classes are needed here so that they are both more specialized than
		//the integral conversion code (above) in order to avoid ambiguities.
		template<class From, class D> struct ScalarConvert<From, bool, D, true, true>
		{
			static inline bool from(const D& from)
			{
				return from != 0;
			}
		};
		template<class From, class D> struct ScalarConvert<From, bool, D, true, false>
		{
			static inline bool from(const D& from)
			{
				return from != 0;
			}
		};

		//Lookup table conversion from byte to float.
		//FIXME surely this can only work properly if D is also byte?
		template <class D> struct ScalarConvert<byte,float,D,false,true> {
		    static inline float from(const D& from) {
			return byte_to_float(from);
		    }
		};
		
		//FIXME this is surely redundant
		template <class D> struct ScalarConvert<byte,float,D,false,false> {
		    static inline float from(const D& from) {
			return static_cast<float>(from * (1.0/255.0));
		    }
		};

		template <class D> struct ScalarConvert<byte,double,D,false, true> {
		  static inline double from(const D& from) {
		    return byte_to_double(from);
		  }
		};
		template <class D> struct ScalarConvert<byte,double,D,false, false> {
		    static inline double from(const D& from) {
		      return from * (1.0/255.0);
		    }
		};
		
		inline double byte_float_to_float(double d) { 
		  return d * traits<double>::max_intensity/traits<byte>::max_intensity; 
		}

#if 0
		template <> struct ScalarConvert<byte,float,float,false> {
		  static inline float from(const float& from) {
		    return byte_float_to_float(from);
		  }
		};
		template <> struct ScalarConvert<byte,double,float,false> {
		  static inline double from(const float& from) {
		    return byte_float_to_float(from);
		  }
		};
		template <> struct ScalarConvert<byte,float,double,false> {
		  static inline float from(const double& from) {
		    return byte_float_to_float(from);
		  }
		};
		template <> struct ScalarConvert<byte,double,double,false> {
		  static inline double from(const double& from) {
		    return byte_float_to_float(from);
		  }
		};
#endif

	}
	

	template <class To, class From, class D> inline To scalar_convert(const D& d) { return Internal::ScalarConvert<From,To,D>::from(d); }
	//template <class To, class From> inline To scalar_convert(const From& d) { return Internal::ScalarConvert<From,To>::from(d); }
	
}
}
#endif
