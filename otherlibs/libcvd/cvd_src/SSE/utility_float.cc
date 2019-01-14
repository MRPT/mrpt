#include "cvd/utility.h"
#include "cvd_src/utility_helpers.h"


#include <xmmintrin.h>


using namespace std;

namespace CVD {

	//Some versions of gcc do not define _mm_load?_ps as taking a const float*
	//Later versions always use the const.
    template <bool Aligned> inline __m128 load_ps(const void* addr) { return _mm_loadu_ps(static_cast<float*>(const_cast<void*>(addr))); }
    template <> inline __m128 load_ps<true>(const void* addr) { return _mm_load_ps(static_cast<float*>(const_cast<void*>(addr))); }

    template <bool Aligned> inline void store_ps(__m128 m, void* addr) { return _mm_storeu_ps((float*)addr, m); }
    template <> inline void store_ps<true>(__m128 m, void* addr) { return _mm_store_ps((float*)addr, m); }

    template <bool Aligned_b> void float_differences(const __m128* a, const __m128* b, __m128* diff, size_t count)
    {
	while (count--) {
	    _mm_stream_ps((float*)diff, _mm_sub_ps(load_ps<true>(a), load_ps<Aligned_b>(b)));
	    ++diff;
	    ++a;
	    ++b;
	}
    }
    
    template <bool Aligned_b> void float_add_multiple_of_sum(const __m128* a, const __m128* b, const float& c, __m128* out, size_t count)
    {
	__m128 cccc = _mm_set1_ps(c);
	while (count--) {
	    *out = _mm_add_ps(_mm_mul_ps(_mm_add_ps(load_ps<true>(a), load_ps<Aligned_b>(b)), cccc), *out);
	    ++out;
	    ++a;
	    ++b;
	}
    }

    template <bool Aligned_out> inline void float_assign_multiple(const __m128* a, const float& c, __m128* out, size_t count)
    {
	const __m128 cccc = _mm_set1_ps(c);
	while (count--)
	    store_ps<Aligned_out>(_mm_mul_ps(*(a++), cccc), out++);
       
    }
    
    template <bool Aligned_b> double float_inner_product(const __m128* a, const __m128* b, size_t count)
    {
	float sums_store[4];
	const size_t BLOCK = 1<<10;
	double dot = 0;
	while (count) {
	    size_t pass = std::min(count, BLOCK);
	    count-=pass;
	    __m128 sums = _mm_setzero_ps();
	    while (pass--) {
		__m128 prod = _mm_mul_ps(*(a++), load_ps<Aligned_b>(b++));
		sums = _mm_add_ps(prod, sums);
	    }
	    _mm_storeu_ps(sums_store, sums);
	    dot += sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
	}
	return dot;
    }

    template <bool Aligned_b> inline double float_sum_squared_differences(const __m128* a, const __m128* b, size_t count) 
    {
	float sums_store[4];
	const size_t BLOCK = 1<<10;
	double ssd = 0;
	while (count) {
	    size_t pass = std::min(count, BLOCK);
	    count-=pass;
	    __m128 sums = _mm_setzero_ps();
	    while (pass--) {
		__m128 diff = _mm_sub_ps(*(a++), load_ps<Aligned_b>(b++));
		sums = _mm_add_ps(_mm_mul_ps(diff,diff), sums);
	    }
	    _mm_storeu_ps(sums_store, sums);
	    ssd += sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
	}
	return ssd;
    }
    
    template <bool Aligned_out> void float_square(const __m128* in, __m128* out, size_t count) {
	while (count--) {
	    __m128 x = load_ps<true>(in);
	    store_ps<Aligned_out>(_mm_mul_ps(x, x), out);
	    ++in;
	    ++out;
	}
    }

    template <bool Aligned_out> void float_subtract_square(const __m128* in, __m128* out, size_t count) {
	while (count--) {
	    __m128 x = load_ps<true>(in);
	    __m128 y = load_ps<Aligned_out>(out);
	    store_ps<Aligned_out>(_mm_sub_ps(y, _mm_mul_ps(x, x)), out);
	    ++in;
	    ++out;
	}
    }

    struct SSE_funcs {
	template <class T1, class T2> static inline void unaligned_differences(const T1* a, const T1* b, T2* diff, size_t count) {
	    differences<T1,T2>(a,b,diff,count);
	}
	
	static inline void aligned_differences(const float* a, const float* b, float* diff, size_t count) {
	    if (is_aligned<16>(b))
		float_differences<true>((const __m128*)a, (const __m128*)b, (__m128*)diff, count>>2);
	    else
		float_differences<false>((const __m128*)a, (const __m128*)b, (__m128*)diff, count>>2);
	}

	template <class T1, class T2> static inline void unaligned_add_mul_add(const T1* a, const T1* b, const T1& c, T2* out, size_t count) {
	    add_multiple_of_sum<T1,T2>(a,b,c,out,count);
	}
	static inline void aligned_add_mul_add(const float* a, const float* b, const float& c, float* out, size_t count) {
	    if (is_aligned<16>(b))
		float_add_multiple_of_sum<true>((const __m128*)a, (const __m128*)b, c, (__m128*)out, count>>2);
	    else
		float_add_multiple_of_sum<false>((const __m128*)a, (const __m128*)b, c, (__m128*)out, count>>2);
	}	

	template <class T1, class T2> static inline void unaligned_assign_mul(const T1* a, const T1& c, T2* out, size_t count) {
	    assign_multiple<T1,T2>(a,c,out,count);
	}
	static inline void aligned_assign_mul(const float* a, const float& c, float* out, size_t count) {
	    if (is_aligned<16>(out)) 
		float_assign_multiple<false>((const __m128*)a, c, (__m128*)out, count>>2);
	    else		
		float_assign_multiple<false>((const __m128*)a, c, (__m128*)out, count>>2);
	}	

	template <class T1> static inline double unaligned_inner_product(const T1* a, const T1* b, size_t count) {
	    return inner_product<T1>(a,b,count);
	}
	
	static inline double aligned_inner_product(const float* a, const float* b, size_t count)
	{
	    if (is_aligned<16>(b))
		return float_inner_product<true>((const __m128*) a, (const __m128*) b, count>>2);
	    else
		return float_inner_product<false>((const __m128*) a, (const __m128*) b, count>>2);
	}	

	template <class T1> static inline double unaligned_ssd(const T1* a, const T1* b, size_t count) {
	    return sum_squared_differences<T1>(a,b,count);
	}
	
	static inline double aligned_ssd(const float* a, const float* b, size_t count)
	{
	    if (is_aligned<16>(b))
		return float_sum_squared_differences<true>((const __m128*) a, (const __m128*) b, count>>2);
	    else
		return float_sum_squared_differences<false>((const __m128*) a, (const __m128*) b, count>>2);
	}	
	
	template <class T1, class T2> static inline void unaligned_square(const T1* in, T2* out, size_t count) {
	    square<T1,T2>(in, out, count);
	}

	static inline void aligned_square(const float* in, float* out, size_t count) {
	    if (is_aligned<16>(out))
		float_square<true>((const __m128*)in, (__m128*)out, count >> 2);
	    else
		float_square<false>((const __m128*)in, (__m128*)out, count >> 2);		
	}
	template <class T1, class T2> static inline void unaligned_subtract_square(const T1* in, T2* out, size_t count) {
	    subtract_square<T1,T2>(in, out, count);
	}

	static inline void aligned_subtract_square(const float* in, float* out, size_t count) {
	    if (is_aligned<16>(out))
		float_subtract_square<true>((const __m128*)in, (__m128*)out, count >> 2);
	    else
		float_subtract_square<false>((const __m128*)in, (__m128*)out, count >> 2);		
	}
    };
    
    void differences(const float* a, const float* b, float* diff, size_t size)
    {
	maybe_aligned_differences<SSE_funcs, float, float, 16, 4>(a,b,diff,size);
    }
    
    void add_multiple_of_sum(const float* a, const float* b, const float& c,  float* out, size_t count)
    {
	maybe_aligned_add_mul_add<SSE_funcs,float,float,16,4>(a,b,c,out,count);
    }
    
    void assign_multiple(const float* a, const float& c,  float* out, size_t count) 
    {
	maybe_aligned_assign_mul<SSE_funcs,float,float,16,4>(a,c,out,count);
    }

    
    double inner_product(const float* a, const float* b, size_t count) 
    {
	return maybe_aligned_inner_product<SSE_funcs,double,float,16,4>(a,b,count);
    }

    double sum_squared_differences(const float* a, const float* b, size_t count)
    {
	return maybe_aligned_ssd<SSE_funcs,double,float,16,4>(a,b,count);
    }
    
    void square(const float* in, float* out, size_t count) 
    {
	maybe_aligned_square<SSE_funcs,float,float,16,4>(in, out, count);
    }

    void subtract_square(const float* in, float* out, size_t count) 
    {
	maybe_aligned_subtract_square<SSE_funcs,float,float,16,4>(in, out, count);
    }
    
}
