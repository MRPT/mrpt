#include "cvd/utility.h"
#include "cvd_src/utility_helpers.h"
#include <emmintrin.h>

#include <algorithm>

namespace CVD {

#ifndef WIN32  // this is not used at all?
    static inline __m128i zero_si128() { __m128i x; asm ( "pxor %0, %0  \n\t" : "=x"(x) ); return x; }
#endif

    template <bool Aligned> inline __m128i load_si128(const void* addr) { return _mm_loadu_si128((const __m128i*)addr); }
    template <> inline __m128i load_si128<true>(const void* addr) { return _mm_load_si128((const __m128i*)addr); }
    template <bool Aligned> inline __m128d load_pd(const void* addr) { return _mm_loadu_pd((const double*)addr); }
    template <> inline __m128d load_pd<true>(const void* addr) { return _mm_load_pd((const double*)addr); }

    template <bool Aligned> inline void store_pd(__m128d m, void* addr) { return _mm_storeu_pd((double*)addr, m); }
    template <> inline void store_pd<true>(__m128d m, void* addr) { return _mm_store_pd((double*)addr, m); }

    template <bool Aligned_b> void int_differences(const __m128i* a, const __m128i* b, __m128i* diff, size_t count)
    {
	while (count--) {
	    *(diff++) = _mm_sub_epi32(*(a++), load_si128<Aligned_b>(b++));
	}
    }
    
    template <bool Aligned_b> void double_differences(const __m128d* a, const __m128d* b, __m128d* diff, size_t count)
    {
	while (count--) {
	    *(diff++) = _mm_sub_pd(*(a++), load_pd<Aligned_b>(b++));
	}
    }

    template <bool Aligned_b> void double_add_multiple_of_sum(const __m128d* a, const __m128d* b, const double& c, __m128d* out, size_t count)
    {
	__m128d cc = _mm_set1_pd(c);
	while (count--) {
	    *out = _mm_add_pd(_mm_mul_pd(_mm_add_pd(*(a++), load_pd<Aligned_b>(b++)), cc), *out);
	    ++out;
	}
    }

    template <bool Aligned_out> void double_assign_multiple(const __m128d* a, const double& c, __m128d* out, size_t count)
    {
	__m128d cc = _mm_set1_pd(c);
	while (count--)
	    store_pd<Aligned_out>(_mm_mul_pd(*(a++), cc), out++);
    }
    template <bool Aligned_b> double double_inner_product(const __m128d* a, const __m128d* b, size_t count)
    {
	double sums_store[2];
	const size_t BLOCK = 1<<16;
	double dot = 0;
	while (count) {
	    size_t pass = std::min(count, BLOCK);
	    count-=pass;
	    __m128d sums = _mm_setzero_pd();
	    while (pass--) {
		__m128d prod = _mm_mul_pd(*(a++), load_pd<Aligned_b>(b++));
		sums = _mm_add_pd(prod, sums);
	    }
	    _mm_storeu_pd(sums_store, sums);
	    dot += sums_store[0] + sums_store[1];
	}
	return dot;
    }

    template <bool Aligned_b> long long byte_sum_squared_differences(const __m128i* a, const __m128i* b, size_t count) {
	unsigned long sums_store[4] = {0};	
	const size_t BLOCK = 1<<15;
	long long ssd = 0;
	while (count) {
	    size_t pass = std::min(count, BLOCK);
	    count -= pass;
	    __m128i sums = _mm_setzero_si128();
	    while (pass--) {
		__m128i lo_a = load_si128<true>(a++);
		__m128i lo_b = load_si128<Aligned_b>(b++);
		__m128i hi_a = _mm_unpackhi_epi8(lo_a, sums);
		__m128i hi_b = _mm_unpackhi_epi8(lo_b, sums);
		lo_a = _mm_unpacklo_epi8(lo_a, sums);
		lo_b = _mm_unpacklo_epi8(lo_b, sums);
		lo_a = _mm_sub_epi16(lo_a, lo_b);
		hi_a = _mm_sub_epi16(hi_a, hi_b);
		lo_a = _mm_madd_epi16(lo_a,lo_a);
		hi_a = _mm_madd_epi16(hi_a,hi_a);
		sums = _mm_add_epi32(_mm_add_epi32(lo_a, hi_a), sums);
	    }
	    _mm_storeu_si128((__m128i*)sums_store, sums);
	    ssd += sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
	}
	return ssd;
    }

    template <bool Aligned_b> inline double double_sum_squared_differences(const __m128d* a, const __m128d* b, size_t count) 
    {
	double sums_store[2];
	const size_t BLOCK = 1<<10;
	double ssd = 0;
	while (count) {
	    size_t pass = std::min(count, BLOCK);
	    count-=pass;
	    __m128d sums = _mm_setzero_pd();
	    while (pass--) {
		__m128d diff = _mm_sub_pd(*(a++), load_pd<Aligned_b>(b++));
		sums = _mm_add_pd(_mm_mul_pd(diff,diff), sums);
	    }
	    _mm_storeu_pd(sums_store, sums);
	    ssd += sums_store[0] + sums_store[1];
	}
	return ssd;
    }

    
    struct SSE2_funcs {
	template <class T1, class T2> static inline void unaligned_differences(const T1* a, const T1* b, T2* diff, size_t count) {
	    differences<T1,T2>(a,b,diff,count);
	}

	static inline void aligned_differences(const int32_t* a, const int32_t* b, int32_t* diff, size_t count) {
	    if (is_aligned<16>(b))
		int_differences<true>((const __m128i*)a, (const __m128i*)b, (__m128i*)diff, count>>2);
	    else
		int_differences<false>((const __m128i*)a, (const __m128i*)b, (__m128i*)diff, count>>2);
	}

	static inline void aligned_differences(const double* a, const double* b, double* diff, size_t count)
	{
	    if (is_aligned<16>(b))
		double_differences<true>((const __m128d*)a,(const __m128d*)b,(__m128d*)diff,count>>1);
	    else
		double_differences<false>((const __m128d*)a,(const __m128d*)b,(__m128d*)diff,count>>1);
	}

	template <class T1, class T2> static inline void unaligned_add_mul_add(const T1* a, const T1* b, const T1& c, T2* out, size_t count) {
	    add_multiple_of_sum<T1,T2>(a,b,c,out,count);
	}
	
	static inline void aligned_add_mul_add(const double* a, const double* b, const double& c, double* out, size_t count)
	{
	    if (is_aligned<16>(b))
		double_add_multiple_of_sum<true>((const __m128d*)a, (const __m128d*)b, c, (__m128d*)out, count>>1);
	    else
		double_add_multiple_of_sum<false>((const __m128d*)a, (const __m128d*)b, c, (__m128d*)out, count>>1);
	}
	
	template <class T1, class T2> static inline void unaligned_assign_mul(const T1* a, const T1& c, T2* out, size_t count) {
	    assign_multiple<T1,T2>(a,c,out,count);
	}

	static inline void aligned_assign_mul(const double* a, const double& c, double* out, size_t count)
	{
	    if (is_aligned<16>(out))
		double_assign_multiple<true>((const __m128d*)a, c, (__m128d*)out, count>>1);
	    else
		double_assign_multiple<false>((const __m128d*)a, c, (__m128d*)out, count>>1);
	}
	
	template <class T1> static inline double unaligned_inner_product(const T1* a, const T1* b, size_t count) {
	    return inner_product<T1>(a,b,count);
	}
	
	static inline double aligned_inner_product(const double* a, const double* b, size_t count)
	{
	    if (is_aligned<16>(b))
		return double_inner_product<true>((const __m128d*) a, (const __m128d*) b, count>>1);
	    else
		return double_inner_product<false>((const __m128d*) a, (const __m128d*) b, count>>1);
	}

	template <class T1> static inline double unaligned_ssd(const T1* a, const T1* b, size_t count) {
	    return sum_squared_differences<T1>(a,b,count);
	}
	
	static inline long long unaligned_ssd(const byte* a, const byte* b, size_t count) {
	    return SumSquaredDifferences<long long, int, byte>::sum_squared_differences(a,b,count);
	}

	static inline double aligned_ssd(const double* a, const double* b, size_t count) 
	{
	    if (is_aligned<16>(b))
		return double_sum_squared_differences<true>((const __m128d*)a, (const __m128d*)b, count>>1);
	    else
		return double_sum_squared_differences<false>((const __m128d*)a, (const __m128d*)b, count>>1);
	}

	static inline long long aligned_ssd(const byte* a, const byte* b, size_t count) 
	{
	    if (is_aligned<16>(b)) 
		return byte_sum_squared_differences<true>((const __m128i*)a, (const __m128i*)b, count>>4);
	    else
		return byte_sum_squared_differences<false>((const __m128i*)a, (const __m128i*)b, count>>4);
	}	
    };

    void differences(const int32_t* a, const int32_t* b, int32_t* diff, size_t size)
    {
	maybe_aligned_differences<SSE2_funcs, int32_t, int32_t, 16, 4>(a,b,diff,size);
    }

    void differences(const double* a, const double* b, double* diff, size_t size)
    {
	maybe_aligned_differences<SSE2_funcs, double, double, 16, 2>(a,b,diff,size);
    }

    void add_multiple_of_sum(const double* a, const double* b, const double& c,  double* out, size_t count)
    {
		maybe_aligned_add_mul_add<SSE2_funcs, double, double, 16, 2>(a,b,c,out,count);
    }

    void assign_multiple(const double* a, const double& c,  double* out, size_t count)
    {
	maybe_aligned_assign_mul<SSE2_funcs, double, double, 16, 2>(a,c,out,count);
    }

    double inner_product(const double* a, const double* b, size_t count)
    {
	return maybe_aligned_inner_product<SSE2_funcs, double, double, 16, 2>(a,b,count);
    }

    double sum_squared_differences(const double* a, const double* b, size_t count)
    {
	return maybe_aligned_ssd<SSE2_funcs, double, double, 16, 2>(a,b,count);
    }

    long long sum_squared_differences(const byte* a, const byte* b, size_t count)
    {
	return maybe_aligned_ssd<SSE2_funcs, long long, byte, 16, 16>(a,b,count); 
    }
    
}
