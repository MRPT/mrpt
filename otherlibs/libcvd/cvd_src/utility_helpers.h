#ifndef CVD_INCLUDE_UTILITY_HELPERS_H
#define CVD_INCLUDE_UTILITY_HELPERS_H

#include <cvd/utility.h>
#include <xmmintrin.h>
using namespace std;

namespace CVD{
    template <class F, class T1, class T2, int A, int M> inline void maybe_aligned_differences(const T1* a, const T1* b, T2* c, size_t count)
    {
	if (count < M*2) {
	    F::unaligned_differences(a,b,c,count);
	    return;
	}
	if (!is_aligned<A>(a)) {	    
	    size_t steps = steps_to_align<A>(a);
	    F::unaligned_differences(a,b,c,steps);
	    count -= steps;
	    a += steps;
	    b += steps;
	    c += steps;
	}
	if (!is_aligned<A>(c) || count < M) {
	    F::unaligned_differences(a,b,c,count);
	    return;
	}	
	size_t block = (count/M)*M;
	F::aligned_differences(a,b,c,block);
	if (count > block) {
	    F::unaligned_differences(a+block,b+block,c+block,count-block);
	}
    }    
    
    template <class F, class T1, class T2, int A, int M> inline void maybe_aligned_add_mul_add(const T1* a, const T1* b, const T1& c, T2* out, size_t count)
    {
	if (count < M*2) {
	    F::unaligned_add_mul_add(a,b,c,out,count);
	    return;
	}
	if (!is_aligned<A>(a)) {      
	    size_t steps = steps_to_align<A>(a);
	    F::unaligned_add_mul_add(a,b,c,out,steps);
	    count -= steps;
	    a += steps;
	    b += steps;
	    out += steps;
	    if (count < M || !is_aligned<16>(out)) {
		F::unaligned_add_mul_add(a,b,c,out,count);
		return;
	    }
	}
	else if (count < M || !is_aligned<16>(out)) {
	    F::unaligned_add_mul_add(a,b,c,out,count);
	    return;
	}
	size_t block = (count/M)*M;
	F::aligned_add_mul_add(a,b,c,out,block);
	if (count > block)
	    F::unaligned_add_mul_add(a+block,b+block,c, out+block,count-block);
    }    

    template <class F, class T1, class T2, int A, int M> inline void maybe_aligned_assign_mul(const T1* a, const T1& c, T2* out, size_t count)
    {
	if (count < M*2) {
	    F::unaligned_assign_mul(a,c,out,count);
	    return;
	}
	if (!is_aligned<A>(a)) {      
	    size_t steps = steps_to_align<A>(a);
	    F::unaligned_assign_mul(a,c,out,steps);
	    count -= steps;
	    a += steps;
	    out += steps;
	    if (count < M) {
		F::unaligned_assign_mul(a,c,out,count);
		return;
	    }
	}
	size_t block = (count/M)*M;
	F::aligned_assign_mul(a,c,out,block);
	if (count > block) {
	    F::unaligned_assign_mul(a+block,c, out+block,count-block);
	}
    }    

    template <class F, class R, class T1, int A, int M> inline R maybe_aligned_inner_product(const T1* a, const T1* b, size_t count)
    {
	if (count < M*2) {
	    return F::unaligned_inner_product(a,b,count);
	}
	R sum = 0;
	if (!is_aligned<A>(a)) {      
	    size_t steps = steps_to_align<A>(a);
	    sum = F::unaligned_inner_product(a,b,steps);
	    count -= steps;
	    a += steps;
	    b += steps;
	    if (count < M) {
		return sum + F::unaligned_inner_product(a,b,count);
	    }
	}
	size_t block = (count/M)*M;
	sum += F::aligned_inner_product(a,b,block);
	if (count > block)
	    sum += F::unaligned_inner_product(a+block,b+block,count-block);
	return sum;
    }    

    template <class F, class R, class T1, int A, int M> inline R maybe_aligned_ssd(const T1* a, const T1* b, size_t count)
    {
	if (count < M*2) {
	    return F::unaligned_ssd(a,b,count);
	}
	R sum = 0;
	if (!is_aligned<A>(a)) {      
	    size_t steps = steps_to_align<A>(a);
	    sum = F::unaligned_ssd(a,b,steps);
	    count -= steps;
	    a += steps;
	    b += steps;
	    if (count < M) {
		return sum + F::unaligned_ssd(a,b,count);
	    }
	}
	size_t block = (count/M)*M;
	sum += F::aligned_ssd(a,b,block);
	if (count > block)
	    sum += F::unaligned_ssd(a+block,b+block,count-block);
	return sum;
    }    

template <class F, class T1, class T2, int A, int M> inline void maybe_aligned_square(const T1* in, T2* out, size_t count)
{
    if (count < M*2) {
	return F::unaligned_square(in,out,count);
    }
    if (!is_aligned<A>(in)) {
	size_t steps = steps_to_align<A>(in);
	F::unaligned_square(in,out,steps);
	count -= steps;
	in += steps;
	out += steps;
	if (count < M) {
	    F::unaligned_square(in,out,count);
	}
    }
    size_t block = (count/M)*M;
    F::aligned_square(in,out,block);
    if (count > block)
	F::unaligned_square(in+block,out+block,count-block);
}    

template <class F, class T1, class T2, int A, int M> inline void maybe_aligned_subtract_square(const T1* in, T2* out, size_t count)
{
    if (count < M*2) {
	return F::unaligned_subtract_square(in,out,count);
    }
    if (!is_aligned<A>(in)) {
	size_t steps = steps_to_align<A>(in);
	F::unaligned_subtract_square(in,out,steps);
	count -= steps;
	in += steps;
	out += steps;
	if (count < M) {
	    F::unaligned_subtract_square(in,out,count);
	}
    }
    size_t block = (count/M)*M;
    F::aligned_subtract_square(in,out,block);
    if (count > block)
	F::unaligned_subtract_square(in+block,out+block,count-block);
}    



}

#endif
