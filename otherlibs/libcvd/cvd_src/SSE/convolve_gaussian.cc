#include <cvd/convolution.h>
#include <xmmintrin.h>

using namespace std;

namespace CVD {

#if defined(WIN32) && !defined(__MINGW32__)
inline __m128 operator+( const __m128 & a, const __m128 & b){
    return _mm_add_ps(a,b);
}

inline __m128 operator+=( __m128 & a, const __m128 & b){
    a = _mm_add_ps(a,b);
    return a;
}

inline __m128 operator-( const __m128 & a, const __m128 & b){
    return _mm_sub_ps(a,b);
}

inline __m128 operator*( const __m128 & a, const __m128 & b){
    return _mm_mul_ps(a,b);
}

#endif

inline void convolveMiddle5(const float* in, double factor, const double kernel[], int count, float* out)
{
    int i;
    const __m128 ffff = _mm_set1_ps(factor);
    const __m128 k1 = _mm_set1_ps(kernel[0]);
    const __m128 k2 = _mm_set1_ps(kernel[1]);
    for (i=0; i+3<count; i+=4, in += 4, out += 4) {
	__m128 sum = (_mm_mul_ps(_mm_loadu_ps(in), ffff) + 
		      _mm_mul_ps(k1, (_mm_loadu_ps(in-1) + _mm_loadu_ps(in+1))) + 
		      _mm_mul_ps(k2, (_mm_loadu_ps(in-2) + _mm_loadu_ps(in+2))));
	_mm_storeu_ps(out, sum);
    }

    for (; i<count; ++i, ++in, ++out) {
	double sum = in[0] * factor + kernel[0] * (in[-1] + in[1]) + kernel[1] * (in[-2] * in[2]);
	*out = sum;
    }
}

inline void convolveMiddle(const float* in, double factor, const vector<double>& kernel, int count, float* out)
{
    const int ksize = kernel.size();
    if (ksize == 2) {
	convolveMiddle5(in, factor, &kernel[0], count, out);
	return;
    }
    int i;
    const __m128 ffff = _mm_set1_ps(factor);
    for (i=0; i+3<count; i+=4, in += 4, out += 4) {
	__m128 sum = _mm_mul_ps(_mm_loadu_ps(in), ffff);
	for (int k=1; k<=ksize; ++k)
	    sum += _mm_set1_ps(kernel[k-1]) * (_mm_loadu_ps(in-k) + _mm_loadu_ps(in+k));
	_mm_storeu_ps(out, sum);
    }

    for (; i<count; ++i, ++in, ++out) {
	double sum = in[0] * factor;
	for (int k=1; k<=ksize; ++k)
	    sum += kernel[k-1] * (in[-k] + in[k]);
	*out = sum;
    }
}

template <bool Aligned> inline __m128 load_ps(const float* f) { return _mm_loadu_ps(f); }
template <> inline __m128 load_ps<true>(const float* f) { return _mm_load_ps(f); }

template <bool Aligned> inline void store_ps(float* f, __m128 d) { _mm_storeu_ps(f,d); }
template <> inline void store_ps<true>(float* f, __m128 d) { _mm_store_ps(f,d); }

template <bool Aligned>
inline void convolveVertical5(const vector<float*>& row, double factor, const double kernel[], int count, float* out)
{
    const int ksize = 2;
    int i;
    for (i=0; i<count && !is_aligned<16>(out); ++i, ++out) {
	double sum = row[ksize][i] * factor + (row[1][i] + row[3][i]) * kernel[0] + (row[0][i] + row[4][i]) * kernel[1];
	*out = sum;
    }

    const __m128 ffff = _mm_set1_ps(factor);
    const __m128 k1 = _mm_set1_ps(kernel[0]);
    const __m128 k2 = _mm_set1_ps(kernel[1]);

    for (; i+3<count; i+=4, out+=4) {
	__m128 sum = (load_ps<Aligned>(row[ksize] + i) * ffff
		      + (load_ps<Aligned>(row[ksize-1]+i) + load_ps<Aligned>(row[ksize+1]+i)) * k1
		      + (load_ps<Aligned>(row[ksize-2]+i) + load_ps<Aligned>(row[ksize+2]+i)) * k2);
	store_ps<true>(out, sum);
    }    
    for (; i<count; ++i, ++out) {
	double sum = row[ksize][i] * factor + (row[1][i] + row[3][i]) * kernel[0] + (row[0][i] + row[4][i]) * kernel[1];
	*out = sum;
    }
}

template <bool Aligned>
inline void convolveVertical(const vector<float*>& row, double factor, const vector<double>& kernel, int count, float* out)
{
    const int ksize = static_cast<int>(kernel.size());
    if (ksize == 2) {
	convolveVertical5<Aligned>(row, factor, &kernel[0], count, out);
	return;
    }
	
    int i;
    for (i=0; i<count && !is_aligned<16>(out); ++i, ++out) {
	double sum = row[ksize][i] * factor;
	for (int k=1; k<=ksize; ++k)
	    sum += kernel[k-1] * (row[ksize-k][i] + row[ksize+k][i]);
	*out = sum;
    }
    const __m128 ffff = _mm_set1_ps(factor);
    for (; i+3<count; i+=4, out+=4) {
	__m128 sum = _mm_mul_ps(load_ps<Aligned>(row[ksize] + i), ffff);
	for (int k=1; k<=ksize; ++k)
	    sum += _mm_set1_ps(kernel[k-1]) * (load_ps<Aligned>(row[ksize-k]+i) + load_ps<Aligned>(row[ksize+k]+i));
	store_ps<true>(out, sum);
    }    
    for (; i<count; ++i, ++out) {
	double sum = row[ksize][i] * factor;
	for (int k=1; k<=ksize; ++k)
	    sum += kernel[k-1] * (row[ksize-k][i] + row[ksize+k][i]);
	*out = sum;
    }
}

void convolveGaussian_simd(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas)
{
    assert(out.size() == I.size());
    int ksize = (int)ceil(sigmas*sigma);
    vector<double> kernel(ksize);
    double ksum = 1.0;
    for (int i=1; i<=ksize; i++)
	ksum += 2*(kernel[i-1] = exp(-i*i/(2*sigma*sigma)));
    double factor = 1.0/ksum;
    for (int i=0; i<ksize; i++)
	kernel[i] *= factor;
    const int w = I.size().x;
    const int h = I.size().y;
    const int swin = 2*ksize;

    vector<float> buffer(w*(swin+1));


    vector<float*> rows(swin+1);

    for (int k=0;k<swin+1;k++)
	rows[k] = buffer.data() + k*w;

    float* output = out.data();

    typedef void (*CONV_VERT_FUNC)(const vector<float*>&, double, const vector<double>&, int, float*);    
    const bool aligned = is_aligned<16>(I[0]) && is_aligned<16>(I[1]);
    CONV_VERT_FUNC conv_vert = aligned ? (CONV_VERT_FUNC)convolveVertical<true> : (CONV_VERT_FUNC)convolveVertical<false>;

    for (int i=0; i<h; i++) {
	float* next_row = rows[swin];
	const float* input = I[i];
	// beginning of row
	for (int j=0; j<min(ksize,w); j++) {
	    double hsum = input[j] * factor;
	    for (int k=0; k<ksize; k++)
		hsum += (input[std::max(j-k-1,0)] + input[std::min(j+k+1,w-1)]) * kernel[k];
	    next_row[j] = hsum;
	}
	// middle of row
	input += ksize;
	convolveMiddle(input, factor, kernel, w-swin, next_row+ksize);
	input += w-swin;
	// end of row
	if(w > ksize)
	    for (int j=w-ksize; j<w; j++, input++) {
		double hsum = *input * factor;
		const int room = w-j;
		for (int k=0; k<ksize; k++) {
		    hsum += (input[-k-1] + input[std::min(k+1,room-1)]) * kernel[k];
		}
		next_row[j] = hsum;
	    }
	// vertical
	if (i >= swin) {
	    conv_vert(rows, factor, kernel, w, output);
	    output += w;
	    if (i == h-1) {
		for (int r=0; r<ksize; r++, output += w) {
		    vector<float*> rrows(rows.size());
		    rrows[ksize] = rows[ksize+r+1];
		    for (int k=0; k<ksize; ++k) {
			rrows[ksize-k-1] = rows[ksize+r-k];
			int hi = ksize+r+k+2;
			rrows[ksize+k+1] = rows[std::min(hi,swin)];
		    }
		    conv_vert(rrows, factor, kernel, w, output);
		}
	    }
	} else if (i == swin-1) {
	    for (int r=0; r<ksize; r++, output += w) {
		vector<float*> rrows(rows.size());
		rrows[ksize] = rows[r+1];
		for (int k=0; k<ksize; ++k) {
		    rrows[ksize-k-1] = rows[std::max(r-k-1,0)+1];
		    rrows[ksize+k+1] = rows[r+k+2];
		}
		conv_vert(rrows, factor, kernel, w, output);
	    }
	}
    
	float* tmp = rows[0];
	for (int r=0;r<swin; r++)
	    rows[r] = rows[r+1];
	rows[swin] = tmp;
    }
}

inline void transpose(__m128& x1, __m128& x2, __m128& x3, __m128& x4)
{
    // abcd  x1
    // efgh  x2
    // ijkl  x3
    // mnop  x4
    __m128 aebf = _mm_unpacklo_ps(x1,x2);
    __m128 imjn = _mm_unpacklo_ps(x3,x4); 
    __m128 cgdh = _mm_unpackhi_ps(x1,x2);
    __m128 kolp = _mm_unpackhi_ps(x3,x4);
    x1 = _mm_shuffle_ps(aebf, imjn, _MM_SHUFFLE(1,0,1,0));  //aeim
    x2 = _mm_shuffle_ps(aebf, imjn, _MM_SHUFFLE(3,2,3,2));  //bfjn
    x3 = _mm_shuffle_ps(cgdh, kolp, _MM_SHUFFLE(1,0,1,0));  //cgko
    x4 = _mm_shuffle_ps(cgdh, kolp, _MM_SHUFFLE(3,2,3,2));  //dhlp
}

inline void forward_to_backward(const __m128 M[], const __m128 i_plus, const __m128 inv_alpha, __m128& y1, __m128& y2, __m128& y3)
{
    __m128 u_plus = inv_alpha * i_plus;
    __m128 v_plus = inv_alpha * u_plus;
    __m128 x1=y1-u_plus, x2=y2-u_plus, x3=y3-u_plus;
    y1 = M[0]*x1 + M[1]*x2 + M[2]*x3 + v_plus;
    y2 = M[3]*x1 + M[4]*x2 + M[5]*x3 + v_plus;
    y3 = M[6]*x1 + M[7]*x2 + M[8]*x3 + v_plus;
}


// Optimized implementation of Young-van Vliet third-order recursive Gaussian filter.
// See "Recursive Gaussian Derivative Filters", by van Vliet, Young and Verbeck, 1998
// and "Boundary Conditions for Young - van Vliet Recursive Filtering", by Triggs and Sdika, 2005
// This can produce output with values slightly outside the input range.
void van_vliet_blur_simd(const double b[], const BasicImage<float> in, BasicImage<float> out)
{
    assert(in.size() == out.size());
    const int w = in.size().x;
    const int h = in.size().y;
    const int is = in.row_stride();
    const int os = out.row_stride();
    assert(is_aligned<16>(in[0]) && is_aligned<16>(out[0]));
    assert((w%4)==0 && (h%4)==0 && (is%4) == 0 && (os%4) == 0);

    unsigned int csr_state = _mm_getcsr();
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

    vector<__m128> tmpArray(w>h?w:h);
    __m128 * tmp = tmpArray.data();

#ifdef WIN32
    __m128 M[9];
#else
    __m128 M[9] __attribute__((aligned(16)));
#endif

    {	
	double m[3][3];
	compute_triggs_M(b, m);
	for (int i=0; i<3; ++i)
	    for (int j=0; j<3; ++j)
		M[i*3+j] = _mm_set1_ps(m[i][j]);
    }

    const double alpha = 1 + b[0] + b[1] + b[2];
    const __m128 a2 = _mm_set1_ps(alpha*alpha);

    
    const __m128 inv_alpha = _mm_set1_ps(1.0/alpha);
    const __m128 bb1 = _mm_set1_ps(b[0]);
    const __m128 bb2 = _mm_set1_ps(b[1]);
    const __m128 bb3 = _mm_set1_ps(b[2]);
    
    // horizontal
    for (int i=0; i<h; i+=4) {
	const float* p = in[i];

	__m128 y3, y2, y1;
	y3 = y2 = y1 = inv_alpha*_mm_setr_ps(p[0], p[is], p[2*is], p[3*is]);
	    
	for (int j=0; j+3<w; j+=4, p+=4) {
	    __m128 y0 = _mm_setr_ps(p[0], p[is], p[2*is], p[3*is]) - (bb1*y1 + bb2*y2 + bb3*y3);
	    y3 = _mm_setr_ps(p[1], p[is+1], p[2*is+1], p[3*is+1]) - (bb1*y0 + bb2*y1 + bb3*y2);
	    y2 = _mm_setr_ps(p[2], p[is+2], p[2*is+2], p[3*is+2]) - (bb1*y3 + bb2*y0 + bb3*y1);
	    y1 = _mm_setr_ps(p[3], p[is+3], p[2*is+3], p[3*is+3]) - (bb1*y2 + bb2*y3 + bb3*y0);
	    tmp[j] = y0;
	    tmp[j+1] = y3;
	    tmp[j+2] = y2;
	    tmp[j+3] = y1;
	}
	
	{
	    const __m128 i_plus = _mm_setr_ps(p[-1], p[is-1], p[2*is-1], p[3*is-1]);
	    __m128 y0 = i_plus - (bb1*y1 + bb2*y2 + bb3*y3);
	    y3 = y2; y2 = y1; y1 = y0;
	    forward_to_backward(M, i_plus, inv_alpha, y1, y2, y3);
	    //y3 = y2 = y1 = tmp[w-1]*inv_alpha;
	}

	float* o = out[i]+w-4;	    
	for (int j=w-1; j>=0; j-=4, o-=4) {
	    __m128 y00 = tmp[j] - (bb1*y1 + bb2*y2 + bb3*y3);
	    __m128 y01 = y3 = tmp[j-1] - (bb1*y00 + bb2*y1 + bb3*y2);
	    __m128 y02 = y2 = tmp[j-2] - (bb1*y01 + bb2*y00 + bb3*y1);
	    __m128 y03 = y1 = tmp[j-3] - (bb1*y02 + bb2*y01 + bb3*y00);
	    transpose(y03,y02,y01,y00);
	    _mm_store_ps(o, a2*y03);
	    _mm_store_ps(o+os, a2*y02);
	    _mm_store_ps(o+2*os, a2*y01);
	    _mm_store_ps(o+3*os, a2*y00);
	}
    }

    // vertical
    for (int i=0; i<w; i+=4) {
	const float* in = out[0] + i;
	__m128 y3, y2, y1;
	y3 = y2 = y1 = inv_alpha*_mm_load_ps(in);
	
	for (int j=0; j<h; j+=4, in += os) {
	    __m128 y0 = _mm_load_ps(in) - (bb1*y1 + bb2*y2 + bb3*y3);
	    tmp[j] = y0;
	    in += os;
	    tmp[j+1] = y3 = _mm_load_ps(in) - (bb1*y0 + bb2*y1 + bb3*y2);
	    in += os;
	    tmp[j+2] = y2 = _mm_load_ps(in) - (bb1*y3 + bb2*y0 + bb3*y1);
	    in += os;
	    tmp[j+3] = y1 = _mm_load_ps(in) - (bb1*y2 + bb2*y3 + bb3*y0);
	}
	
	{
	    const __m128 i_plus = _mm_load_ps(in - os);
	    __m128 y0 = i_plus - (bb1*y1 + bb2*y2 + bb3*y3);
	    y3 = y2; y2 = y1; y1 = y0;
	    forward_to_backward(M, i_plus, inv_alpha, y1, y2, y3);
	    //y3 = y2 = y1 = tmp[h-1]*inv_alpha;
	}


	float* o = out[h-1] + i;
	//const __m128 zeros = _mm_setzero_ps();
	//const __m128 ones = _mm_set1_ps(1.0);

	for (int j=h-1; j>=0; j-=4, o-=os) {
	    __m128 y0 = tmp[j] - (bb1*y1 + bb2*y2 + bb3*y3);	    
	    y3 = tmp[j-1] - (bb1*y0 + bb2*y1 + bb3*y2);
	    y2 = tmp[j-2] - (bb1*y3 + bb2*y0 + bb3*y1);
	    y1 = tmp[j-3] - (bb1*y2 + bb2*y3 + bb3*y0);

	    _mm_store_ps(o, a2*y0);//_mm_min_ps(ones, _mm_max_ps(zeros,a4*y0)));
	    o -= os;
	    _mm_store_ps(o, a2*y3);//_mm_min_ps(ones, _mm_max_ps(zeros,a4*y3)));
	    o -= os;
	    _mm_store_ps(o, a2*y2);//_mm_min_ps(ones, _mm_max_ps(zeros,a4*y2)));
	    o -= os;
	    _mm_store_ps(o, a2*y1);//_mm_min_ps(ones, _mm_max_ps(zeros,a4*y1)));
	}
    }
    _mm_setcsr(csr_state);
}

// Try to choose the fastest method
void convolveGaussian(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas)
{
    int ksize = (int)ceil(sigma*sigmas);
    bool nice = ((I.size().x%4) == 0 && 
		 (I.size().y%4)==0 && 
		 (I.row_stride()%4) == 0 &&
		 (out.row_stride()%4) == 0 &&
		 is_aligned<16>(I[0]) && 
		 is_aligned<16>(out[0]));
    if (nice && ksize > 2) {
	double b[3];
	compute_van_vliet_b(sigma, b);
	van_vliet_blur_simd(b, I, out);
    } else if (ksize > 6) {
	double b[3];
	compute_van_vliet_b(sigma, b);
	van_vliet_blur(b, I, out);
    } else
	convolveGaussian_simd(I, out, sigma, sigmas);    
}

void convolveGaussian_fir(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas)
{
    convolveGaussian_simd(I, out, sigma, sigmas);    
}


};

