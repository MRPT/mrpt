
#ifndef CVD_CONVOLUTION_H_
#define CVD_CONVOLUTION_H_

#include <vector>
#include <memory>
#include <numeric>
#include <algorithm>

#include <cvd/config.h>
#include <cvd/exceptions.h>
#include <cvd/image.h>
#include <cvd/internal/pixel_operations.h>
#include <cvd/utility.h>

namespace CVD {

/// creates a Gaussian kernel with given maximum value and standard deviation.
/// All elements of the passed vector are filled up, therefore the vector
/// defines the size of the computed kernel. The normalizing value is returned.
/// @param k vector of T's holds the kernel values
/// @param maxval the maximum value to be used
/// @param stddev standard deviation of the kernel
/// @return the sum of the kernel elements for normalization
/// @ingroup gVision
template <class T>
T gaussianKernel(std::vector<T>& k, T maxval, double stddev)
{
    double sum = 0;
    unsigned int i, argmax=0;
    std::vector<double> kernel(k.size());
    for (i=0;i<k.size();i++) {
        double x = i +0.5 - k.size()/2.0;
        sum += kernel[i] = exp(-x*x/(2*stddev*stddev));
        if (kernel[i] > kernel[argmax])
        argmax = i;
    }
    T finalSum = 0;
    for (i=0;i<k.size();i++)
    finalSum += k[i] = (T)(kernel[i]*maxval/kernel[argmax]);
    return finalSum;
}

/// scales a GaussianKernel to a different maximum value. The new kernel is
/// returned in scaled. The new normalizing value is returned.
/// @param k input kernel
/// @param scaled output vector to hold the resulting kernel
/// @param maxval the new maximum value
/// @return sum of the new kernel elements for normalization
/// @ingroup gVision
template <class S, class T>
T scaleKernel(const std::vector<S>& k, std::vector<T>& scaled, T maxval)
{
    unsigned int i,argmax=0;
    for (i=1;i<k.size();i++)
        if (k[i]>k[argmax])
            argmax = i;
    scaled.resize(k.size());
    T sum = 0;
    for (i=0;i<k.size();i++)
        sum += (scaled[i] = (T)((k[i]*maxval)/k[argmax]));
    return sum;
}

template <class T>
void convolveGaussian5_1(BasicImage<T>& I)
{
    int w = I.size().x;
    int h = I.size().y;
    int i,j;
    for (j=0;j<w;j++) {
        T* src = I.data()+j;
        T* end = src + w*(h-4);
        while (src != end) {
            T sum= (T)(0.0544887*(src[0]+src[4*w])
                    + 0.2442010*(src[w]+src[3*w])
                    + 0.4026200*src[2*w]);
            *(src) = sum;
            src += w;
        }
    }
    for (i=h-5;i>=0;i--) {
        T* src = I.data()+i*w;
        T* end = src + w-4;
        while (src != end) {
            T sum= (T)(0.0544887*(src[0]+src[4])
                    + 0.2442010*(src[1]+src[3])
                    + 0.4026200*src[2]);
            *(src+2*w+2)=sum;
            ++src;
        }
    }
}

namespace Exceptions {

    /// %Exceptions specific to vision algorithms
    /// @ingroup gException
    namespace Convolution {
        /// Base class for all Image_IO exceptions
        /// @ingroup gException
        struct All: public CVD::Exceptions::All {};

        /// Input images have incompatible dimensions
        /// @ingroup gException
        struct IncompatibleImageSizes : public All {
            IncompatibleImageSizes(const std::string & function)
            {
                what = "Incompatible image sizes in " + function;
	    }
	};

				
        /// Input images have incompatible dimensions
        /// @ingroup gException
        struct OddSizedKernelRequired : public All {
            OddSizedKernelRequired(const std::string & function)
            {
                what = "Odd sized kernel required in " + function;
            };
        };
    }
}
//void convolveGaussian5_1(BasicImage<byte>& I);

/// convolves an image with a box of given size.
/// @param I input image, modified in place
/// @param hwin window size, this is half of the box size
/// @ingroup gVision
template <class T> void convolveWithBox(const BasicImage<T>& I, BasicImage<T>& J, ImageRef hwin)
{
    typedef typename Pixel::traits<T>::wider_type sum_type;
    if (I.size() != J.size()) {
	throw Exceptions::Convolution::IncompatibleImageSizes("convolveWithBox");
    }
    int w = I.size().x;
    int h = I.size().y;
    ImageRef win = 2*hwin+ImageRef(1,1);
    const double factor = 1.0/(win.x*win.y);
    std::vector<sum_type> buffer(w*win.y);
    std::vector<sum_type> sums_v(w);
    sum_type* sums = &sums_v[0];
    sum_type* next_row = &buffer[0];
    sum_type* oldest_row = &buffer[0];
    zeroPixels(sums, w);
    const T* input = I.data();
    T* output = J[hwin.y] - hwin.x;
    for (int i=0; i<h; i++) {
	sum_type hsum=sum_type();
	const T* back = input;
	int j;
	for (j=0; j<win.x-1; j++)
	    hsum += input[j];
	for (; j<w; j++) {
	    hsum += input[j];
	    next_row[j] = hsum;
	    sums[j] += hsum;
	    hsum -= *(back++);
	}
	if (i >= win.y-1) {
	    assign_multiple(sums+win.x-1, factor, output+win.x-1, w-win.x+1);
	    differences(sums+win.x-1, oldest_row+win.x-1, sums+win.x-1, w-win.x+1);
	    output += w;
	    oldest_row += w;
	    if (oldest_row == &buffer[0] + w*win.y)
		oldest_row = &buffer[0];
	}    
	input += w;
	next_row += w;
	if (next_row == &buffer[0] + w*win.y)
	    next_row = &buffer[0];
    }
}

template <class T> inline void convolveWithBox(const BasicImage<T>& I, BasicImage<T>& J, int hwin)
{
    convolveWithBox(I, J, ImageRef(hwin,hwin));
}

template <class T> inline void convolveWithBox(BasicImage<T>& I, int hwin) {
    convolveWithBox(I,I,hwin);
}

template <class T> inline void convolveWithBox(BasicImage<T>& I, ImageRef hwin) {
    convolveWithBox(I,I,hwin);
}
    

template <class T, int A, int B, int C> void convolveSymmetric(Image<T>& I)
  {
    typedef typename Pixel::traits<T>::wider_type wider;
    static const wider S = (A+B+C+B+A);
    int width = I.size().x;
    int height = I.size().y;
    T* p = I.data();
    int i,j;
    for (i=0; i<height; i++) {
      wider a = p[0];
      wider b = p[1];
      wider c = p[2];
      wider d = p[3];
      p[0] = (T)(((c+c)*A+(b+b)*B + a*C) /S);
      p[1] = (T)(((b+d)*A+(a+c)*B + b*C) /S);
      for (j=0;j<width-4;j++,p++) {
        wider e = p[4];
        p[2] = (T)(((a+e)*A + (b+d)*B + c*C)/S);
        a = b; b = c; c = d; d = e;
      }
      p[2] = (T)(((a+c)*A + (b+d)*B + c*C) /S);
      p[3] = (T)(((b+b)*A + (c+c)*B + d*C) /S);
      p += 4;
    }
    for (j=0;j<width;j++) {
      p = I.data()+j;
      wider a = p[0];
      wider b = p[width];
      p[0] = (T)(((p[2*width]+p[2*width])*A+(b+b)*B + a*C) /S);
      p[width] = (T)(((b+p[width*3])*A+(a+p[2*width])*B + b*C) /S);
      for (i=0;i<height-4;i++) {
        wider c = p[2*width];
        p[2*width] = (T)(((a+p[4*width])*A + (b+p[3*width])*B + c*C)/S);
        a=b; b=c;
        p += width;
      }
      wider c = p[2*width];
      p[2*width] = (T)(((a+c)*A + (b+p[width*3])*B + c*C) /S);
      p[3*width] = (T)(((b+b)*A + (c+c)*B + p[width*3]*C) /S);
    }
  }

  template <class T, int A, int B, int C, int D> void convolveSymmetric(Image<T>& I)
  {
    typedef typename Pixel::traits<T>::wider_type wider;
    static const wider S = (A+B+C+D+C+B+A);
    int width = I.size().x;
    int height = I.size().y;
    T* p = I.data();
    int i,j;
    for (i=0; i<height; i++) {
      wider a = p[0];
      wider b = p[1];
      wider c = p[2];
      wider d = p[3];
      p[0] = (T)(((d+d)*A + (c+c)*B + (b+b)*C + a*D)/S);
      p[1] = (T)(((c+p[4])*A + (b+d)*B + (a+c)*C + b*D)/S);
      p[2] = (T)(((b+p[5])*A + (a+p[4])*B + (b+d)*C + c*D)/S);
      for (j=0;j<width-6;j++,p++) {
        d = p[3];
        p[3] = (T)(((a+p[6])*A + (b+p[5])*B + (c+p[4])*C + d*D)/S);
        a=b; b=c; c=d;
      }
      d = p[3];
      wider e = p[4];
      p[3] = (T)(((a+e)*A + (b+p[5])*B + (c+e)*C + d*D)/S);
      p[4] = (T)(((b+d)*A + (c+e)*B + (d+p[5])*C + e*D)/S);
      p[5] = (T)(((c+c)*A + (d+d)*B + (e+e)*C + p[5]*D)/S);
      p += 6;
    }
    for (j=0;j<width;j++) {
      p = I.data()+j;
      wider a = p[0];
      wider b = p[width];
      wider c = p[2*width];
      wider d = p[3*width];
      p[0] = (T)(((d+d)*A + (c+c)*B + (b+b)*C + a*D)/S);
      p[width] = (T)(((c+p[4*width])*A + (b+d)*B + (a+c)*C + b*D)/S);
      p[2*width] = (T)(((b+p[5*width])*A + (a+p[4*width])*B + (b+d)*C + c*D)/S);
      for (i=0;i<height-6;i++) {
        d = p[3*width];
        p[3*width] = (T)(((a+p[width*6])*A + (b+p[width*5])*B + (c+p[width*4])*C+d*D)/S);
        a=b; b=c; c=d;
        p += width;
      }
      d = p[3*width];
      wider e = p[4*width];
      p[3*width] = (T)(((a+e)*A + (b+p[5*width])*B + (c+e)*C + d*D)/S);
      p[4*width] = (T)(((b+d)*A + (c+e)*B + (d+p[5*width])*C + e*D)/S);
      p[5*width] = (T)(((c+c)*A + (d+d)*B + (e+e)*C + p[5*width]*D)/S);
    }
  }

template <class T, class K> void convolveSeparableSymmetric(Image<T>& I, const std::vector<K>& kernel, K divisor)
  {
    typedef typename Pixel::traits<T>::wider_type sum_type;
    int w = I.size().x;
    int h = I.size().y;
    int r = (int)kernel.size()/2;
    int i,j;
    int m;
    double factor = 1.0/divisor;
    for (j=0;j<w;j++) {
      T* src = I.data()+j;
      for (i=0; i<h-2*r; i++,src+=w) {
        sum_type sum = src[r*w]*kernel[r], v;
        for (m=0; m<r; m++)
	  sum += (src[m*w] + src[(2*r-m)*w]) * kernel[m];
	*(src) = static_cast<T>(sum * factor);
      }
    }
    int offset = r*w + r;
    for (i=h-2*r-1;i>=0;i--) {
      T* src = I[w];
      for (j=0;j<w-2*r;j++, src++) {
        sum_type sum = src[r] * kernel[r], v;
        for (m=0; m<r; m++)
	  sum += (src[m] + src[2*r-m])*kernel[m];
	*(src+offset) = static_cast<T>(sum*factor);
      }
    }
  }
  

template <class A, class B> struct GetPixelRowTyped {
  static inline const B* get(const A* row, int w, B* rowbuf) {
    std::copy(row, row+w, rowbuf);
    return rowbuf;
  }
};

template <class T> struct GetPixelRowTyped<T,T> {
  static inline const T* get(const T* row, int , T* ) {
    return row;
  }
};

template <class A, class B> const B* getPixelRowTyped(const A* row, int n, B* rowbuf) {
  return GetPixelRowTyped<A,B>::get(row,n,rowbuf);
}

template <class T, class S> struct CastCopy {
  static inline void cast_copy(const T* from, S* to, int count) {
    for (int i=0; i<count; i++)
      to[i] = static_cast<S>(from[i]);
  }
};

template <class T> struct CastCopy<T,T> {
  static inline void cast_copy(const T* from, T* to, int count) {
    std::copy(from, from+count, to);
  }
};

template <class T, class S> inline void cast_copy(const T* from, S* to, int count) { CastCopy<T,S>::cast_copy(from,to,count); }

template <class T, int N=-1, int C = Pixel::Component<T>::count> struct ConvolveMiddle {
  template <class S> static inline T at(const T* input, const S& factor, const S* kernel) { return ConvolveMiddle<T,-1,C>::at(input,factor, kernel, N); }
};

template <class T, int N> struct ConvolveMiddle<T,N,1> {
  template <class S> static inline T at(const T* input, const S& factor, const S* kernel) { return ConvolveMiddle<T,N-1>::at(input,factor, kernel) + (input[-N]+input[N])*kernel[N-1]; }
};

template <class T> struct ConvolveMiddle<T,-1,1> {
  template <class S> static inline T at(const T* input, const S& factor, const S* kernel, int ksize) {
    T hsum = *input * factor;
    for (int k=0; k<ksize; k++)
      hsum += (input[-k-1] + input[k+1]) * kernel[k];
    return hsum;
  }
};

template <class T, int C> struct ConvolveMiddle<T,-1, C> {
  template <class S> static inline T at(const T* input, const S& factor, const S* kernel, int ksize) {
    T hsum = *input * factor;
    for (int k=0; k<ksize; k++)
      hsum += (input[-k-1] + input[k+1]) * kernel[k];
    return hsum;
  }
};

template <class T> struct ConvolveMiddle<T,0,1> {
  template <class S> static inline T at(const T* input, const S& factor, const S* ) { return *input * factor; }
};

template <class T,class S> inline const T* convolveMiddle(const T* input, const S& factor, const S* kernel, int ksize, int n, T* output) {
#define CALL_CM(I) for (int j=0; j<n; ++j, ++input, ++output) { *output = ConvolveMiddle<T,I>::at(input, factor, kernel); } break
    
  switch (ksize) {
  case 0: CALL_CM(0);
  case 1: CALL_CM(1);
  case 2: CALL_CM(2);
  case 3: CALL_CM(3);
  case 4: CALL_CM(4);
  case 5: CALL_CM(5);
  case 6: CALL_CM(6);
  case 7: CALL_CM(7);
  case 8: CALL_CM(8);
  default: for (int j=0; j<n; j++, input++) { *(output++) = ConvolveMiddle<T,-1>::at(input, factor, kernel, ksize); }     
  }
  return input;
#undef CALL_CM
}


template <class T> inline void convolveGaussian(BasicImage<T>& I, double sigma, double sigmas=3.0)
{
  convolveGaussian(I,I,sigma,sigmas);
}

template <class T> void convolveGaussian(const BasicImage<T>& I, BasicImage<T>& out, double sigma, double sigmas=3.0)
{
    typedef typename Pixel::traits<typename Pixel::Component<T>::type>::float_type sum_comp_type;
    typedef typename Pixel::traits<T>::float_type sum_type;
    assert(out.size() == I.size());
    int ksize = (int)ceil(sigmas*sigma);
    //std::cerr << "sigma: " << sigma << " kernel: " << ksize << std::endl;
    std::vector<sum_comp_type> kernel(ksize);
    sum_comp_type ksum = sum_comp_type();
    for (int i=1; i<=ksize; i++)
	ksum += (kernel[i-1] = static_cast<sum_comp_type>(exp(-i*i/(2*sigma*sigma))));
    for (int i=0; i<ksize; i++)
	kernel[i] /= (2*ksum+1);
    double factor = 1.0/(2*ksum+1);
    int w = I.size().x;
    int h = I.size().y;
    int swin = 2*ksize;

    std::vector<sum_type> buffer(w*(swin+1));
    std::vector<sum_type> aligned_rowbuf(w);
    std::vector<sum_type> aligned_outbuf(w);

    sum_type* rowbuf = aligned_rowbuf.data();
    sum_type* outbuf = aligned_outbuf.data();

    std::vector<sum_type*> rows(swin+1);
    for (int k=0;k<swin+1;k++)
	rows[k] = buffer.data() + k*w;

    T* output = out.data();
    for (int i=0; i<h; i++) {
	sum_type* next_row = rows[swin];
	const sum_type* input = getPixelRowTyped(I[i], w, rowbuf);
	// beginning of row
	for (int j=0; j<ksize; j++) {
	    sum_type hsum = static_cast<sum_type>(input[j] * factor);
	    for (int k=0; k<ksize; k++)
		hsum += (input[std::max(j-k-1,0)] + input[j+k+1]) * kernel[k];
	    next_row[j] = hsum;
	}
	// middle of row
	input += ksize;
	input = convolveMiddle<sum_type, sum_comp_type>(input, static_cast<sum_comp_type>(factor), &kernel.front(), ksize, w-swin, next_row+ksize);
	// end of row
	for (int j=w-ksize; j<w; j++, input++) {
	    sum_type hsum = static_cast<sum_type>(*input * factor);
	    const int room = w-j;
	    for (int k=0; k<ksize; k++) {
		hsum += (input[-k-1] + input[std::min(k+1,room-1)]) * kernel[k];
	    }
	    next_row[j] = hsum;
	}
	// vertical
	if (i >= swin) {
	    const sum_type* middle_row = rows[ksize];
	    assign_multiple(middle_row, factor, outbuf, w);
	    for (int k=0; k<ksize; k++) {
		const sum_comp_type m = kernel[k];
		const sum_type* row1 = rows[ksize-k-1];
		const sum_type* row2 = rows[ksize+k+1];	
		add_multiple_of_sum(row1, row2, m, outbuf, w);
	    }
	    cast_copy(outbuf, output, w);
	    output += w;
	    if (i == h-1) {
		for (int r=0; r<ksize; r++) {
		    const sum_type* middle_row = rows[ksize+r+1];
		    assign_multiple(middle_row, factor, outbuf, w);
		    for (int k=0; k<ksize; k++) {
			const sum_comp_type m = kernel[k];
			const sum_type* row1 = rows[ksize+r-k];
			const sum_type* row2 = rows[std::min(ksize+r+k+2, swin)];
			add_multiple_of_sum(row1, row2, m, outbuf, w);
		    }
		    cast_copy(outbuf, output, w);
		    output += w;
		}	
	    }
	} else if (i == swin-1) {
	    for (int r=0; r<ksize; r++) {
		const sum_type* middle_row = rows[r+1];
		assign_multiple(middle_row, factor, outbuf, w);
		for (int k=0; k<ksize; k++) {
		    const sum_comp_type m = kernel[k];
		    const sum_type* row1 = rows[std::max(r-k-1,0)+1];
		    const sum_type* row2 = rows[r+k+2];	
		    add_multiple_of_sum(row1, row2, m, outbuf, w);
		}
		cast_copy(outbuf, output, w);
		output += w;
	    }
	}
    
	sum_type* tmp = rows[0];
	for (int r=0;r<swin; r++)
	    rows[r] = rows[r+1];
	rows[swin] = tmp;
    }
}

void compute_van_vliet_b(double sigma, double b[]);
void compute_triggs_M(const double b[], double M[][3]);
void van_vliet_blur(const double b[], const BasicImage<float> in, BasicImage<float> out);

void convolveGaussian(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas=3.0);
void convolveGaussian_fir(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas=3.0);

template <class T, class O, class K> void convolve_gaussian_3(const BasicImage<T>& I, BasicImage<O>& out, K k1, K k2)
{    
    assert(I.size() == out.size());
    const T* a=I.data();
    const int w = I.size().x;
    O* o = out.data()+w+1;
    int total = I.totalsize() - 2*w-2;
    const double cross = k1*k2;
    k1 *= k1;
    k2 *= k2;
    while (total--) {
	const double sum = k1*(a[0] + a[2] + a[w*2] + a[w*2+2]) + cross*(a[1] + a[w*2+1] + a[w] + a[w+2]) + k2*a[w+1];
	*o++ = Pixel::scalar_convert<O,T,double>(sum);
	++a;
    }
}

} // namespace CVD

#endif
