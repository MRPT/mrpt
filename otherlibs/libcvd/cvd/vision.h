
#ifndef CVD_VISION_H_
#define CVD_VISION_H_

#include <vector>
#include <memory>
#include <algorithm>

#include <cvd/vision_exceptions.h>
#include <cvd/image.h>
#include <cvd/internal/pixel_operations.h>
#include <cvd/utility.h>


#if defined(CVD_HAVE_TOON)
#include <TooN/TooN.h>
#include <TooN/helpers.h>
#endif

namespace CVD{



///Downsample an image using linear interpolation. 
///This will give horrendous aliasing if scale is more than 2, but not if it's
///substantially less, due to the low pass filter nature of bilinear
///interpretation. Don't use this function unless you know what youre doing!
///
///Image resampling has more or less the following meaning. The image is represented
///as a real valued signal of sample points, by one delta function per pixel. To linearly
///interpolate this to fill up the real domain, it's convolved with a triangle kernel which is 0 
///when it hits a neighbouring sample point, 1 at zero, and symmetric. Now we have a real valued
///signal, we can then sample it which is effectively a multiplication with a delta comb.
///
///The triangle kernel isn't band limited (though it does fall off), so when you resample, you will
///alias some high frequency information. But not all that much.
///
///
///@ingroup gVision
///@param in input image
///@param scale fraction to downsample
template<class C>
Image<C> linearInterpolationDownsample(const BasicImage<C>& in, float scale)
{
	Image<C> out(ImageRef(std::floor(in.size().x*scale), std::floor(in.size().y*scale)));
	
	scale = 1./scale;

	if(scale == 1)
		out.copy_from(in);
	else
		for(int r=0; r < out.size().y; r++)
		{
			float r_in = r * scale;
			int r_i = floor(r_in);
			float r_delta = r_in - floor(r_in);


			//4x unrolling makes it about 15% faster for some reason.
			//Maybe because of aliasing?
			int c=0;
			for(;c < out.size().x-4; c+=4)
			{
				float c_in1 = c * scale;
				int c_i1 = floor(c_in1);
				float c_delta1 = c_in1 - floor(c_in1);
				auto r11 = in[r_i  ][c_i1] * (1-c_delta1) + in[r_i  ][c_i1+1]*c_delta1;
				auto r21 = in[r_i+1][c_i1] * (1-c_delta1) + in[r_i+1][c_i1+1]*c_delta1;
				
				float c_in2 = (c+1) * scale;
				int c_i2 = floor(c_in2);
				float c_delta2 = c_in2 - floor(c_in2);
				auto r12 = in[r_i  ][c_i2] * (1-c_delta2) + in[r_i  ][c_i2+1]*c_delta2;
				auto r22 = in[r_i+1][c_i2] * (1-c_delta2) + in[r_i+1][c_i2+1]*c_delta2;
				
				float c_in3 = (c+2) * scale;
				int c_i3 = floor(c_in3);
				float c_delta3 = c_in3 - floor(c_in3);
				auto r13 = in[r_i  ][c_i3] * (1-c_delta3) + in[r_i  ][c_i3+1]*c_delta3;
				auto r23 = in[r_i+1][c_i3] * (1-c_delta3) + in[r_i+1][c_i3+1]*c_delta3;
				
				float c_in4 = (c+3) * scale;
				int c_i4 = floor(c_in4);
				float c_delta4 = c_in4 - floor(c_in4);
				auto r14 = in[r_i  ][c_i4] * (1-c_delta4) + in[r_i  ][c_i4+1]*c_delta4;
				auto r24 = in[r_i+1][c_i4] * (1-c_delta4) + in[r_i+1][c_i4+1]*c_delta4;
				


				out[r][c  ] = r11*(1-r_delta) + r21 * r_delta;
				out[r][c+1] = r12*(1-r_delta) + r22 * r_delta;
				out[r][c+2] = r13*(1-r_delta) + r23 * r_delta;
				out[r][c+3] = r14*(1-r_delta) + r24 * r_delta;
			}

			for(; c < out.size().x; c++)
			{
				float c_in1 = c * scale;
				int c_i1 = floor(c_in1);
				float c_delta1 = c_in1 - floor(c_in1);
				auto r11 = in[r_i  ][c_i1] * (1-c_delta1) + in[r_i  ][c_i1+1]*c_delta1;
				auto r21 = in[r_i+1][c_i1] * (1-c_delta1) + in[r_i+1][c_i1+1]*c_delta1;
				
				out[r][c  ] = r11*(1-r_delta) + r21 * r_delta;
			}
		}
	
	return out;
}


///Downsample an image using some fast hacks.
///
///The image is half-sampled as much as it can be, then 
///twoThirdsSampled if possible and then finally resampled with 
///linear interpolation. This ensures that the linear interpolation never goes
///further than a factor af about 1.5. Repeated area sampling and linear interpolation
///aren't perfect, so the resulting image won't be completely free from aliasing 
///artefacts. However, it's pretty good, decently fast for small rescales and very fast 
///for large ones.
///
///
///@ingroup gVision
///@param in input image
///@param scale fraction to downsample
template<class C>
Image<C> fastApproximateDownSample(const BasicImage<C>& in, double scale)
{
	const BasicImage<C>* current_ptr = &in;
	Image<C> reduced;

	while(scale < 0.5)
	{
		reduced = halfSample(*current_ptr);	
		current_ptr = & reduced;
		scale *=2;
	}

	if(scale < 2./3)
	{
		reduced = twoThirdsSample(*current_ptr);
		current_ptr = & reduced;
		scale *=3./2.;
	}
	
	return linearInterpolationDownsample(*current_ptr, scale);
}










/** Subsamples an image to 2/3 of its size by averaging 3x3 blocks into 2x2 blocks.
@param in input image
@param out output image (must be <code>out.size() == in.size()/3*2 </code>)
@throw IncompatibleImageSizes if out does not have the correct dimensions.
@ingroup gVision
*/
template<class C> void twoThirdsSample(const BasicImage<C>& in, BasicImage<C>& out)
{
    typedef typename Pixel::traits<C>::wider_type sum_type;
	if( (in.size()/3*2) != out.size())
        throw Exceptions::Vision::IncompatibleImageSizes(__FUNCTION__);
	
	for(int yy=0, y=0; y < in.size().y-2; y+=3, yy+=2)
		for(int xx=0, x=0; x < in.size().x-2; x+=3, xx+=2)
		{
			// a b c
			// d e f
			// g h i

			sum_type b = in[y][x+1]*2;
			sum_type d = in[y+1][x]*2;
			sum_type f = in[y+1][x+2]*2;
			sum_type h = in[y+2][x+1]*2;
			sum_type e = in[y+1][x+1];

			out[yy][xx]     = static_cast<C>((in[  y][  x]*4+b+d+e)/9);
			out[yy][xx+1]   = static_cast<C>((in[  y][x+2]*4+b+f+e)/9);
			out[yy+1][xx]   = static_cast<C>((in[y+2][  x]*4+h+d+e)/9);
			out[yy+1][xx+1] = static_cast<C>((in[y+2][x+2]*4+h+f+e)/9);
		}
}

/**
@overload
*/
void twoThirdsSample(const BasicImage<byte>& in, BasicImage<byte>& out);

///Subsamples an image by averaging 3x3 blocks in to 2x2 ones.
/// Note that this is performed using lazy evaluation, so subsampling
/// happens on assignment, and memory allocation is not performed if
/// unnecessary.
/// @param from The image to convert from
/// @return The converted image
/// @ingroup gVision
template<class C> Image<C> twoThirdsSample(const BasicImage<C>& from)
{
	Image<C> to(from.size()/3*2);
	twoThirdsSample(from, to);
	return to;
}


    /// subsamples an image to half its size by averaging 2x2 pixel blocks
    /// @param in input image
    /// @param out output image, must have the right dimensions versus input image
    /// @throw IncompatibleImageSizes if out does not have half the dimensions of in
    /// @ingroup gVision
    template <class T>
    void halfSample(const BasicImage<T>& in, BasicImage<T>& out)
    {
	typedef typename Pixel::traits<T>::wider_type sum_type;
	if( (in.size()/2) != out.size())
	    throw Exceptions::Vision::IncompatibleImageSizes("halfSample");
	
	for(int yo=0; yo < out.size().y; yo++)
	    for(int xo=0; xo < out.size().x; xo++)
	    	out[yo][xo] = static_cast<T>((sum_type(in[yo*2][xo*2]) + in[yo*2+1][xo*2] + in[yo*2][xo*2+1] + in[yo*2+1][xo*2+1])/4);
    }

void halfSample(const BasicImage<byte>& in, BasicImage<byte>& out);

/// subsamples an image to half its size by averaging 2x2 pixel blocks
/// @param in input image
/// @return The output image
/// @throw IncompatibleImageSizes if out does not have half the dimensions of in
/// @ingroup gVision
template <class T>
inline Image<T> halfSample(const BasicImage<T>& in)
{
	Image<T> out(in.size()/2);
	halfSample(in, out);
	return out;
}

/// subsamples an image repeatedly by half its size by averaging 2x2 pixel blocks.
/// This version will not create a copy for 0 octaves because it receives already
/// an Image and will reuse the data.
/// @param in input image
/// @param octaves number of halfsamplings 
/// @return The output image
/// @throw IncompatibleImageSizes if out does not have half the dimensions of in
/// @ingroup gVision
template <class T>
inline Image<T> halfSample( Image<T> in, unsigned int octaves){
    for( ;octaves > 0; --octaves){
        in = halfSample(in);
    }
    return in;
}

/// thresholds an image by setting all pixel values below a minimum to 0 and all values above to a given maximum
/// @param im input image changed in place
/// @param minimum threshold value
/// @param hi maximum value for values above the threshold
/// @ingroup gVision
template <class T>
void threshold(BasicImage<T>& im, const T& minimum, const T& hi)
{
  typename BasicImage<T>::iterator it = im.begin();
  typename BasicImage<T>::iterator end = im.end();
  while (it != end) {
    if (*it < minimum)
      *it = T();
    else
      *it = hi;
    ++it;
  }
}

/// computes mean and stddev of intensities in an image. These are computed for each component of the
/// pixel type, therefore the output are two pixels with mean and stddev for each component.
/// @param im input image
/// @param mean pixel element containing the mean of intensities in the image for each component
/// @param stddev pixel element containing the standard deviation for each component
/// @ingroup gVision
template <class T>
void stats(const BasicImage<T>& im, T& mean, T& stddev)
{
    const int c = Pixel::Component<T>::count;
    double v;
    double sum[c] = {0};
    double sumSq[c] = {0};
    const T* p = im.data();
    const T* end = im.data()+im.totalsize();
    while (p != end) {
        for (int k=0; k<c; k++) {
            v = Pixel::Component<T>::get(*p, k);
            sum[k] += v;
            sumSq[k] += v*v;
        }
        ++p;
    }
    for (int k=0; k<c; k++) {
        double m = sum[k]/im.totalsize();
        Pixel::Component<T>::get(mean,k) = (typename Pixel::Component<T>::type)m;
        sumSq[k] /= im.totalsize();
        Pixel::Component<T>::get(stddev,k) = (typename Pixel::Component<T>::type)sqrt(sumSq[k] - m*m);
    }
}

/// a functor multiplying pixels with constant value.
/// @ingroup gVision
template <class T>
struct multiplyBy
{
  const T& factor;
  multiplyBy(const T& f) : factor(f) {};
  template <class S> inline S operator()(const S& s) const {
    return s * factor;
  }
};

template <class S, class T, int Sn=Pixel::Component<S>::count, int Tn=Pixel::Component<T>::count> struct Gradient;

template <class S, class T> struct Gradient<S,T,1,2> {
	typedef typename Pixel::Component<S>::type SComp;
	typedef typename Pixel::Component<T>::type TComp;
	typedef typename Pixel::traits<SComp>::wider_type diff_type;
	static void gradient(const BasicImage<S>& I, BasicImage<T>& grad) {
		for(int y=0; y < I.size().y; y++)
			for(int x=0; x < I.size().x; x++)
			{
				Pixel::Component<T>::get(grad[y][x], 0) = Pixel::scalar_convert<TComp,SComp,diff_type>(diff_type(I[y][x+1]) - I[y][x-1]);
				Pixel::Component<T>::get(grad[y][x], 1) = Pixel::scalar_convert<TComp,SComp,diff_type>(diff_type(I[y+1][x]) - I[y-1][x]);
			}

		zeroBorders(grad);
	}
};

/// computes the gradient image from an image. The gradient image contains two components per pixel holding
/// the x and y components of the gradient.
/// @param im input image
/// @param out output image, must have the same dimensions as input image
/// @throw IncompatibleImageSizes if out does not have same dimensions as im
/// @ingroup gVision
template <class S, class T> void gradient(const BasicImage<S>& im, BasicImage<T>& out)
{
  if( im.size() != out.size())
    throw Exceptions::Vision::IncompatibleImageSizes("gradient");
  Gradient<S,T>::gradient(im,out);
}


#ifndef DOXYGEN_IGNORE_INTERNAL
void gradient(const BasicImage<byte>& im, BasicImage<short[2]>& out);
#endif


template <class T, class S, typename Precision> inline void sample(const BasicImage<S>& im, Precision x, Precision y, T& result)
{
  typedef typename Pixel::Component<S>::type SComp;
  typedef typename Pixel::Component<T>::type TComp;
  const int lx = (int)x;
  const int ly = (int)y;
  x -= lx;
  y -= ly;
  for(unsigned int i = 0; i < Pixel::Component<T>::count; i++){
    Pixel::Component<T>::get(result,i) = Pixel::scalar_convert<TComp,SComp>(
        (1-y)*((1-x)*Pixel::Component<S>::get(im[ly][lx],i) + x*Pixel::Component<S>::get(im[ly][lx+1], i)) +
          y * ((1-x)*Pixel::Component<S>::get(im[ly+1][lx],i) + x*Pixel::Component<S>::get(im[ly+1][lx+1],i)));
  }
 }

template <class T, class S, typename Precision> inline T sample(const BasicImage<S>& im, Precision x, Precision y){
    T result;
    sample( im, x, y, result);
    return result;
}

inline void sample(const BasicImage<float>& im, double x, double y, float& result)
{
    const int lx = (int)x;
    const int ly = (int)y;
    const int w = im.row_stride();
    const float* base = im[ly]+lx;
    const float a = base[0];
    const float b = base[1];
    const float c = base[w];
    const float d = base[w+1];
    const float e = a-b;
    x-=lx;
    y-=ly;
    result = (float)(x*(y*(e-c+d)-e)+y*(c-a)+a);
}

#if defined (CVD_HAVE_TOON)

/**
 * a normal member taking two arguments and returning an integer value.
 * @param in a image containing the information to be extracted.
 * @param out the image to be filled.  The whole image out image is filled by the in image.
 * @param M the matrix used to map point in the out matrix to those in the in matrix
 * @param inOrig origin in the in image
 * @param outOrig origin in the out image
 * @return the number of pixels not in the in image 
 * @Note: this will collide with transform in the std namespace
 */
template <typename T, typename S, typename P>
int transform(const BasicImage<S>& in, BasicImage<T>& out, const TooN::Matrix<2, 2, P>& M, const TooN::Vector<2, P>& inOrig, const TooN::Vector<2, P>& outOrig, const T defaultValue = T())
{
    const int w = out.size().x, h = out.size().y, iw = in.size().x, ih = in.size().y; 
    const TooN::Vector<2, P> across = M.T()[0];
    const TooN::Vector<2, P> down =   M.T()[1];
   
    const TooN::Vector<2, P> p0 = inOrig - M*outOrig;
    const TooN::Vector<2, P> p1 = p0 + w*across;
    const TooN::Vector<2, P> p2 = p0 + h*down;
    const TooN::Vector<2, P> p3 = p0 + w*across + h*down;
        
    // ul --> p0
    // ur --> w*across + p0
    // ll --> h*down + p0
    // lr --> w*across + h*down + p0
    P min_x = p0[0], min_y = p0[1];
    P max_x = min_x, max_y = min_y;
   
    // Minimal comparisons needed to determine bounds
    if (across[0] < 0)
	min_x += w*across[0];
    else
	max_x += w*across[0];
    if (down[0] < 0)
	min_x += h*down[0];
    else
	max_x += h*down[0];
    if (across[1] < 0)
	min_y += w*across[1];
    else
	max_y += w*across[1];
    if (down[1] < 0)
	min_y += h*down[1];
    else
	max_y += h*down[1];
   
    // This gets from the end of one row to the beginning of the next
    const TooN::Vector<2, P> carriage_return = down - w*across;

    //If the patch being extracted is completely in the image then no 
    //check is needed with each point.
    if (min_x >= 0 && min_y >= 0 && max_x < iw-1 && max_y < ih-1) 
    {
	TooN::Vector<2, P> p = p0;
	for (int i=0; i<h; ++i, p+=carriage_return)
	    for (int j=0; j<w; ++j, p+=across) 
		sample(in,p[0],p[1],out[i][j]);
	return 0;
    } 
    else // Check each source location
    {
	// Store as doubles to avoid conversion cost for comparison
	const P x_bound = iw-1;
	const P y_bound = ih-1;
	int count = 0;
	TooN::Vector<2, P> p = p0;
	for (int i=0; i<h; ++i, p+=carriage_return) {
	    for (int j=0; j<w; ++j, p+=across) {
		//Make sure that we are extracting pixels in the image
		if (0 <= p[0] && 0 <= p[1] &&  p[0] < x_bound && p[1] < y_bound)
		    sample(in,p[0],p[1],out[i][j]);
		else {
            out[i][j] = defaultValue;
		    ++count;
		}
	    }
	}
	return count;
    }
}

  template <class T>  void transform(const BasicImage<T>& in, BasicImage<T>& out, const TooN::Matrix<3>& Minv /* <-- takes points in "out" to points in "in" */)
  {
    TooN::Vector<3> base = Minv.T()[2];
    TooN::Vector<2> offset;
    offset[0] = in.size().x/2;
    offset[1] = in.size().y/2;
    offset -= TooN::project(base);
    TooN::Vector<3> across = Minv.T()[0];
    TooN::Vector<3> down = Minv.T()[1];
    double w = in.size().x-1;
    double h = in.size().y-1;
    int ow = out.size().x;
    int oh = out.size().y;
    base -= down*(oh/2) + across*(ow/2);
    for (int row = 0; row < oh; row++, base+=down) {
      TooN::Vector<3> x = base;
      for (int col = 0; col < ow; col++, x += across) {
	TooN::Vector<2> p = project(x) + offset;
	if (p[0] >= 0 && p[0] <= w-1 && p[1] >=0 && p[1] <= h-1)
	  sample(in,p[0],p[1], out[row][col]);
	else
	  zeroPixel(out[row][col]);
      }
    }
  }
  
/// warp or unwarps an image according to two camera models.
template <typename T, typename CAM1, typename CAM2>
void warp( const BasicImage<T> & in, const CAM1 & cam_in, BasicImage<T> & out, const CAM2 & cam_out){
	const ImageRef size = out.size();
	for(int y = 0; y < size.y; ++y){
		for(int x = 0; x < size.x; ++x){
			TooN::Vector<2> l = cam_in.project(cam_out.unproject(TooN::makeVector(x,y)));
			if(l[0] >= 0 && l[0] <= in.size().x - 1 && l[1] >= 0 && l[1] <= in.size().y -1){
				sample(in, l[0], l[1], out[y][x]);
			} else 
				out[y][x] = T();
		}
	}
}

/// warps or unwarps an image according to two camera models and
/// returns the result image. The size of the output image needs to be
/// passed in as well.
template <typename T, typename CAM1, typename CAM2>
Image<T> warp( const BasicImage<T> & in, const CAM1 & cam_in, const ImageRef & size, const CAM2 & cam_out){
	Image<T> result(size);
	warp(in, cam_in, result, cam_out);
	return result;
}

/// warps or unwarps an image according to two camera models and
/// returns the result image. The size of the output image is the
/// same as the input image size.
template <typename T, typename CAM1, typename CAM2>
Image<T> warp( const BasicImage<T> & in, const CAM1 & cam_in, const CAM2 & cam_out){
	Image<T> result(in.size());
	warp(in, cam_in, result, cam_out);
	return result;
}

#endif

/// flips an image vertically in place.
template <class T> void flipVertical( Image<T> & in )
{
  int w = in.size().x;
  T * buffer = new T[w];
  T * top = in.data();
  T * bottom = top + (in.size().y - 1)*w;
  while( top < bottom )
  {
    std::copy(top, top+w, buffer);
    std::copy(bottom, bottom+w, top);
    std::copy(buffer, buffer+w, bottom);
    top += w;
    bottom -= w;
  }
  delete[] buffer;
}

/// flips an image horizontally in place.
template <class T> void flipHorizontal( Image<T> & in )
{
  int w = in.size().x;
  int h = in.size().y;
  T * buffer = new T[w];
  T * left = in.data();
  T * right = left + w;
  int row = 0;
  while(row < h)
  {
    std::copy(left, right, buffer);
    std::reverse_copy(buffer, buffer+w-1, left);
    row++;
    left += w;
    right += w;
  }
  delete[] buffer;
}


namespace median {
    template <class T> inline T median3(T a, T b, T c) {
	if (b<a)
	    return std::max(b,std::min(a,c));
	else
	    return std::max(a,std::min(b,c));	
    }
    
    template <class T> inline void sort3(T& a, T& b, T& c) {
	using std::swap;
	if (b<a) swap(a,b);
	if (c<b) swap(b,c);
	if (b<a) swap(a,b);
    }
    
    template <class T> T median_3x3(const T* p, const int w) {
	T a = p[-w-1], b = p[-w], c = p[-w+1], d=p[-1], e=p[0], f=p[1], g=p[w-1], h=p[w], i=p[w+1];
	sort3(a,b,c);
	sort3(d,e,f);
	sort3(g,h,i);
	e = median3(b,e,h);
	g = std::max(std::max(a,d),g);
	c = std::min(c,std::min(f,i));
	return median3(c,e,g);
    }
    
    template <class T> void median_filter_3x3(const T* p, const int w, const int n, T* out)
    {
	T a = p[-w-1], b = p[-w], d=p[-1], e=p[0], g=p[w-1], h=p[w];
	sort3(a,d,g);
	sort3(b,e,h);
	for (int j=0; j<n; ++j, ++p, ++out) {
	    T c = p[-w+1], f = p[1], i = p[w+1];
	    sort3(c,f,i);
	    *out = median3(std::min(std::min(g,h),i), 
			   median3(d,e,f), 
			   std::max(std::max(a,b),c));
	    a=b; b=c; d=e; e=f; g=h; h=i;
	}
    }
}

    template <class T> void median_filter_3x3(const BasicImage<T>& I, BasicImage<T> out)
    {
	assert(out.size() == I.size());
	const int s = I.row_stride();
	const int n = I.size().x - 2;
	for (int i=1; i+1<I.size().y; ++i)
	    median::median_filter_3x3(I[i]+1, s, n, out[i]+1);
    }

void median_filter_3x3(const BasicImage<byte>& I, BasicImage<byte> out);

//template<class T>


}; // namespace CVD

#endif // CVD_VISION_H_
