#include <cvd/convolution.h>
using namespace std;

namespace CVD
{

    // Try to choose the fastest method
    void convolveGaussian(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas)
    {
	int ksize = (int)ceil(sigma*sigmas);
	if (ksize > 6) {
	    double b[3];
	    compute_van_vliet_b(sigma, b);
	    van_vliet_blur(b, I, out);
	} else
	    convolveGaussian<float>(I, out, sigma, sigmas);
    }    
    
    void convolveGaussian_fir(const BasicImage<float>& I, BasicImage<float>& out, double sigma, double sigmas)
	{
	    convolveGaussian<float>(I, out, sigma, sigmas);
	}
}
