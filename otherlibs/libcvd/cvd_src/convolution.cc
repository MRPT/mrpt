#include "cvd/convolution.h"
#include <cmath>
using namespace std;

namespace CVD {

namespace Internal {

void convolveSeparableGray(unsigned char* I, unsigned int width, unsigned int height, const int kernel[], unsigned int size, int divisor)
{
    std::vector<unsigned char> buffer(width>height ? width : height);
    unsigned char* p = I;
    unsigned int i,j,m;
    if (size%2 == 0) {
		throw Exceptions::Convolution::OddSizedKernelRequired("convolveSeparable");
    }
    for (i=height; i>0; i--) {
        for (j=0;j<width-size+1;j++) {
            int sum = 0;
            for (m=0; m<size; m++)
                sum += p[j+m] * kernel[m];
            buffer[j] = (unsigned char)(sum/divisor);
        }
        memcpy(p+size/2, &buffer.front(), width-size+1);
        p += width;
    }
    for (j=0;j<width-size+1;j++) {
        p = I+j+(size/2)*width;
        for (i=0; i<height;i++)
            buffer[i] = I[i*width+j];
        for (i=0;i<height-size+1;i++) {
            int sum = 0;
            for (m=0; m<size; m++)
                sum += buffer[i+m] * kernel[m];
            *p = (unsigned char)(sum/divisor);
            p += width;
        }
    }
}

};

void convolveGaussian5_1(Image<byte>& I)
{
    int w = I.size().x;
    int h = I.size().y;
    int i,j;
    for (j=0;j<w;j++) {
        byte* src = I.data()+j;
        byte* end = src + w*(h-4);
        while (src != end) {
            int sum= (3571*(src[0]+src[4*w])
                    + 16004*(src[w]+src[3*w])
                    + 26386*src[2*w]);
            *(src) = sum >> 16;
            src += w;
        }
    }
    for (i=h-5;i>=0;i--) {
        byte* src = I.data()+i*w;
        byte* end = src + w-4;
        while (src != end) {
            int sum= (3571*(src[0]+src[4])
                    + 16004*(src[1]+src[3])
                    + 26386*src[2]);
            *(src+2*w+2) = sum >> 16;
            ++src;
        }
    }
}

double compute_van_vliet_variance(const double d[])
{
    const double a = d[0];
    const double b = d[1];
    const double num = 2*(a*(a*(a-2) + 1 + b*b) - 2*b*b);
    const double den1 = a*a - b*b -2*a + 1;
    const double den2 = 2*(a-1)*b;
    const double inv_den = 1.0/(den1*den1 + den2*den2);

    const double inv_den3 = 1.0/((d[2]-1)*(d[2]-1));

    return 2*(num * inv_den + d[2] * inv_den3);
}

double compute_van_vliet_variance(const double d[], double J[3])
{
    const double a = d[0];
    const double b = d[1];
    const double num = 2*(a*(a*(a-2) + 1 + b*b) - 2*b*b);
    const double den1 = a*a - b*b -2*a + 1;
    const double den2 = 2*(a-1)*b;
    const double inv_den = 1.0/(den1*den1 + den2*den2);
    const double inv_den3 = 1.0/((d[2]-1)*(d[2]-1));

    double N_D = num*inv_den;
    J[0] = 2*inv_den * (2*(a*(3*a - 4) + (1+b*b)) - N_D * 4*(den1*(a-1) + den2*b));
    J[1] = 2*inv_den * (4*b*(a - 2) - N_D * 4*(den2*(a-1) - den1*b));
    J[2] = 2*inv_den3*(1 - d[2]*inv_den3*2*(d[2]-1));

    return 2*(N_D + d[2] * inv_den3);
}

void compute_scaling_jacobian(const double d[], double J[])
{
    double rr = d[0]*d[0] + d[1]*d[1];
    double lnr = 0.5 * log(rr);
    double theta = atan2(d[1], d[0]);
    J[0] = d[0]*lnr - d[1]*theta;
    J[1] = d[0]*theta + d[1]*lnr;
    J[2] = d[2]*log(d[2]);
}

void scale_d(double d[], double p)
{
    double theta = atan2(d[1], d[0]);
    double rr = d[0]*d[0] + d[1]*d[1];
    double new_theta = theta*p;
    double new_r = pow(rr, p*0.5);
    d[0] = new_r * cos(new_theta);
    d[1] = new_r * sin(new_theta);
    d[2] = pow(d[2], p);
}

void compute_van_vliet_scaled_d(double sigma, double d[])
{
    // Approximately sigma = 2
    d[0] = 1.41650;
    d[1] = 1.00829;
    d[2] = 1.86543;

    // Cubic fitted to log(p) s.t. poles->poles^p gives sigma
    const double A = 3.69892409013e-03;
    const double B =-4.28967830150e-02;
    const double C =-3.38065667167e-01;
    const double D = 5.44264202732e-01;

    double log_var = 2*log(sigma);
    double log_p = D + log_var*(C + log_var*(B + log_var*A));
    double predicted_p = exp(log_p);
    
    scale_d(d, predicted_p);

    double total_p = 1;
    const double target_var = sigma*sigma;

    // Newton-Rapheson on scale of poles

    for (int i=0; i<5; ++i)
    {
	double sj[3], vj[3];
	compute_scaling_jacobian(d, sj);
	double v = target_var - compute_van_vliet_variance(d, vj);
	double step = v / (vj[0]*sj[0] + vj[1]*sj[1] + vj[2]*sj[2]);
	if (abs(step) < 1e-6)
	    break;
	double exp_step = exp(std::min(std::max(step, -1.0), 1.0));
	scale_d(d, exp_step);
	total_p *= exp_step;
    }
}

void build_van_vliet_b(const double d[], double b[])
{
    double B = 1.0/(d[2]*(d[0]*d[0] + d[1]*d[1]));
    b[2] = -B;
    b[1] = B*(2*d[0] + d[2]);
    b[0] = -B*(d[0]*d[0] + d[1]*d[1] + 2*(d[0] * d[2]));
}

    void compute_van_vliet_b(double sigma, double b[])
    {
	double d[3];
	compute_van_vliet_scaled_d(sigma, d);
	build_van_vliet_b(d, b);
    }

void compute_triggs_M(const double b[], double M[][3])
{
    const double a1= -b[0];
    const double a2= -b[1];
    const double a3= -b[2];

    const double factor = 1.0/((1+a1-a2+a3)*
			       (1-a1-a2-a3)*
			       (1+a2+(a1-a3)*a3));
    M[0][0] = factor*(1-a2-a1*a3-a3*a3);
    M[0][1] = factor*(a3+a1)*(a2+a1*a3);
    M[0][2] = factor*a3*(a1 + a2*a3);
    M[1][0] = a1*M[0][0] + M[0][1];
    M[1][1] = a2*M[0][0] + M[0][2];
    M[1][2] = a3*M[0][0];
    M[2][0] = a1*M[1][0] + M[1][1];
    M[2][1] = a2*M[1][0] + M[1][2];    
    M[2][2] = a3*M[1][0];
}

inline void forward_to_backward(const double M[][3], const double i_plus, const double inv_alpha, double& y1, double& y2, double& y3)
{
    const double u_plus = i_plus*inv_alpha;
    const double v_plus = u_plus*inv_alpha;
    double x1=y1-u_plus, x2=y2-u_plus, x3=y3-u_plus;
    y1 = M[0][0]*x1 + M[0][1]*x2 + M[0][2]*x3 + v_plus;
    y2 = M[1][0]*x1 + M[1][1]*x2 + M[1][2]*x3 + v_plus;
    y3 = M[2][0]*x1 + M[2][1]*x2 + M[2][2]*x3 + v_plus;    
}

template <class T> inline T clamp01(T x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }


// Implementation of Young-van Vliet third-order recursive gaussian filter.
// See "Recursive Gaussian Derivative Filters", by van Vliet, Young and Verbeck, 1998
// and "Boundary Conditions for Young - van Vliet Recursive Filtering", by Triggs and Sdika, 2005
// Can result in values just outside of the input range
void van_vliet_blur(const double b[], const CVD::BasicImage<float> in, CVD::BasicImage<float> out)
{
    assert(in.size() == out.size());
    const int w = in.size().x;
    const int h = in.size().y;

    double M[3][3];
    compute_triggs_M(b, M);

    vector<double> tmp(w);
    
    const double b0 = b[0];
    const double b1 = b[1];
    const double b2 = b[2];

    const int rw = w%4;
    const double alpha = 1 + b0 + b1 + b2;
    const double inv_alpha = 1.0/alpha;

    for (int i=0; i<h; ++i) {
	const float* p = in[i]+w-1;

	double y3, y2, y1;
	y3 = y2 = y1 = inv_alpha* *p;


	for (int j=w-1; j-3>=0; j-=4, p-=4) {
	    double y0 = p[0] - (b0*y1 + b1*y2 + b2 * y3);
	    y3 = p[-1] - (b0*y0 + b1*y1 + b2 * y2);
	    y2 = p[-2] - (b0*y3 + b1*y0 + b2 * y1);
	    y1 = p[-3] - (b0*y2 + b1*y3 + b2 * y0);
	    tmp[j] = y0;
	    tmp[j-1] = y3;
	    tmp[j-2] = y2;
	    tmp[j-3] = y1;
	}

	for (int j=rw-1; j>=0; --j, --p) {
	    double y0 = p[0] - (b0*y1 + b1*y2 + b2 * y3);
	    tmp[j] = y0;
	    y3 = y2; y2 = y1; y1 = y0;
	}

	{
	    const double i_plus = p[1];
	    double y0 = i_plus - (b0*y1 + b1*y2 + b2*y3);
	    y3=y2; y2=y1; y1=y0;
	    forward_to_backward(M, i_plus, inv_alpha, y1, y2, y3);		    
	}

	float* o = out[i];
	for (int j=0; j+3<w; j+=4, o+=4) {
	    double y0 = tmp[j] - (b0*y1 + b1*y2 + b2 * y3);
	    y3 = tmp[j+1] - (b0*y0 + b1*y1 + b2 * y2);
	    y2 = tmp[j+2] - (b0*y3 + b1*y0 + b2 * y1);
	    y1 = tmp[j+3] - (b0*y2 + b1*y3 + b2 * y0);
	    o[0] = (float)y0;
	    o[1] = (float)y3;
	    o[2] = (float)y2;
	    o[3] = (float)y1;
	}

	for (int j=w-rw; j<w; ++j, ++o) {
	    double y0 = tmp[j] - (b0*y1 + b1*y2 + b2 * y3);
	    o[0] = (float)y0;
	    y3 = y2; y2 = y1; y1 = y0;
	}

    }

    tmp.resize(h);

    const double alpha_fourth = alpha*alpha*alpha*alpha;
   
    const int rh = h%4;
    const int stride = out.row_stride();
    for (int i=0; i<w; ++i) {
	double y3, y2, y1;

	const float* in = out[h-1] + i;
	y3 = y2 = y1 = inv_alpha* *in;

	for (int j=h-1; j-3>=0; j-=4, in -= stride) {
	    double y0 = in[0] - (b0*y1 + b1*y2 + b2 * y3);
	    in -= stride;
	    y3 = in[0] - (b0*y0 + b1*y1 + b2 * y2);
	    in -= stride;
	    y2 = in[0] - (b0*y3 + b1*y0 + b2 * y1);
	    in -= stride;
	    y1 = in[0] - (b0*y2 + b1*y3 + b2 * y0);
	    tmp[j] = y0;
	    tmp[j-1] = y3;
	    tmp[j-2] = y2;
	    tmp[j-3] = y1;
	}

	for (int j=rh-1; j>=0; --j, in-=stride) {
	    double y0 = in[0] - (b0*y1 + b1*y2 + b2 * y3);
	    tmp[j] = y0;
	    y3 = y2; y2 = y1; y1 = y0;
	}

	{
	    const double i_plus = in[stride];
	    double y0 = i_plus - (b0*y1 + b1*y2 + b2*y3);
	    y3=y2; y2=y1; y1=y0;
	    forward_to_backward(M, i_plus, inv_alpha, y1, y2, y3);
	}

	float* o = out[0]+i;
	for (int j=0; j+3<h; j+=4) {
	    double y0 = tmp[j] - (b0*y1 + b1*y2 + b2 * y3);
	    y3 = tmp[j+1] - (b0*y0 + b1*y1 + b2 * y2);
	    y2 = tmp[j+2] - (b0*y3 + b1*y0 + b2 * y1);
	    y1 = tmp[j+3] - (b0*y2 + b1*y3 + b2 * y0);
	    o[0] = (float)(alpha_fourth*y0);//clamp01(alpha_fourth*y0); 
	    o+=stride;
	    o[0] = (float)(alpha_fourth*y3);//clamp01(alpha_fourth*y3); 
	    o+=stride;
	    o[0] = (float)(alpha_fourth*y2);//clamp01(alpha_fourth*y2); 
	    o+=stride;
	    o[0] = (float)(alpha_fourth*y1);//clamp01(alpha_fourth*y1); 
	    o+=stride;
	}

	for (int j=h-rh; j<h; ++j, o+=stride) {
	    double y0 = tmp[j] - (b0*y1 + b1*y2 + b2 * y3);
	    o[0] = (float)(alpha_fourth*y0);//clamp01(alpha_fourth*y0);
	    y3 = y2; y2 = y1; y1 = y0;
	}
    }
}


};

