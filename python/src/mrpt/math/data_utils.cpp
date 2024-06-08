#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/filters.h>
#include <mrpt/math/fourier.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <sstream> // __str__
#include <utility>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_math_data_utils(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::averageLogLikelihood(const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/data_utils.h line:515
	M("mrpt::math").def("averageLogLikelihood", (double (*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::averageLogLikelihood, "A numerically-stable method to compute average likelihood values with\n strongly different ranges (unweighted likelihoods: compute the arithmetic\n mean).\n  This method implements this equation:\n\n  \n\n See also the \n* href=\"http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability\">tutorial\n page.\n \n\n\n \n\nC++: mrpt::math::averageLogLikelihood(const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("logLikelihoods"));

	// mrpt::math::averageWrap2Pi(const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/data_utils.h line:522
	M("mrpt::math").def("averageWrap2Pi", (double (*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::averageWrap2Pi, "Computes the average of a sequence of angles in radians taking into account\n the correct wrapping in the range \n\n, for example, the mean\n of (2,-2) is \n\n, not 0.\n \n\n\n \n\nC++: mrpt::math::averageWrap2Pi(const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("angles"));

	// mrpt::math::averageLogLikelihood(const class mrpt::math::CVectorDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/data_utils.h line:536
	M("mrpt::math").def("averageLogLikelihood", (double (*)(const class mrpt::math::CVectorDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::averageLogLikelihood, "A numerically-stable method to average likelihood values with strongly\n different ranges (weighted likelihoods).\n  This method implements this equation:\n\n  \n\n\n See also the \n* href=\"http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability\">tutorial\n page.\n \n\n\n \n\nC++: mrpt::math::averageLogLikelihood(const class mrpt::math::CVectorDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("logWeights"), pybind11::arg("logLikelihoods"));

	// mrpt::math::normalPDF(double, double, double) file:mrpt/math/distributions.h line:25
	M("mrpt::math").def("normalPDF", (double (*)(double, double, double)) &mrpt::math::normalPDF, "Evaluates the univariate normal (Gaussian) distribution at a given point\n \"x\".\n\nC++: mrpt::math::normalPDF(double, double, double) --> double", pybind11::arg("x"), pybind11::arg("mu"), pybind11::arg("std"));

	// mrpt::math::normalQuantile(double) file:mrpt/math/distributions.h line:128
	M("mrpt::math").def("normalQuantile", (double (*)(double)) &mrpt::math::normalQuantile, "Evaluates the Gaussian distribution quantile for the probability value\n p=[0,1].\n  The employed approximation is that from Peter J. Acklam\n (pjacklam.no),\n  freely available in http://home.online.no/~pjacklam.\n\nC++: mrpt::math::normalQuantile(double) --> double", pybind11::arg("p"));

	// mrpt::math::normalCDF(double) file:mrpt/math/distributions.h line:135
	M("mrpt::math").def("normalCDF", (double (*)(double)) &mrpt::math::normalCDF, "Evaluates the Gaussian cumulative density function.\n  The employed approximation is that from W. J. Cody\n  freely available in http://www.netlib.org/specfun/erf\n  \n\n Equivalent to MATLAB normcdf(x,mu,s) with p=(x-mu)/s\n\nC++: mrpt::math::normalCDF(double) --> double", pybind11::arg("p"));

	// mrpt::math::chi2inv(double, unsigned int) file:mrpt/math/distributions.h line:143
	M("mrpt::math").def("chi2inv", [](double const & a0) -> double { return mrpt::math::chi2inv(a0); }, "", pybind11::arg("P"));
	M("mrpt::math").def("chi2inv", (double (*)(double, unsigned int)) &mrpt::math::chi2inv, "The \"quantile\" of the Chi-Square distribution, for dimension \"dim\" and\n probability 0<P<1 (the inverse of chi2CDF)\n An aproximation from the Wilson-Hilferty transformation is used.\n  \n\n Equivalent to MATLAB chi2inv(), but note that this is just an\n approximation, which becomes very poor for small values of \"P\".\n\nC++: mrpt::math::chi2inv(double, unsigned int) --> double", pybind11::arg("P"), pybind11::arg("dim"));

	// mrpt::math::noncentralChi2CDF(unsigned int, double, double) file:mrpt/math/distributions.h line:165
	M("mrpt::math").def("noncentralChi2CDF", (double (*)(unsigned int, double, double)) &mrpt::math::noncentralChi2CDF, "Cumulative non-central chi square distribution (approximate).\n\n  Computes approximate values of the cumulative density of a chi square\ndistribution with \n  and noncentrality parameter  at the given argument\n   i.e. the probability that a random number drawn from the\ndistribution is below \n  It uses the approximate transform into a normal distribution due to Wilson\nand Hilferty\n  (see Abramovitz, Stegun: \"Handbook of Mathematical Functions\", formula\n26.3.32).\n  The algorithm's running time is independent of the inputs. The accuracy is\nonly\n  about 0.1 for few degrees of freedom, but reaches about 0.001 above dof = 5.\n\n  \n Function code from the Vigra project\n(http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\nGPL-compatible.\n \n\n noncentralChi2PDF_CDF\n\nC++: mrpt::math::noncentralChi2CDF(unsigned int, double, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("noncentrality"), pybind11::arg("arg"));

	// mrpt::math::chi2CDF(unsigned int, double) file:mrpt/math/distributions.h line:180
	M("mrpt::math").def("chi2CDF", (double (*)(unsigned int, double)) &mrpt::math::chi2CDF, "Cumulative chi square distribution.\n\n  Computes the cumulative density of a chi square distribution with \n  and tolerance  at the given argument  i.e. the probability\n   that\n  a random number drawn from the distribution is below \n  by calling noncentralChi2CDF(degreesOfFreedom, 0.0, arg, accuracy).\n\n  \n Function code from the Vigra project\n   (http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\n   GPL-compatible.\n\nC++: mrpt::math::chi2CDF(unsigned int, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"));

	// mrpt::math::chi2PDF(unsigned int, double, double) file:mrpt/math/distributions.h line:192
	M("mrpt::math").def("chi2PDF", [](unsigned int const & a0, double const & a1) -> double { return mrpt::math::chi2PDF(a0, a1); }, "", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"));
	M("mrpt::math").def("chi2PDF", (double (*)(unsigned int, double, double)) &mrpt::math::chi2PDF, "Chi square distribution PDF.\n	Computes the density of a chi square distribution with \n	and tolerance  at the given argument \n	by calling noncentralChi2(degreesOfFreedom, 0.0, arg, accuracy).\n	\n\n Function code from the Vigra project\n(http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\nGPL-compatible.\n\n \n Equivalent to MATLAB's chi2pdf(arg,degreesOfFreedom)\n\nC++: mrpt::math::chi2PDF(unsigned int, double, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"), pybind11::arg("accuracy"));

	// mrpt::math::noncentralChi2PDF_CDF(unsigned int, double, double, double) file:mrpt/math/distributions.h line:198
	M("mrpt::math").def("noncentralChi2PDF_CDF", [](unsigned int const & a0, double const & a1, double const & a2) -> std::pair<double, double> { return mrpt::math::noncentralChi2PDF_CDF(a0, a1, a2); }, "", pybind11::arg("degreesOfFreedom"), pybind11::arg("noncentrality"), pybind11::arg("arg"));
	M("mrpt::math").def("noncentralChi2PDF_CDF", (struct std::pair<double, double> (*)(unsigned int, double, double, double)) &mrpt::math::noncentralChi2PDF_CDF, "Returns the 'exact' PDF (first) and CDF (second) of a Non-central\n chi-squared probability distribution, using an iterative method.\n \n\n Equivalent to MATLAB's ncx2cdf(arg,degreesOfFreedom,noncentrality)\n\nC++: mrpt::math::noncentralChi2PDF_CDF(unsigned int, double, double, double) --> struct std::pair<double, double>", pybind11::arg("degreesOfFreedom"), pybind11::arg("noncentrality"), pybind11::arg("arg"), pybind11::arg("eps"));

	{ // mrpt::math::LowPassFilter_IIR1 file:mrpt/math/filters.h line:24
		pybind11::class_<mrpt::math::LowPassFilter_IIR1, std::shared_ptr<mrpt::math::LowPassFilter_IIR1>> cl(M("mrpt::math"), "LowPassFilter_IIR1", "1-order low-pass IIR filter.\n Discrete time equation: `y[k]=alpha*y[k-1]+(1-alpha)*x[k]`.\n With: x[k] input, y[k] output, alpha a parameter in [0,1]");
		cl.def( pybind11::init( [](){ return new mrpt::math::LowPassFilter_IIR1(); } ), "doc" );
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::math::LowPassFilter_IIR1(a0); } ), "doc" , pybind11::arg("alpha"));
		cl.def( pybind11::init<double, double>(), pybind11::arg("alpha"), pybind11::arg("y_k_minus_1") );

		cl.def( pybind11::init( [](mrpt::math::LowPassFilter_IIR1 const &o){ return new mrpt::math::LowPassFilter_IIR1(o); } ) );
		cl.def_readwrite("alpha", &mrpt::math::LowPassFilter_IIR1::alpha);
		cl.def("filter", (double (mrpt::math::LowPassFilter_IIR1::*)(double)) &mrpt::math::LowPassFilter_IIR1::filter, "Processes one input sample, updates the filter state and return the\n filtered value. \n\nC++: mrpt::math::LowPassFilter_IIR1::filter(double) --> double", pybind11::arg("x"));
		cl.def("getLastOutput", (double (mrpt::math::LowPassFilter_IIR1::*)() const) &mrpt::math::LowPassFilter_IIR1::getLastOutput, "C++: mrpt::math::LowPassFilter_IIR1::getLastOutput() const --> double");
	}
	// mrpt::math::fft_real(class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &) file:mrpt/math/fourier.h line:24
	M("mrpt::math").def("fft_real", (void (*)(class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &)) &mrpt::math::fft_real, "Computes the FFT of a 2^N-size vector of real numbers, and returns the\n Re+Im+Magnitude parts.\n \n\n fft2_real\n\nC++: mrpt::math::fft_real(class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &, class mrpt::math::CVectorDynamic<float> &) --> void", pybind11::arg("in_realData"), pybind11::arg("out_FFT_Re"), pybind11::arg("out_FFT_Im"), pybind11::arg("out_FFT_Mag"));

	// mrpt::math::dft2_real(const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) file:mrpt/math/fourier.h line:41
	M("mrpt::math").def("dft2_real", (void (*)(const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::dft2_real, "Compute the 2D Discrete Fourier Transform (DFT) of a real matrix, returning\n the real and imaginary parts separately.\n \n\n The N_1xN_2 matrix.\n \n\n The N_1xN_2 output matrix which will store the real values\n (user has not to initialize the size of this matrix).\n \n\n The N_1xN_2 output matrix which will store the imaginary\n values (user has not to initialize the size of this matrix).\n \n\n fft_real, ifft2_read, fft2_complex\n  If the dimensions of the matrix are powers of two, the fast fourier\n transform (FFT) is used instead of the general algorithm.\n\nC++: mrpt::math::dft2_real(const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_data"), pybind11::arg("out_real"), pybind11::arg("out_imag"));

	// mrpt::math::idft2_real(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) file:mrpt/math/fourier.h line:55
	M("mrpt::math").def("idft2_real", (void (*)(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::idft2_real, "Compute the 2D inverse Discrete Fourier Transform (DFT)\n \n\n The N_1xN_2 input matrix with real values.\n \n\n The N_1xN_2 input matrix with imaginary values.\n \n\n The N_1xN_2 output matrix (user has not to initialize the\n size of this matrix).\n  Note that the real and imaginary parts of the FFT will NOT be checked to\n assure that they represent the transformation\n    of purely real data.\n  If the dimensions of the matrix are powers of two, the fast fourier\n transform (FFT) is used instead of the general algorithm.\n \n\n fft_real, fft2_real\n\nC++: mrpt::math::idft2_real(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_real"), pybind11::arg("in_imag"), pybind11::arg("out_data"));

	// mrpt::math::dft2_complex(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) file:mrpt/math/fourier.h line:69
	M("mrpt::math").def("dft2_complex", (void (*)(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::dft2_complex, "Compute the 2D Discrete Fourier Transform (DFT) of a complex matrix,\n returning the real and imaginary parts separately.\n \n\n The N_1xN_2 matrix with the real part.\n \n\n The N_1xN_2 matrix with the imaginary part.\n \n\n The N_1xN_2 output matrix which will store the real values\n (user has not to initialize the size of this matrix).\n \n\n The N_1xN_2 output matrix which will store the imaginary\n values (user has not to initialize the size of this matrix).\n  If the dimensions of the matrix are powers of two, the fast fourier\n transform (FFT) is used instead of the general algorithm.\n \n\n fft_real, idft2_complex,dft2_real\n\nC++: mrpt::math::dft2_complex(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_real"), pybind11::arg("in_imag"), pybind11::arg("out_real"), pybind11::arg("out_imag"));

	// mrpt::math::idft2_complex(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) file:mrpt/math/fourier.h line:88
	M("mrpt::math").def("idft2_complex", (void (*)(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::idft2_complex, "Compute the 2D inverse Discrete Fourier Transform (DFT).\n \n\n The N_1xN_2 input matrix with real values, where both\n dimensions MUST BE powers of 2.\n \n\n The N_1xN_2 input matrix with imaginary values, where both\n dimensions MUST BE powers of 2.\n \n\n The N_1xN_2 output matrix for real part (user has not to\n initialize the size of this matrix).\n \n\n The N_1xN_2 output matrix for imaginary part (user has not\n to initialize the size of this matrix).\n \n\n fft_real, dft2_real,dft2_complex\n  If the dimensions of the matrix are powers of two, the fast fourier\n transform (FFT) is used instead of the general algorithm.\n\nC++: mrpt::math::idft2_complex(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_real"), pybind11::arg("in_imag"), pybind11::arg("out_real"), pybind11::arg("out_imag"));

	// mrpt::math::cross_correlation_FFT(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) file:mrpt/math/fourier.h line:96
	M("mrpt::math").def("cross_correlation_FFT", (void (*)(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::cross_correlation_FFT, "Correlation of two matrixes using 2D FFT\n\nC++: mrpt::math::cross_correlation_FFT(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("A"), pybind11::arg("B"), pybind11::arg("out_corr"));

}
