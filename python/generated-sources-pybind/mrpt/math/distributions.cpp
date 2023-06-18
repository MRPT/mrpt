#include <iterator>
#include <memory>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/filters.h>
#include <mrpt/math/fresnel.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/model_search.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/math/robust_kernels.h>
#include <mrpt/math/slerp.h>
#include <sstream> // __str__
#include <string>
#include <utility>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_math_distributions(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::normalPDF(double, double, double) file:mrpt/math/distributions.h line:25
	M("mrpt::math").def("normalPDF", (double (*)(double, double, double)) &mrpt::math::normalPDF, "Evaluates the univariate normal (Gaussian) distribution at a given point\n \"x\".\n\nC++: mrpt::math::normalPDF(double, double, double) --> double", pybind11::arg("x"), pybind11::arg("mu"), pybind11::arg("std"));

	// mrpt::math::normalQuantile(double) file:mrpt/math/distributions.h line:132
	M("mrpt::math").def("normalQuantile", (double (*)(double)) &mrpt::math::normalQuantile, "Evaluates the Gaussian distribution quantile for the probability value\n p=[0,1].\n  The employed approximation is that from Peter J. Acklam\n (pjacklam.no),\n  freely available in http://home.online.no/~pjacklam.\n\nC++: mrpt::math::normalQuantile(double) --> double", pybind11::arg("p"));

	// mrpt::math::normalCDF(double) file:mrpt/math/distributions.h line:139
	M("mrpt::math").def("normalCDF", (double (*)(double)) &mrpt::math::normalCDF, "Evaluates the Gaussian cumulative density function.\n  The employed approximation is that from W. J. Cody\n  freely available in http://www.netlib.org/specfun/erf\n  \n\n Equivalent to MATLAB normcdf(x,mu,s) with p=(x-mu)/s\n\nC++: mrpt::math::normalCDF(double) --> double", pybind11::arg("p"));

	// mrpt::math::chi2inv(double, unsigned int) file:mrpt/math/distributions.h line:147
	M("mrpt::math").def("chi2inv", [](double const & a0) -> double { return mrpt::math::chi2inv(a0); }, "", pybind11::arg("P"));
	M("mrpt::math").def("chi2inv", (double (*)(double, unsigned int)) &mrpt::math::chi2inv, "The \"quantile\" of the Chi-Square distribution, for dimension \"dim\" and\n probability 0<P<1 (the inverse of chi2CDF)\n An aproximation from the Wilson-Hilferty transformation is used.\n  \n\n Equivalent to MATLAB chi2inv(), but note that this is just an\n approximation, which becomes very poor for small values of \"P\".\n\nC++: mrpt::math::chi2inv(double, unsigned int) --> double", pybind11::arg("P"), pybind11::arg("dim"));

	// mrpt::math::noncentralChi2CDF(unsigned int, double, double) file:mrpt/math/distributions.h line:169
	M("mrpt::math").def("noncentralChi2CDF", (double (*)(unsigned int, double, double)) &mrpt::math::noncentralChi2CDF, "Cumulative non-central chi square distribution (approximate).\n\n	Computes approximate values of the cumulative density of a chi square\ndistribution with \n	and noncentrality parameter  at the given argument\n	 i.e. the probability that a random number drawn from the\ndistribution is below \n	It uses the approximate transform into a normal distribution due to Wilson\nand Hilferty\n	(see Abramovitz, Stegun: \"Handbook of Mathematical Functions\", formula\n26.3.32).\n	The algorithm's running time is independent of the inputs. The accuracy is\nonly\n	about 0.1 for few degrees of freedom, but reaches about 0.001 above dof = 5.\n\n	\n Function code from the Vigra project\n(http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\nGPL-compatible.\n \n\n noncentralChi2PDF_CDF\n\nC++: mrpt::math::noncentralChi2CDF(unsigned int, double, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("noncentrality"), pybind11::arg("arg"));

	// mrpt::math::chi2CDF(unsigned int, double) file:mrpt/math/distributions.h line:185
	M("mrpt::math").def("chi2CDF", (double (*)(unsigned int, double)) &mrpt::math::chi2CDF, "Cumulative chi square distribution.\n\n	Computes the cumulative density of a chi square distribution with \n	and tolerance  at the given argument  i.e. the probability\n   that\n	a random number drawn from the distribution is below \n	by calling noncentralChi2CDF(degreesOfFreedom, 0.0, arg, accuracy).\n\n	\n Function code from the Vigra project\n   (http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\n   GPL-compatible.\n\nC++: mrpt::math::chi2CDF(unsigned int, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"));

	// mrpt::math::chi2PDF(unsigned int, double, double) file:mrpt/math/distributions.h line:197
	M("mrpt::math").def("chi2PDF", [](unsigned int const & a0, double const & a1) -> double { return mrpt::math::chi2PDF(a0, a1); }, "", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"));
	M("mrpt::math").def("chi2PDF", (double (*)(unsigned int, double, double)) &mrpt::math::chi2PDF, "Chi square distribution PDF.\n	Computes the density of a chi square distribution with \n	and tolerance  at the given argument \n	by calling noncentralChi2(degreesOfFreedom, 0.0, arg, accuracy).\n	\n\n Function code from the Vigra project\n(http://hci.iwr.uni-heidelberg.de/vigra/); code under \"MIT X11 License\", GNU\nGPL-compatible.\n\n \n Equivalent to MATLAB's chi2pdf(arg,degreesOfFreedom)\n\nC++: mrpt::math::chi2PDF(unsigned int, double, double) --> double", pybind11::arg("degreesOfFreedom"), pybind11::arg("arg"), pybind11::arg("accuracy"));

	// mrpt::math::noncentralChi2PDF_CDF(unsigned int, double, double, double) file:mrpt/math/distributions.h line:204
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
	// mrpt::math::fresnel_sin_integral(double) file:mrpt/math/fresnel.h line:25
	M("mrpt::math").def("fresnel_sin_integral", (double (*)(double)) &mrpt::math::fresnel_sin_integral, "Evaluates the integral from 0 to x of sqrt(2/pi) sin(t^2) dt. Equivalent to\n MATLAB fresnels()\n \n\n https://en.wikipedia.org/wiki/Fresnel_integral\n \n\n Code based on\n http://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html \n\nC++: mrpt::math::fresnel_sin_integral(double) --> double", pybind11::arg("x"));

	// mrpt::math::fresnel_cos_integral(double) file:mrpt/math/fresnel.h line:32
	M("mrpt::math").def("fresnel_cos_integral", (double (*)(double)) &mrpt::math::fresnel_cos_integral, "Evaluates the integral from 0 to x of sqrt(2/pi) cos(t^2) dt. Equivalent to\nMATLAB fresnelc()\n \n\n https://en.wikipedia.org/wiki/Fresnel_integral\n\n Code based on\nhttp://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html \n\nC++: mrpt::math::fresnel_cos_integral(double) --> double", pybind11::arg("x"));

	// mrpt::math::lfresnel_sin_integral(long double) file:mrpt/math/fresnel.h line:35
	M("mrpt::math").def("lfresnel_sin_integral", (long double (*)(long double)) &mrpt::math::lfresnel_sin_integral, "long double version of fresnel_sin_integral \n\nC++: mrpt::math::lfresnel_sin_integral(long double) --> long double", pybind11::arg("x"));

	// mrpt::math::lfresnel_cos_integral(long double) file:mrpt/math/fresnel.h line:38
	M("mrpt::math").def("lfresnel_cos_integral", (long double (*)(long double)) &mrpt::math::lfresnel_cos_integral, "long double version of fresnel_cos_integral \n\nC++: mrpt::math::lfresnel_cos_integral(long double) --> long double", pybind11::arg("x"));

	// mrpt::math::interpolate2points(const double, const double, const double, const double, const double, bool) file:mrpt/math/interp_fit.h line:36
	M("mrpt::math").def("interpolate2points", [](const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> double { return mrpt::math::interpolate2points(a0, a1, a2, a3, a4); }, "", pybind11::arg("x"), pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"));
	M("mrpt::math").def("interpolate2points", (double (*)(const double, const double, const double, const double, const double, bool)) &mrpt::math::interpolate2points, "Linear interpolation/extrapolation: evaluates at \"x\" the line\n (x0,y0)-(x1,y1).\n  If wrap2pi is true, output is wrapped to ]-pi,pi] (It is assumed that input\n \"y\" values already are in the correct range).\n \n\n spline, interpolate, leastSquareLinearFit\n\nC++: mrpt::math::interpolate2points(const double, const double, const double, const double, const double, bool) --> double", pybind11::arg("x"), pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("wrap2pi"));

	{ // mrpt::math::ModelSearch file:mrpt/math/model_search.h line:65
		pybind11::class_<mrpt::math::ModelSearch, std::shared_ptr<mrpt::math::ModelSearch>> cl(M("mrpt::math"), "ModelSearch", "Model search implementations: RANSAC and genetic algorithm\n\n  The type   is a user-supplied struct/class that implements this\n interface:\n  - Types:\n    -  : The numeric type to use (typ: double, float)\n    -  : The type of the model to be fitted (for example: A matrix, a\n TLine2D, a TPlane3D, ...)\n  - Methods:\n    - size_t getSampleCount() const : return the number of samples. This\n should not change during a model search.\n    - bool fitModel( const std::vector<size_t>& useIndices, Model& model )\n const :\n This function fits a model to the data selected by the indices. The return\n value indicates the success, hence false means a degenerate case, where no\n model was found.\n    - Real testSample( size_t index, const Model& model ) const : return some\n value that indicates how well a sample fits to the model. This way the\n thresholding is moved to the searching procedure and the model just tells how\n good a sample is.\n\n  There are two methods provided in this class to fit a model:\n    -  (RANSAC): Just like mrpt::math::RANSAC_Template\n\n    -  (Genetic): Provides a mixture of a genetic and\n the ransac algorithm.\n         Instead of selecting a set of data in each iteration, it takes more\n (ex. 10) and order these model\n         using some fitness function: the average error of the inliers scaled\n by the number of outliers (This\n         fitness might require some fine tuning). Than the (ex 10) new kernel\n for the next iteration is created as follows:\n             - Take the best kernels (as for the original ransac)\n             - Select two kernels ( with a higher probability for the better\n models) and let the new kernel be a subset of the two original kernels (\n additionally to leave the local minimums an additional random seed might\n appear - mutation)\n             - Generate some new random samples.\n\n  For an example of usage, see \"samples/model_search_test/\"\n  \n\n mrpt::math::RANSAC_Template, another RANSAC implementation where models\n can be matrices only.\n\n  \n Zoltar Gaal\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::ModelSearch(); } ) );
	}
	// mrpt::math::solve_poly3(double *, double, double, double) file:mrpt/math/poly_roots.h line:29
	M("mrpt::math").def("solve_poly3", (int (*)(double *, double, double, double)) &mrpt::math::solve_poly3, "Solves cubic equation `x^3 + a*x^2 + b*x + c = 0`. Returns the number of\n real roots `N`<=3.\n The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,\n `x[0]` and `x[1]` if N=2, etc.\n \n\n array of size 3\n \n\n Based on `poly34.h`, by Khashin S.I.\n http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com\n\nC++: mrpt::math::solve_poly3(double *, double, double, double) --> int", pybind11::arg("x"), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"));

	// mrpt::math::solve_poly4(double *, double, double, double, double) file:mrpt/math/poly_roots.h line:44
	M("mrpt::math").def("solve_poly4", (int (*)(double *, double, double, double, double)) &mrpt::math::solve_poly4, "Solves quartic equation `x^4 + a*x^3 + b*x^2 + c*x + d = 0` by Dekart-Euler\n method.\n Returns the number of real roots `N`<=4:\n - return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots\n - return 2: 2 real roots x[0], x[1] and complex x[2]+-i*x[3],\n - return 0: two pair of complex roots: x[0]+-i*x[1],  x[2]+-i*x[3],\n\n The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,\n `x[0]` and `x[1]` if N=2, etc.\n \n\n array of size 4\n \n\n Based on `poly34.h`, by Khashin S.I.\n http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com\n\nC++: mrpt::math::solve_poly4(double *, double, double, double, double) --> int", pybind11::arg("x"), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"), pybind11::arg("d"));

	// mrpt::math::solve_poly5(double *, double, double, double, double, double) file:mrpt/math/poly_roots.h line:54
	M("mrpt::math").def("solve_poly5", (int (*)(double *, double, double, double, double, double)) &mrpt::math::solve_poly5, "Solves equation `x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0`.\n Returns the number of real roots `N`<=5.\n The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,\n `x[0]` and `x[1]` if N=2, etc.\n \n\n array of size 5\n \n\n Based on `poly34.h`, by Khashin S.I.\n http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com\n\nC++: mrpt::math::solve_poly5(double *, double, double, double, double, double) --> int", pybind11::arg("x"), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"), pybind11::arg("d"), pybind11::arg("e"));

	// mrpt::math::solve_poly2(double, double, double, double &, double &) file:mrpt/math/poly_roots.h line:64
	M("mrpt::math").def("solve_poly2", (int (*)(double, double, double, double &, double &)) &mrpt::math::solve_poly2, "Solves equation `a*x^2 + b*x + c = 0`.\n Returns the number of real roots: either 0 or 2; or 1 if a=0 (in this case\n the root is in r1).\n r1, r2 are the roots. (r1<=r2)\n \n\n Based on `poly34.h`, by Khashin S.I.\n http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com\n\nC++: mrpt::math::solve_poly2(double, double, double, double &, double &) --> int", pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"), pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::TRobustKernelType file:mrpt/math/robust_kernels.h line:23
	pybind11::enum_<mrpt::math::TRobustKernelType>(M("mrpt::math"), "TRobustKernelType", pybind11::arithmetic(), "The different types of kernels for usage within a robustified least-squares\n estimator.\n \n\n Use these types as arguments of the template RobustKernel<>")
		.value("rkLeastSquares", mrpt::math::rkLeastSquares)
		.value("rkPseudoHuber", mrpt::math::rkPseudoHuber)
		.export_values();

;

	// mrpt::math::slerp(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) file:mrpt/math/slerp.h line:87
	M("mrpt::math").def("slerp", (void (*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &)) &mrpt::math::slerp, "SLERP interpolation between two 6D poses - like mrpt::math::slerp for\n quaternions, but interpolates the [X,Y,Z] coordinates as well.\n \n\n The pose for t=0\n \n\n The pose for t=1\n \n\n  A \"time\" parameter, in the range [0,1].\n \n\n The output, interpolated pose.\n \n\n std::exception Only in Debug, if t is not in the valid range.\n\nC++: mrpt::math::slerp(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) --> void", pybind11::arg("q0"), pybind11::arg("q1"), pybind11::arg("t"), pybind11::arg("p"));

}
