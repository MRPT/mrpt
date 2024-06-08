#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/fresnel.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/math/model_search.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/math/robust_kernels.h>
#include <mrpt/math/slerp.h>
#include <mrpt/math/utils.h>
#include <optional>
#include <sstream> // __str__
#include <string>

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

void bind_mrpt_math_fresnel(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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

	// mrpt::math::solve_poly2(double, double, double, double &, double &) file:mrpt/math/poly_roots.h line:63
	M("mrpt::math").def("solve_poly2", (int (*)(double, double, double, double &, double &)) &mrpt::math::solve_poly2, "Solves equation `a*x^2 + b*x + c = 0`.\n Returns the number of real roots: either 0 or 2; or 1 if a=0 (in this case\n the root is in r1).\n r1, r2 are the roots. (r1<=r2)\n \n\n Based on `poly34.h`, by Khashin S.I.\n http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com\n\nC++: mrpt::math::solve_poly2(double, double, double, double &, double &) --> int", pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"), pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::TRobustKernelType file:mrpt/math/robust_kernels.h line:23
	pybind11::enum_<mrpt::math::TRobustKernelType>(M("mrpt::math"), "TRobustKernelType", pybind11::arithmetic(), "The different types of kernels for usage within a robustified least-squares\n estimator.\n \n\n Use these types as arguments of the template RobustKernel<>")
		.value("rkLeastSquares", mrpt::math::rkLeastSquares)
		.value("rkPseudoHuber", mrpt::math::rkPseudoHuber)
		.export_values();

;

	// mrpt::math::slerp(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) file:mrpt/math/slerp.h line:80
	M("mrpt::math").def("slerp", (void (*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &)) &mrpt::math::slerp, "SLERP interpolation between two 6D poses - like mrpt::math::slerp for\n quaternions, but interpolates the [X,Y,Z] coordinates as well.\n \n\n The pose for t=0\n \n\n The pose for t=1\n \n\n  A \"time\" parameter, in the range [0,1].\n \n\n The output, interpolated pose.\n \n\n std::exception Only in Debug, if t is not in the valid range.\n\nC++: mrpt::math::slerp(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) --> void", pybind11::arg("q0"), pybind11::arg("q1"), pybind11::arg("t"), pybind11::arg("p"));

	// mrpt::math::slerp_ypr(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) file:mrpt/math/slerp.h line:86
	M("mrpt::math").def("slerp_ypr", (void (*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &)) &mrpt::math::slerp_ypr, "as mrpt::math::TPose3D\n form as yaw,pitch,roll angles. XYZ are ignored.\n\nC++: mrpt::math::slerp_ypr(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) --> void", pybind11::arg("q0"), pybind11::arg("q1"), pybind11::arg("t"), pybind11::arg("p"));

	// mrpt::math::factorial64(unsigned int) file:mrpt/math/utils.h line:163
	M("mrpt::math").def("factorial64", (uint64_t (*)(unsigned int)) &mrpt::math::factorial64, "Computes the factorial of an integer number and returns it as a 64-bit\n integer number.\n\nC++: mrpt::math::factorial64(unsigned int) --> uint64_t", pybind11::arg("n"));

	// mrpt::math::factorial(unsigned int) file:mrpt/math/utils.h line:168
	M("mrpt::math").def("factorial", (double (*)(unsigned int)) &mrpt::math::factorial, "Computes the factorial of an integer number and returns it as a double value\n (internally it uses logarithms for avoiding overflow).\n\nC++: mrpt::math::factorial(unsigned int) --> double", pybind11::arg("n"));

	// mrpt::math::MATLAB_plotCovariance2D(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &, float, const std::string &, size_t) file:mrpt/math/utils.h line:180
	M("mrpt::math").def("MATLAB_plotCovariance2D", [](const class mrpt::math::CMatrixDynamic<float> & a0, const class mrpt::math::CVectorDynamic<float> & a1, float const & a2) -> std::string { return mrpt::math::MATLAB_plotCovariance2D(a0, a1, a2); }, "", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"));
	M("mrpt::math").def("MATLAB_plotCovariance2D", [](const class mrpt::math::CMatrixDynamic<float> & a0, const class mrpt::math::CVectorDynamic<float> & a1, float const & a2, const std::string & a3) -> std::string { return mrpt::math::MATLAB_plotCovariance2D(a0, a1, a2, a3); }, "", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"), pybind11::arg("style"));
	M("mrpt::math").def("MATLAB_plotCovariance2D", (std::string (*)(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &, float, const std::string &, size_t)) &mrpt::math::MATLAB_plotCovariance2D, "Generates a string with the MATLAB commands required to plot an confidence\n interval (ellipse) for a 2D Gaussian ('float' version)..\n  \n\n The 2x2 covariance matrix\n  \n\n  The 2-length vector with the mean\n  \n\n How many \"quantiles\" to get into the area of the ellipse:\n 2: 95%, 3:99.97%,...\n  \n\n A matlab style string, for colors, line styles,...\n  \n\n The number of points in the ellipse to generate\n \n\n\n \n\nC++: mrpt::math::MATLAB_plotCovariance2D(const class mrpt::math::CMatrixDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &, float, const std::string &, size_t) --> std::string", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"), pybind11::arg("style"), pybind11::arg("nEllipsePoints"));

	// mrpt::math::MATLAB_plotCovariance2D(const class mrpt::math::CMatrixDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &, float, const std::string &, size_t) file:mrpt/math/utils.h line:197
	M("mrpt::math").def("MATLAB_plotCovariance2D", [](const class mrpt::math::CMatrixDynamic<double> & a0, const class mrpt::math::CVectorDynamic<double> & a1, float const & a2) -> std::string { return mrpt::math::MATLAB_plotCovariance2D(a0, a1, a2); }, "", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"));
	M("mrpt::math").def("MATLAB_plotCovariance2D", [](const class mrpt::math::CMatrixDynamic<double> & a0, const class mrpt::math::CVectorDynamic<double> & a1, float const & a2, const std::string & a3) -> std::string { return mrpt::math::MATLAB_plotCovariance2D(a0, a1, a2, a3); }, "", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"), pybind11::arg("style"));
	M("mrpt::math").def("MATLAB_plotCovariance2D", (std::string (*)(const class mrpt::math::CMatrixDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &, float, const std::string &, size_t)) &mrpt::math::MATLAB_plotCovariance2D, "Generates a string with the MATLAB commands required to plot an confidence\n interval (ellipse) for a 2D Gaussian ('double' version).\n  \n\n The 2x2 covariance matrix\n  \n\n  The 2-length vector with the mean\n  \n\n How many \"quantiles\" to get into the area of the ellipse:\n 2: 95%, 3:99.97%,...\n  \n\n A matlab style string, for colors, line styles,...\n  \n\n The number of points in the ellipse to generate\n \n\n\n \n\nC++: mrpt::math::MATLAB_plotCovariance2D(const class mrpt::math::CMatrixDynamic<double> &, const class mrpt::math::CVectorDynamic<double> &, float, const std::string &, size_t) --> std::string", pybind11::arg("cov22"), pybind11::arg("mean"), pybind11::arg("stdCount"), pybind11::arg("style"), pybind11::arg("nEllipsePoints"));

}
