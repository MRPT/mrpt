/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/utils.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/interp_fit.hpp>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>

#include <mrpt/system/string_utils.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/ops_matrices.h>
#include <cmath>  // erf(), ...
#include <algorithm>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;
using mrpt::serialization::CArchive;

/*---------------------------------------------------------------
						normalPDF
 ---------------------------------------------------------------*/
double math::normalPDF(double x, double mu, double std)
{
	return ::exp(-0.5 * square((x - mu) / std)) /
		   (std * 2.506628274631000502415765284811);
}

/*---------------------------------------------------------------
						chi2inv
 ---------------------------------------------------------------*/
double math::chi2inv(double P, unsigned int dim)
{
	ASSERT_(P >= 0 && P < 1);
	if (P == 0)
		return 0;
	else
		return dim * pow(1.0 - 2.0 / (9 * dim) +
							 sqrt(2.0 / (9 * dim)) * normalQuantile(P),
						 3);
}

/*---------------------------------------------------------------
						factorial64
 ---------------------------------------------------------------*/
uint64_t math::factorial64(unsigned int n)
{
	uint64_t ret = 1;

	for (unsigned int i = 2; i <= n; i++) ret *= i;

	return ret;
}

/*---------------------------------------------------------------
						factorial
 ---------------------------------------------------------------*/
double math::factorial(unsigned int n)
{
	double retLog = 0;

	for (unsigned int i = 2; i <= n; i++) retLog += ::log((double)n);

	return ::exp(retLog);
}

/*---------------------------------------------------------------
						normalQuantile
 ---------------------------------------------------------------*/
double math::normalQuantile(double p)
{
	double q, t, u;

	static const double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02,
								-2.759285104469687e+02, 1.383577518672690e+02,
								-3.066479806614716e+01, 2.506628277459239e+00};
	static const double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02,
								-1.556989798598866e+02, 6.680131188771972e+01,
								-1.328068155288572e+01};
	static const double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01,
								-2.400758277161838e+00, -2.549732539343734e+00,
								4.374664141464968e+00,  2.938163982698783e+00};
	static const double d[4] = {7.784695709041462e-03, 3.224671290700398e-01,
								2.445134137142996e+00, 3.754408661907416e+00};

	ASSERT_(!std::isnan(p));
	ASSERT_(p < 1.0 && p > 0.0);

	q = min(p, 1 - p);

	if (q > 0.02425)
	{
		/* Rational approximation for central region. */
		u = q - 0.5;
		t = u * u;
		u = u *
			(((((a[0] * t + a[1]) * t + a[2]) * t + a[3]) * t + a[4]) * t +
			 a[5]) /
			(((((b[0] * t + b[1]) * t + b[2]) * t + b[3]) * t + b[4]) * t + 1);
	}
	else
	{
		/* Rational approximation for tail region. */
		t = sqrt(-2 * ::log(q));
		u = (((((c[0] * t + c[1]) * t + c[2]) * t + c[3]) * t + c[4]) * t +
			 c[5]) /
			((((d[0] * t + d[1]) * t + d[2]) * t + d[3]) * t + 1);
	}

	/* The relative error of the approximation has absolute value less
	than 1.15e-9.  One iteration of Halley's rational method (third
	order) gives full machine precision... */
	t = normalCDF(u) - q; /* error */
	t = t * 2.506628274631000502415765284811 *
		::exp(u * u / 2); /* f(u)/df(u) */
	u = u - t / (1 + u * t / 2); /* Halley's method */

	return (p > 0.5 ? -u : u);
}

/*---------------------------------------------------------------
						normalCDF
 ---------------------------------------------------------------*/
double math::normalCDF(double u)
{
	static const double a[5] = {1.161110663653770e-002, 3.951404679838207e-001,
								2.846603853776254e+001, 1.887426188426510e+002,
								3.209377589138469e+003};
	static const double b[5] = {1.767766952966369e-001, 8.344316438579620e+000,
								1.725514762600375e+002, 1.813893686502485e+003,
								8.044716608901563e+003};
	static const double c[9] = {
		2.15311535474403846e-8, 5.64188496988670089e-1, 8.88314979438837594e00,
		6.61191906371416295e01, 2.98635138197400131e02, 8.81952221241769090e02,
		1.71204761263407058e03, 2.05107837782607147e03, 1.23033935479799725E03};
	static const double d[9] = {
		1.00000000000000000e00, 1.57449261107098347e01, 1.17693950891312499e02,
		5.37181101862009858e02, 1.62138957456669019e03, 3.29079923573345963e03,
		4.36261909014324716e03, 3.43936767414372164e03, 1.23033935480374942e03};
	static const double p[6] = {1.63153871373020978e-2, 3.05326634961232344e-1,
								3.60344899949804439e-1, 1.25781726111229246e-1,
								1.60837851487422766e-2, 6.58749161529837803e-4};
	static const double q[6] = {1.00000000000000000e00, 2.56852019228982242e00,
								1.87295284992346047e00, 5.27905102951428412e-1,
								6.05183413124413191e-2, 2.33520497626869185e-3};
	double y, z;

	ASSERT_(!std::isnan(u));
	ASSERT_(std::isfinite(u));

	y = fabs(u);
	if (y <= 0.46875 * 1.4142135623730950488016887242097)
	{
		/* evaluate erf() for |u| <= sqrt(2)*0.46875 */
		z = y * y;
		y = u * ((((a[0] * z + a[1]) * z + a[2]) * z + a[3]) * z + a[4]) /
			((((b[0] * z + b[1]) * z + b[2]) * z + b[3]) * z + b[4]);
		return 0.5 + y;
	}

	z = ::exp(-y * y / 2) / 2;
	if (y <= 4.0)
	{
		/* evaluate erfc() for sqrt(2)*0.46875 <= |u| <= sqrt(2)*4.0 */
		y = y / 1.4142135623730950488016887242097;
		y = ((((((((c[0] * y + c[1]) * y + c[2]) * y + c[3]) * y + c[4]) * y +
				c[5]) *
				   y +
			   c[6]) *
				  y +
			  c[7]) *
				 y +
			 c[8])

			/ ((((((((d[0] * y + d[1]) * y + d[2]) * y + d[3]) * y + d[4]) * y +
				  d[5]) *
					 y +
				 d[6]) *
					y +
				d[7]) *
				   y +
			   d[8]);

		y = z * y;
	}
	else
	{
		/* evaluate erfc() for |u| > sqrt(2)*4.0 */
		z = z * 1.4142135623730950488016887242097 / y;
		y = 2 / (y * y);
		y = y *
			(((((p[0] * y + p[1]) * y + p[2]) * y + p[3]) * y + p[4]) * y +
			 p[5]) /
			(((((q[0] * y + q[1]) * y + q[2]) * y + q[3]) * y + q[4]) * y +
			 q[5]);
		y = z * (0.564189583547756286948 - y);
	}
	return (u < 0.0 ? y : 1 - y);
}

// Loads a vector from a text file:
bool math::loadVector(std::istream& f, ::std::vector<int>& d)
{
	MRPT_START

	std::string str;
	if (!std::getline(f, str)) return false;

	const char* s = str.c_str();

	char *nextTok, *context;
	const char* delim = " \t";

	d.clear();
	nextTok = mrpt::system::strtok((char*)s, delim, &context);
	while (nextTok != nullptr)
	{
		d.push_back(atoi(nextTok));
		nextTok = mrpt::system::strtok(nullptr, delim, &context);
	};

	return true;
	MRPT_END
}

bool math::loadVector(std::istream& f, ::std::vector<double>& d)
{
	MRPT_START

	std::string str;
	if (!std::getline(f, str)) return false;

	const char* s = str.c_str();

	char *nextTok, *context;
	const char* delim = " \t";

	d.clear();
	nextTok = mrpt::system::strtok((char*)s, delim, &context);
	while (nextTok != nullptr)
	{
		d.push_back(atof(nextTok));
		nextTok = mrpt::system::strtok(nullptr, delim, &context);
	};

	return true;
	MRPT_END
}

// See declaration for the documentation
double math::averageLogLikelihood(
	const CVectorDouble& logWeights, const CVectorDouble& logLikelihoods)
{
	MRPT_START

	// Explained in:
	// http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability
	ASSERT_(logWeights.size() == logLikelihoods.size());

	if (!logWeights.size())
		THROW_EXCEPTION("ERROR: logWeights vector is empty!");

	CVectorDouble::const_iterator itLW, itLL;
	double lw_max = math::maximum(logWeights);
	double ll_max = math::maximum(logLikelihoods);
	double SUM1 = 0, SUM2 = 0, tmpVal;

	for (itLW = logWeights.begin(), itLL = logLikelihoods.begin();
		 itLW != logWeights.end(); itLW++, itLL++)
	{
		tmpVal = *itLW - lw_max;
		SUM1 += std::exp(tmpVal);
		SUM2 += std::exp(tmpVal + *itLL - ll_max);
	}

	double res = -std::log(SUM1) + std::log(SUM2) + ll_max;
	MRPT_CHECK_NORMAL_NUMBER(res);
	return res;
	MRPT_END
}

// Unweighted version:
double math::averageLogLikelihood(const CVectorDouble& logLikelihoods)
{
	MRPT_START

	// Explained in:
	// http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability
	size_t N = logLikelihoods.size();
	if (!N) THROW_EXCEPTION("ERROR: logLikelihoods vector is empty!");

	double ll_max = math::maximum(logLikelihoods);
	double SUM1 = 0;

	for (size_t i = 0; i < N; i++) SUM1 += exp(logLikelihoods[i] - ll_max);

	double res = log(SUM1) - log(static_cast<double>(N)) + ll_max;

	MRPT_CHECK_NORMAL_NUMBER(res);
	return res;
	MRPT_END
}

// Wrapped angles average:
double math::averageWrap2Pi(const CVectorDouble& angles)
{
	if (angles.empty()) return 0;

	int W_phi_R = 0, W_phi_L = 0;
	double phi_R = 0, phi_L = 0;

	// First: XY
	// -----------------------------------
	for (CVectorDouble::Index i = 0; i < angles.size(); i++)
	{
		double phi = angles[i];
		if (abs(phi) > 1.5707963267948966192313216916398)
		{
			// LEFT HALF: 0,2pi
			if (phi < 0) phi = (M_2PI + phi);

			phi_L += phi;
			W_phi_L++;
		}
		else
		{
			// RIGHT HALF: -pi,pi
			phi_R += phi;
			W_phi_R++;
		}
	}

	// Next: PHI
	// -----------------------------------
	// The mean value from each side:
	if (W_phi_L) phi_L /= static_cast<double>(W_phi_L);  // [0,2pi]
	if (W_phi_R) phi_R /= static_cast<double>(W_phi_R);  // [-pi,pi]

	// Left side to [-pi,pi] again:
	if (phi_L > M_PI) phi_L -= M_2PI;

	// The total mean:
	return ((phi_L * W_phi_L + phi_R * W_phi_R) / (W_phi_L + W_phi_R));
}

string math::MATLAB_plotCovariance2D(
	const CMatrixFloat& cov, const CVectorFloat& mean, const float& stdCount,
	const string& style, const size_t& nEllipsePoints)
{
	MRPT_START
	CMatrixD cov2(cov);
	CVectorDouble mean2(2);
	mean2[0] = mean[0];
	mean2[1] = mean[1];

	return MATLAB_plotCovariance2D(
		cov2, mean2, stdCount, style, nEllipsePoints);

	MRPT_END
}

string math::MATLAB_plotCovariance2D(
	const CMatrixDouble& cov, const CVectorDouble& mean, const float& stdCount,
	const string& style, const size_t& nEllipsePoints)
{
	MRPT_START

	ASSERT_(cov.cols() == cov.rows() && cov.cols() == 2);
	ASSERT_(cov(0, 1) == cov(1, 0));
	ASSERT_(!((cov(0, 0) == 0) ^ (cov(1, 1) == 0)));  // Both or none 0
	ASSERT_(mean.size() == 2);

	std::vector<float> X, Y, COS, SIN;
	std::vector<float>::iterator x, y, Cos, Sin;
	double ang;
	CMatrixD eigVal, eigVec, M;
	string str;

	X.resize(nEllipsePoints);
	Y.resize(nEllipsePoints);
	COS.resize(nEllipsePoints);
	SIN.resize(nEllipsePoints);

	// Fill the angles:
	for (Cos = COS.begin(), Sin = SIN.begin(), ang = 0; Cos != COS.end();
		 ++Cos, ++Sin, ang += (M_2PI / (nEllipsePoints - 1)))
	{
		*Cos = (float)cos(ang);
		*Sin = (float)sin(ang);
	}

	cov.eigenVectors(eigVec, eigVal);
	eigVal = eigVal.array().sqrt().matrix();
	M = eigVal * eigVec.adjoint();

	// Compute the points of the ellipsoid:
	// ----------------------------------------------
	for (x = X.begin(), y = Y.begin(), Cos = COS.begin(), Sin = SIN.begin();
		 x != X.end(); ++x, ++y, ++Cos, ++Sin)
	{
		*x = (float)(mean[0] + stdCount * (*Cos * M(0, 0) + *Sin * M(1, 0)));
		*y = (float)(mean[1] + stdCount * (*Cos * M(0, 1) + *Sin * M(1, 1)));
	}

	// Save the code to plot the ellipsoid:
	// ----------------------------------------------
	str += string("plot([ ");
	for (x = X.begin(); x != X.end(); ++x)
	{
		str += format("%.4f", *x);
		if (x != (X.end() - 1)) str += format(",");
	}
	str += string("],[ ");
	for (y = Y.begin(); y != Y.end(); ++y)
	{
		str += format("%.4f", *y);
		if (y != (Y.end() - 1)) str += format(",");
	}

	str += format("],'%s');\n", style.c_str());

	return str;
	MRPT_END_WITH_CLEAN_UP(std::cerr << "The matrix that led to error was: "
									 << std::endl
									 << cov << std::endl;)
}

double mrpt::math::interpolate2points(
	const double x, const double x0, const double y0, const double x1,
	const double y1, bool wrap2pi)
{
	MRPT_START
	if (x0 == x1)
		THROW_EXCEPTION_FMT("ERROR: Both x0 and x1 are equal (=%f)", x0);

	const double Ax = x1 - x0;
	const double Ay = y1 - y0;

	double r = y0 + Ay * (x - x0) / Ax;
	if (!wrap2pi)
		return r;
	else
		return mrpt::math::wrapToPi(r);

	MRPT_END
}

/*---------------------------------------------------------------
					median filter of a vector
 ---------------------------------------------------------------*/
// template<typename VECTOR>
void mrpt::math::medianFilter(
	const std::vector<double>& inV, std::vector<double>& outV,
	const int& _winSize, const int& numberOfSigmas)
{
	MRPT_UNUSED_PARAM(numberOfSigmas);
	ASSERT_((int)inV.size() >= _winSize);
	ASSERT_(_winSize >= 2);  // The minimum window size is 3 elements
	size_t winSize = _winSize;

	if (!(winSize % 2))  // We use an odd number of elements for the window size
		winSize++;

	size_t sz = inV.size();
	outV.resize(sz);

	std::vector<double> aux(winSize);
	size_t mpoint = winSize / 2;
	for (size_t k = 0; k < sz; ++k)
	{
		aux.clear();

		size_t idx_to_start =
			std::max(size_t(0), k - mpoint);  // Dealing with the boundaries
		size_t n_elements =
			std::min(std::min(winSize, sz + mpoint - k), k + mpoint + 1);

		aux.resize(n_elements);
		for (size_t m = idx_to_start, n = 0; m < idx_to_start + n_elements;
			 ++m, ++n)
			aux[n] = inV[m];

		std::sort(aux.begin(), aux.end());

		size_t auxSz = aux.size();
		size_t auxMPoint = auxSz / 2;
		outV[k] = (auxSz % 2) ? (aux[auxMPoint])
							  : (0.5 * (aux[auxMPoint - 1] +
										aux[auxMPoint]));  // If the window is
		// even, take the
		// mean value of the
		// middle points
	}  // end-for
}  // end medianFilter

double mrpt::math::chi2CDF(unsigned int degreesOfFreedom, double arg)
{
	return noncentralChi2CDF(degreesOfFreedom, 0.0, arg);
}

template <class T>
void noncentralChi2OneIteration(
	T arg, T& lans, T& dans, T& pans, unsigned int& j)
{
	double tol = -50.0;
	if (lans < tol)
	{
		lans = lans + std::log(arg / j);
		dans = std::exp(lans);
	}
	else
	{
		dans = dans * arg / j;
	}
	pans = pans - dans;
	j += 2;
}

std::pair<double, double> mrpt::math::noncentralChi2PDF_CDF(
	unsigned int degreesOfFreedom, double noncentrality, double arg, double eps)
{
	ASSERTMSG_(
		noncentrality >= 0.0 && arg >= 0.0 && eps > 0.0,
		"noncentralChi2PDF_CDF(): parameters must be positive.");

	if (arg == 0.0 && degreesOfFreedom > 0) return std::make_pair(0.0, 0.0);

	// Determine initial values
	double b1 = 0.5 * noncentrality, ao = std::exp(-b1), eps2 = eps / ao,
		   lnrtpi2 = 0.22579135264473, probability, density, lans, dans, pans,
		   sum, am, hold;
	unsigned int maxit = 500, i, m;
	if (degreesOfFreedom % 2)
	{
		i = 1;
		lans = -0.5 * (arg + std::log(arg)) - lnrtpi2;
		dans = std::exp(lans);
		pans = std::erf(std::sqrt(arg / 2.0));
	}
	else
	{
		i = 2;
		lans = -0.5 * arg;
		dans = std::exp(lans);
		pans = 1.0 - dans;
	}

	// Evaluate first term
	if (degreesOfFreedom == 0)
	{
		m = 1;
		degreesOfFreedom = 2;
		am = b1;
		sum = 1.0 / ao - 1.0 - am;
		density = am * dans;
		probability = 1.0 + am * pans;
	}
	else
	{
		m = 0;
		degreesOfFreedom = degreesOfFreedom - 1;
		am = 1.0;
		sum = 1.0 / ao - 1.0;
		while (i < degreesOfFreedom)
			noncentralChi2OneIteration(arg, lans, dans, pans, i);
		degreesOfFreedom = degreesOfFreedom + 1;
		density = dans;
		probability = pans;
	}
	// Evaluate successive terms of the expansion
	for (++m; m < maxit; ++m)
	{
		am = b1 * am / m;
		noncentralChi2OneIteration(arg, lans, dans, pans, degreesOfFreedom);
		sum = sum - am;
		density = density + am * dans;
		hold = am * pans;
		probability = probability + hold;
		if ((pans * sum < eps2) && (hold < eps2)) break;  // converged
	}
	if (m == maxit) THROW_EXCEPTION("noncentralChi2PDF_CDF(): no convergence.");
	return std::make_pair(
		0.5 * ao * density, std::min(1.0, std::max(0.0, ao * probability)));
}

double mrpt::math::chi2PDF(
	unsigned int degreesOfFreedom, double arg, double accuracy)
{
	return mrpt::math::noncentralChi2PDF_CDF(
			   degreesOfFreedom, 0.0, arg, accuracy)
		.first;
}

double mrpt::math::noncentralChi2CDF(
	unsigned int degreesOfFreedom, double noncentrality, double arg)
{
	const double a = degreesOfFreedom + noncentrality;
	const double b = (a + noncentrality) / square(a);
	const double t =
		(std::pow((double)arg / a, 1.0 / 3.0) - (1.0 - 2.0 / 9.0 * b)) /
		std::sqrt(2.0 / 9.0 * b);
	return 0.5 * (1.0 + std::erf(t / std::sqrt(2.0)));
}

CArchive& mrpt::math::operator>>(CArchive& s, CVectorFloat& v)
{
	v.resize(s.ReadAs<uint32_t>());
	if (v.size() > 0) s.ReadBufferFixEndianness(&v[0], v.size());
	return s;
}
CArchive& mrpt::math::operator>>(CArchive& s, CVectorDouble& v)
{
	v.resize(s.ReadAs<uint32_t>());
	if (v.size() > 0) s.ReadBufferFixEndianness(&v[0], v.size());
	return s;
}
CArchive& mrpt::math::operator<<(CArchive& s, const CVectorFloat& v)
{
	s.WriteAs<uint32_t>(v.size());
	if (v.size() > 0) s.WriteBufferFixEndianness(&v[0], v.size());
	return s;
}
CArchive& mrpt::math::operator<<(CArchive& s, const CVectorDouble& v)
{
	s.WriteAs<uint32_t>(v.size());
	if (v.size() > 0) s.WriteBufferFixEndianness(&v[0], v.size());
	return s;
}
