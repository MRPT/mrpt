/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

const double eps = 1e-12;

TEST(distributions, normalPDF_1d)
{
  EXPECT_NEAR(normalPDF(0, 0, 1), 0.398942280401433, eps);
  EXPECT_NEAR(normalPDF(5, 5, 1), 0.398942280401433, eps);
  EXPECT_NEAR(normalPDF(0, 0, 2), 0.199471140200716, eps);
  EXPECT_NEAR(normalPDF(1, 0, 1), 0.241970724519143, eps);
}

TEST(distributions, normalPDF_vector)
{
  const double cov_vals[3 * 3] = {4.0, 2.0, 1.0, 2.0, 3.0, 0.5, 1.0, 0.5, 1.0};
  const double x1_vals[3] = {1.0, 0.0, 0.0};
  const double x2_vals[3] = {1.0, 2.0, 3.0};

  const CMatrixDouble33 COV(cov_vals);
  const CMatrixFixed<double, 3, 1> x0;
  const CMatrixFixed<double, 3, 1> x1(x1_vals);
  const CMatrixFixed<double, 3, 1> x2(x2_vals);

  EXPECT_NEAR(
      normalPDF(x0, x0, COV), 0.02592116832548877620,
      eps);  // sprintf('%.20f',mvnpdf([0;0;0],[0;0;0],S))
  EXPECT_NEAR(
      normalPDF(x2, x2, COV), 0.02592116832548877620,
      eps);  // sprintf('%.20f',mvnpdf([0;0;0],[0;0;0],S))

  EXPECT_NEAR(
      normalPDF(x1, x0, COV), 0.02061240910323311470,
      eps);  // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
  EXPECT_NEAR(
      normalPDF(x2, x0, COV), 0.00008423820480102986,
      eps);  // sprintf('%.20f',mvnpdf([1;2;3],[0;0;0],S))

  EXPECT_NEAR(
      normalPDF(x1, COV), 0.02061240910323311470,
      eps);  // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
  EXPECT_NEAR(
      normalPDF(x2, COV), 0.00008423820480102986,
      eps);  // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
}

TEST(distributions, erfc)
{
  const double eps2 = 1e-7;

  EXPECT_NEAR(std::erfc(0), 1, eps);
  EXPECT_NEAR(std::erfc(1), 0.157299207050285, eps2);
  EXPECT_NEAR(std::erfc(2), 0.004677734981047, eps2);
}

TEST(distributions, erf)
{
  const double eps2 = 1e-7;

  EXPECT_NEAR(std::erf(0), 0, eps);
  EXPECT_NEAR(std::erf(1), 0.842700792949715, eps2);
  EXPECT_NEAR(std::erf(2), 0.995322265018953, eps2);
}

TEST(distributions, normalCDF)
{
  EXPECT_NEAR(mrpt::math::normalCDF(0), 0.5, eps);
  EXPECT_NEAR(mrpt::math::normalCDF(1), 0.841344746068543, eps);
  EXPECT_NEAR(mrpt::math::normalCDF(2), 0.977249868051821, eps);
  EXPECT_NEAR(mrpt::math::normalCDF(3), 0.998650101968370, eps);
}

TEST(distributions, chi2inv)
{
  EXPECT_NEAR(mrpt::math::chi2inv(0.0, 1), 0, eps);
  EXPECT_NEAR(mrpt::math::chi2inv(0.5, 3), 2.365973884375338, 0.1);
  EXPECT_NEAR(mrpt::math::chi2inv(0.95, 3), 7.814727903251178, 0.1);
}

TEST(distributions, chi2PDF)
{
  EXPECT_NEAR(mrpt::math::chi2PDF(1, 1.0), 0.241970724519143, eps);
  EXPECT_NEAR(mrpt::math::chi2PDF(1, 2.0), 0.103776874355149, eps);
  EXPECT_NEAR(mrpt::math::chi2PDF(1, 3.0), 0.051393443267923, eps);
  EXPECT_NEAR(mrpt::math::chi2PDF(1, 4.0), 0.026995483256594, eps);

  EXPECT_NEAR(mrpt::math::chi2PDF(4, 1.0), 0.151632664928158, eps);
}

TEST(distributions, noncentralChi2PDF_CDF)
{
  const double eps2 = 1e-7;

  // ncx2cdf(arg,degreesOfFreedom,noncentrality)
  // noncentralChi2PDF_CDF(degreesOfFreedom,noncentrality,arg)
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1.0, 0).first, 0, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 2.0, 0).first, 0, eps2);

  // MATLAB: ncx2cdf(1:3,1,1)
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 1.0).second, 0.477249868051821, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 2.0).second, 0.652756536682270, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 3.0).second, 0.764784149631031, eps2);
  // MATLAB: ncx2pdf(1:3,1,1)
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 1.0).first, 0.226466623457311, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 2.0).first, 0.137103272271503, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(1, 1, 3.0).first, 0.090852330823658, eps2);

  // MATLAB: ncx2cdf(1:3,2,3)
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 1.0).second, 0.121825497229364, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 2.0).second, 0.252206942426039, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 3.0).second, 0.378499822919087, eps2);
  // MATLAB: ncx2pdf(1:3,2,3)
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 1.0).first, 0.128765424775546, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 2.0).first, 0.129923687128879, eps2);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3, 3.0).first, 0.121500177080913, eps2);
}

TEST(data_utils, mahalanobisDistanceSqAndLogPDF)
{
  const double cov_vals[3 * 3] = {0.00393682,   -6.11165e-07, -8.62169e-05,
                                  -6.11165e-07, 7.44917e-05,  -1.17274e-07,
                                  -8.62169e-05, -1.17274e-07, 0.000108955};
  const CMatrixDouble33 COV(cov_vals);

  const double x_vals[3] = {0.0135442, 0.00504134, -0.000452334};
  const CMatrixDouble31 x(x_vals);

  double out_maha2, out_ml;
  mrpt::math::mahalanobisDistanceSqAndLogPDF(x, COV, out_maha2, out_ml);

  EXPECT_NEAR(out_maha2, 0.388264, 1e-4);
  EXPECT_NEAR(out_ml, 9.14118, 1e-4);
}

TEST(distributions, normalPDFInf)
{
  const double cov_vals[3 * 3] = {4.0, 2.0, 1.0, 2.0, 3.0, 0.5, 1.0, 0.5, 1.0};
  const CMatrixDouble33 COV(cov_vals);
  const CMatrixDouble33 COV_inv = COV.inverse_LLt();

  const double x1_vals[3] = {1.0, 0.0, 0.0};
  const CMatrixFixed<double, 3, 1> x0;
  const CMatrixFixed<double, 3, 1> x1(x1_vals);

  // The unscaled version must match the plain covariance-based evaluation:
  EXPECT_NEAR(normalPDFInf(x1, x0, COV_inv), normalPDF(x1, x0, COV), 1e-12);

  // The scaled version peaks at 1 for x==mu:
  EXPECT_NEAR(normalPDFInf(x0, x0, COV_inv, true), 1.0, 1e-12);
  EXPECT_LT(normalPDFInf(x1, x0, COV_inv, true), 1.0);
}

TEST(distributions, KLD_Gaussians)
{
  CMatrixDouble cov0(2, 2);
  cov0.setIdentity();
  CMatrixDouble cov1(2, 2);
  cov1.setIdentity();

  CVectorDouble mu0(2);
  mu0.fill(0);
  CVectorDouble mu1(2);
  mu1.fill(0);

  // Identical distributions => zero divergence
  EXPECT_NEAR(KLD_Gaussians(mu0, cov0, mu1, cov1), 0.0, 1e-12);

  // Shifting the mean by 1 sigma in one axis adds 0.5:
  mu1[0] = 1.0;
  EXPECT_NEAR(KLD_Gaussians(mu0, cov0, mu1, cov1), 0.5, 1e-12);

  // Doubling the variance of one axis:
  cov1(0, 0) = 4.0;
  mu1[0] = 0.0;
  EXPECT_NEAR(KLD_Gaussians(mu0, cov0, mu1, cov1), 0.5 * (std::log(4.0) + 0.25 - 1.0), 1e-12);

  // Mismatched dimensions are rejected:
  CMatrixDouble cov3(3, 3);
  cov3.setIdentity();
  CVectorDouble mu3(3);
  mu3.fill(0);
  EXPECT_THROW(KLD_Gaussians(mu0, cov0, mu3, cov3), std::exception);
}

TEST(distributions, confidenceIntervals)
{
  // A uniform ramp of samples in [0,100]: the 10%-90% interval must be close
  // to [10,90] and the mean to 50.
  CVectorDouble data(1001);
  for (int i = 0; i < 1001; i++) data[i] = i * 0.1;

  double mean = 0, lower = 0, upper = 0;
  confidenceIntervals(data, mean, lower, upper, 0.1, 100);

  EXPECT_NEAR(mean, 50.0, 1e-6);
  EXPECT_NEAR(lower, 10.0, 2.0);
  EXPECT_NEAR(upper, 90.0, 2.0);

  // Invalid arguments:
  CVectorDouble empty(0);
  EXPECT_THROW(confidenceIntervals(empty, mean, lower, upper), std::exception);
  EXPECT_THROW(confidenceIntervals(data, mean, lower, upper, 0.0), std::exception);
  EXPECT_THROW(confidenceIntervals(data, mean, lower, upper, 1.0), std::exception);
}

TEST(distributions, confidenceIntervalsFromHistogram)
{
  // A flat histogram over [0,100) in 100 bins:
  std::vector<double> coords(100), hits(100, 0.01);
  for (int i = 0; i < 100; i++) coords[i] = i;

  double lower = 0, upper = 0;
  confidenceIntervalsFromHistogram(coords, hits, lower, upper, 0.1);

  EXPECT_NEAR(lower, 9.9, 2.0);
  EXPECT_NEAR(upper, 89.1, 2.0);

  EXPECT_THROW(confidenceIntervalsFromHistogram(coords, hits, lower, upper, 0.0), std::exception);
}

// Reference values below: scipy 1.11 / mpmath (40 digits).
namespace
{
struct XYRef
{
  double x;
  double y;
};

struct Chi2Ref
{
  unsigned int dof;
  double x;
  double pdf;
  double cdf;
};

struct NcChi2Ref
{
  unsigned int dof;
  double lambda;
  double x;
  double pdf;
  double cdf;
};

struct Chi2InvRef
{
  unsigned int dof;
  double p;
  double x;
};

const Chi2Ref kChi2Refs[] = {
    {1,              0.01,     3.9695254747701174,   0.07965567455405799},
    {1,               0.1,     1.2000389484301359,   0.24817036595415076},
    {1,               0.5,    0.43939128946772243,   0.52049987781304663},
    {1,                 1,    0.24197072451914337,   0.68268949213708585},
    {1, 3.841458820694124,    0.02981946110542984,   0.95000000000000007},
    {1,                10, 0.00085003666025203336,    0.9984345977419975},
    {2,              0.01,    0.49750623959634116,  0.004987520807317688},
    {2,               0.5,    0.38940039153570244,   0.22119921692859512},
    {2,                 2,    0.18393972058572114,   0.63212055882855767},
    {2,                10,  0.0033689734995427331,   0.99326205300091452},
    {3,               0.1,    0.12000389484301362, 0.0081625762681235194},
    {3,                 1,    0.24197072451914337,   0.19874804309879915},
    {3,                 5,   0.073224912809632448,   0.82820285570326646},
    {4,               0.5,   0.097350097883925624,  0.026499021160743912},
    {4, 3.841458820694124,    0.14069349123808167,   0.57211295303775245},
    {5,                 2,     0.1383691658068649,   0.15085496391539038},
    {5,                30, 6.6842620035749058e-06,   0.99998525141896155},
    {8,                 1,  0.0063180277053399327, 0.0017516225562908239},
    {8,                10,   0.070186947907140335,   0.73497408470263847},
};

const NcChi2Ref kNcChi2Refs[] = {
    {1, 0.5, 0.5,    0.3858716661290269,  0.42135039647485745},
    {1,   3,   2,   0.13509959657844864,  0.37447734620034817},
    {1,  20,  10,  0.026749416030065542, 0.095121888990315873},
    {1,  20,  40, 0.0056717691338375505,  0.96801718669240588},
    {2, 0.5,   2,   0.18136697355847872,  0.54573709886224164},
    {2,   3,  10,  0.031440679134170695,   0.8874767216968269},
    {2,  20,  10,  0.022700868783383676, 0.074392014749310995},
    {3, 0.5, 0.5,   0.17831791741872949, 0.064714561637398471},
    {3,   3,   2,   0.10867600842277858,  0.15712532935479098},
    {3,  20,  40, 0.0080210928317221725,  0.95197500102896138},
    {5, 0.5,  10,  0.035263933787232762,  0.89499240036407424},
    {5,   3,   2,  0.053841061578039584, 0.049443206198711766},
    {5,  20,  40,  0.010942483626089006,  0.93009003377678379},
};

const Chi2InvRef kChi2InvRefs[] = {
    { 1,  0.01, 0.00015708785790970184},
    { 1,   0.1,   0.015790774093431222},
    { 1,   0.5,      0.454936423119572},
    { 1,   0.9,      2.705543454095404},
    { 1,  0.95,      3.841458820694124},
    { 1,  0.99,     6.6348966010212145},
    { 1, 0.999,     10.827566170662733},
    { 2,  0.01,   0.020100671707002873},
    { 2,   0.1,    0.21072103131565273},
    { 2,   0.5,      1.386294361119891},
    { 2,  0.95,      5.991464547107979},
    { 2,  0.99,     9.2103403719761801},
    { 2, 0.999,     13.815510557964274},
    { 3,  0.01,    0.11483180189911707},
    { 3,   0.5,     2.3659738843753377},
    { 3,  0.95,     7.8147279032511792},
    { 3,  0.99,     11.344866730144373},
    { 4,  0.95,      9.487729036781154},
    { 6,  0.95,     12.591587243743977},
    { 6, 0.999,     22.457744484825323},
    {10,  0.01,     2.5582121601872063},
    {10,  0.95,     18.307038053275146},
    {10, 0.999,     29.588298445074422},
};
}  // namespace

TEST(distributions, normalCDF_AgainstReference)
{
  const XYRef refs[] = {
      {  -37.0, 5.7255712225245768e-300},
      {  -30.0, 4.9067139271481871e-198},
      {  -20.0,  2.7536241186062337e-89},
      {  -10.0,  7.6198530241605261e-24},
      {   -8.0,  6.2209605742717841e-16},
      {   -6.0,  9.8658764503769814e-10},
      {   -5.7,   5.9903714010635282e-9},
      {   -5.5,   1.8989562465887719e-8},
      {   -5.0,   2.8665157187919391e-7},
      {   -4.5,   3.3976731247300604e-6},
      {-4.0015,   3.1471097602762569e-5},
      {   -3.0,   1.3498980316300945e-3},
      {   -1.5,   6.6807201268858066e-2},
      {   -0.6,   2.7425311775007359e-1},
      {   -0.3,   3.8208857781104737e-1},
      {    0.0,                     0.5},
      {    0.3,   6.1791142218895263e-1},
      {    0.6,   7.2574688224992641e-1},
      {    1.5,   9.3319279873114193e-1},
      {    3.0,   9.9865010196836991e-1},
  };
  for (const auto& r : refs)
  {
    EXPECT_NEAR(mrpt::math::normalCDF(r.x) / r.y, 1.0, 2e-13) << "u=" << r.x;
  }
  // Upper tail: only absolute accuracy is meaningful for 1-Q(u)
  EXPECT_NEAR(mrpt::math::normalCDF(6.0), 9.9999999901341235e-1, 1e-15);
  for (double u = 0.1; u < 8.0; u += 0.37)
  {
    EXPECT_NEAR(mrpt::math::normalCDF(u) + mrpt::math::normalCDF(-u), 1.0, 1e-15);
  }
}

TEST(distributions, normalQuantile_AgainstReference)
{
  const XYRef refs[] = {
      { 1e-300,  -37.047096299361201},
      { 1e-100,  -21.273453560965322},
      {  1e-20,  -9.2623400897984087},
      {  1e-10,  -6.3613409024040557},
      {   1e-5,  -4.2648907939228247},
      {  0.001,  -3.0902323061678132},
      {   0.01,  -2.3263478740408408},
      {0.02425,  -1.9729610513118849},
      {  0.025,  -1.9599639845400545},
      {    0.1,  -1.2815515655446004},
      {    0.3, -0.52440051270804089},
      {    0.5,                  0.0},
      {    0.7,  0.52440051270804067},
      {    0.9,   1.2815515655446004},
      {  0.975,    1.959963984540054},
      {   0.99,   2.3263478740408408},
      {  0.999,   3.0902323061678132},
  };
  for (const auto& r : refs)
  {
    EXPECT_NEAR(mrpt::math::normalQuantile(r.x), r.y, 1e-12 * std::max(1.0, std::abs(r.y)))
        << "p=" << r.x;
  }
  EXPECT_THROW(mrpt::math::normalQuantile(0.0), std::exception);
  EXPECT_THROW(mrpt::math::normalQuantile(1.0), std::exception);

  for (double lp = -15; lp < -0.31; lp += 0.25)
  {
    const double p = std::pow(10.0, lp);
    EXPECT_NEAR(mrpt::math::normalCDF(mrpt::math::normalQuantile(p)) / p, 1.0, 1e-12) << "p=" << p;
  }
}

TEST(distributions, chi2inv_AgainstReference)
{
  for (const auto& r : kChi2InvRefs)
  {
    const double v = mrpt::math::chi2inv(r.p, r.dof);
    EXPECT_GT(v, 0.0);
    EXPECT_NEAR(v / r.x, 1.0, 1e-8) << "dof=" << r.dof << " p=" << r.p;
  }
  // Small probabilities, where Wilson-Hilferty alone goes negative:
  EXPECT_NEAR(mrpt::math::chi2inv(1e-3, 1) / 1.5707971492624921e-06, 1.0, 1e-8);
  EXPECT_NEAR(mrpt::math::chi2inv(1e-3, 2) / 0.002001000667167068, 1.0, 1e-8);
  // Large dimension (Wilson-Hilferty only):
  EXPECT_NEAR(mrpt::math::chi2inv(0.99, 500) / 576.49281251165519, 1.0, 1e-3);

  EXPECT_EQ(mrpt::math::chi2inv(0.0, 2), 0.0);
  EXPECT_THROW(mrpt::math::chi2inv(1.0, 2), std::exception);
  EXPECT_THROW(mrpt::math::chi2inv(-0.1, 2), std::exception);
  EXPECT_THROW(mrpt::math::chi2inv(0.5, 0), std::exception);
}

TEST(distributions, chi2PDF_chi2CDF_AgainstReference)
{
  for (const auto& r : kChi2Refs)
  {
    EXPECT_NEAR(mrpt::math::chi2CDF(r.dof, r.x), r.cdf, 1e-10) << "dof=" << r.dof << " x=" << r.x;
    EXPECT_NEAR(mrpt::math::chi2PDF(r.dof, r.x) / r.pdf, 1.0, 1e-10)
        << "dof=" << r.dof << " x=" << r.x;
  }
  // Round trip with chi2inv():
  for (const auto& r : kChi2InvRefs)
  {
    EXPECT_NEAR(mrpt::math::chi2CDF(r.dof, mrpt::math::chi2inv(r.p, r.dof)), r.p, 1e-10);
  }
  // At the origin:
  EXPECT_NEAR(mrpt::math::chi2CDF(1, 0.0), 0.0, 1e-15);
  EXPECT_NEAR(mrpt::math::chi2CDF(3, 0.0), 0.0, 1e-15);
  EXPECT_NEAR(mrpt::math::chi2PDF(2, 0.0), 0.5, 1e-15);
  EXPECT_NEAR(mrpt::math::chi2PDF(3, 0.0), 0.0, 1e-15);
  EXPECT_NEAR(mrpt::math::noncentralChi2PDF_CDF(2, 3.0, 0.0).first, 0.5 * std::exp(-1.5), 1e-15);
}

TEST(distributions, noncentralChi2_AgainstReference)
{
  for (const auto& r : kNcChi2Refs)
  {
    const auto pc = mrpt::math::noncentralChi2PDF_CDF(r.dof, r.lambda, r.x);
    EXPECT_NEAR(pc.first, r.pdf, 1e-7) << "dof=" << r.dof << " lambda=" << r.lambda;
    EXPECT_NEAR(pc.second, r.cdf, 1e-7) << "dof=" << r.dof << " lambda=" << r.lambda;
    // The Wilson-Hilferty approximation, within its documented accuracy:
    EXPECT_NEAR(mrpt::math::noncentralChi2CDF(r.dof, r.lambda, r.x), r.cdf, 0.1);
  }
}

TEST(distributions, confidenceIntervalsFromHistogram_ReturnsBinCoordinates)
{
  const std::vector<double> coords = {0.0, 1.0, 2.0, 3.0};
  const std::vector<double> hits = {1.0, 1.0, 1.0, 1.0};
  double lo = 0;
  double hi = 0;
  mrpt::math::confidenceIntervalsFromHistogram(coords, hits, lo, hi, 0.25);
  EXPECT_DOUBLE_EQ(lo, 0.0);
  EXPECT_DOUBLE_EQ(hi, 3.0);
}

TEST(distributions, confidenceIntervals_GaussianSamples)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  mrpt::math::CVectorDouble data(200000);
  for (int i = 0; i < data.size(); i++)
  {
    data[i] = rng.drawGaussian1D(3.0, 2.0);
  }
  double m = 0;
  double lo = 0;
  double hi = 0;
  mrpt::math::confidenceIntervals(data, m, lo, hi, 0.1, 2000);
  // 10% and 90% quantiles of N(3, 2^2): 3 -/+ 2*1.2815515655446004
  EXPECT_NEAR(m, 3.0, 0.03);
  EXPECT_NEAR(lo, 3.0 - 2.0 * 1.2815515655446004, 0.05);
  EXPECT_NEAR(hi, 3.0 + 2.0 * 1.2815515655446004, 0.05);
}

TEST(data_utils, mahalanobisDistance_MatchesExplicitInverse)
{
  mrpt::math::CMatrixDouble C(3, 3);
  const double cv[] = {4.0, 2.0, 1.0, 2.0, 3.0, 0.5, 1.0, 0.5, 1.0};
  for (int i = 0; i < 9; i++)
  {
    C(i / 3, i % 3) = cv[i];
  }
  mrpt::math::CVectorDouble x(3);
  mrpt::math::CVectorDouble mu(3);
  x[0] = 1.0;
  x[1] = -2.0;
  x[2] = 0.5;
  mu[0] = 0.2;
  mu[1] = 0.1;
  mu[2] = -0.3;
  const Eigen::Vector3d d(0.8, -2.1, 0.8);
  const double ref2 = d.dot(C.asEigen().inverse() * d);

  EXPECT_NEAR(mrpt::math::mahalanobisDistanceSq(x, mu, C), ref2, 1e-12);
  EXPECT_NEAR(mrpt::math::mahalanobisDistance(x, mu, C), std::sqrt(ref2), 1e-12);

  // Fixed-size covariance and vectors:
  mrpt::math::CMatrixDouble33 Cf;
  Cf.asEigen() = C.asEigen();
  mrpt::math::CVectorFixedDouble<3> xf;
  mrpt::math::CVectorFixedDouble<3> muf;
  for (int i = 0; i < 3; i++)
  {
    xf[i] = x[i];
    muf[i] = mu[i];
  }
  EXPECT_NEAR(mrpt::math::mahalanobisDistanceSq(xf, muf, Cf), ref2, 1e-12);

  mrpt::math::CVectorDouble dv(3);
  dv[0] = d[0];
  dv[1] = d[1];
  dv[2] = d[2];
  EXPECT_NEAR(mrpt::math::mahalanobisDistanceSq(dv, C), ref2, 1e-12);
  EXPECT_NEAR(mrpt::math::mahalanobisDistance(dv, C), std::sqrt(ref2), 1e-12);

  double maha2 = 0;
  double pdf = 0;
  mrpt::math::mahalanobisDistanceSqAndPDF(dv, C, maha2, pdf);
  EXPECT_NEAR(maha2, ref2, 1e-12);
  const double refPdf =
      std::exp(-0.5 * ref2) / std::sqrt(std::pow(2 * M_PI, 3) * C.asEigen().determinant());
  EXPECT_NEAR(pdf / refPdf, 1.0, 1e-12);
  EXPECT_NEAR(mrpt::math::normalPDF(dv, C) / refPdf, 1.0, 1e-12);
}

TEST(data_utils, mahalanobisDistanceCorrelatedGaussians)
{
  Eigen::Matrix2d S1;
  Eigen::Matrix2d S2;
  Eigen::Matrix2d S12;
  S1 << 2.0, 0.3, 0.3, 1.0;
  S2 << 1.5, -0.2, -0.2, 2.0;
  S12 << 0.4, 0.2, -0.1, 0.3;  // not symmetric
  const Eigen::Vector2d d(1.0, -0.5);
  const Eigen::Matrix2d S = S1 + S2 - S12 - S12.transpose();
  const double ref = d.dot(S.inverse() * d);

  mrpt::math::CMatrixDouble c1(2, 2);
  mrpt::math::CMatrixDouble c2(2, 2);
  mrpt::math::CMatrixDouble c12(2, 2);
  c1.asEigen() = S1;
  c2.asEigen() = S2;
  c12.asEigen() = S12;
  mrpt::math::CVectorDouble dv(2);
  dv[0] = d[0];
  dv[1] = d[1];
  EXPECT_NEAR(mrpt::math::mahalanobisDistanceSq(dv, c1, c2, c12), ref, 1e-12);
  EXPECT_NEAR(mrpt::math::mahalanobisDistance(dv, c1, c2, c12), std::sqrt(ref), 1e-12);

  // Independent Gaussians: (I + I)^-1 = I/2 => d^2 = 1 for d=(1,1)
  mrpt::math::CMatrixDouble I(2, 2);
  mrpt::math::CMatrixDouble Z(2, 2);
  I.setIdentity();
  Z.setZero();
  mrpt::math::CVectorDouble ones(2);
  ones.fill(1.0);
  EXPECT_NEAR(mrpt::math::mahalanobisDistance(ones, I, I, Z), 1.0, 1e-12);
}

TEST(data_utils, productIntegralTwoGaussians)
{
  Eigen::Matrix2d S1;
  Eigen::Matrix2d S2;
  Eigen::Matrix2d S12;
  S1 << 2.0, 0.3, 0.3, 1.0;
  S2 << 1.5, -0.2, -0.2, 2.0;
  S12 << 0.4, 0.2, -0.1, 0.3;
  const Eigen::Vector2d d(1.0, -0.5);
  const auto gaussAt = [&d](const Eigen::Matrix2d& S)
  { return std::exp(-0.5 * d.dot(S.inverse() * d)) / (2 * M_PI * std::sqrt(S.determinant())); };
  const double ref = gaussAt(S1 + S2);

  const std::vector<double> dv = {d[0], d[1]};
  mrpt::math::CMatrixDouble c1(2, 2);
  mrpt::math::CMatrixDouble c2(2, 2);
  mrpt::math::CMatrixDouble c12(2, 2);
  c1.asEigen() = S1;
  c2.asEigen() = S2;
  c12.asEigen() = S12;
  EXPECT_NEAR(mrpt::math::productIntegralTwoGaussians(dv, c1, c2) / ref, 1.0, 1e-12);

  mrpt::math::CMatrixDouble22 f1;
  mrpt::math::CMatrixDouble22 f2;
  f1.asEigen() = S1;
  f2.asEigen() = S2;
  EXPECT_NEAR((mrpt::math::productIntegralTwoGaussians<double, 2>(dv, f1, f2)) / ref, 1.0, 1e-12);

  double maha2 = 0;
  double ip = 0;
  mrpt::math::CVectorDouble dc(2);
  dc[0] = d[0];
  dc[1] = d[1];
  mrpt::math::productIntegralAndMahalanobisTwoGaussians(dc, c1, c2, maha2, ip);
  EXPECT_NEAR(ip / ref, 1.0, 1e-12);
  EXPECT_NEAR(maha2, d.dot((S1 + S2).inverse() * d), 1e-12);

  // Correlated version:
  const Eigen::Matrix2d S = S1 + S2 - S12 - S12.transpose();
  mrpt::math::productIntegralAndMahalanobisTwoGaussians(dc, c1, c2, maha2, ip, &c12);
  EXPECT_NEAR(maha2, d.dot(S.inverse() * d), 1e-12);
  EXPECT_NEAR(ip / gaussAt(S), 1.0, 1e-12);
}

TEST(data_utils, KLD_Gaussians_FullCovariance)
{
  Eigen::Matrix3d S0;
  S0 << 2.0, 0.3, -0.2, 0.3, 1.0, 0.1, -0.2, 0.1, 0.5;
  Eigen::Matrix3d S1;
  S1 << 1.0, -0.2, 0.0, -0.2, 3.0, 0.4, 0.0, 0.4, 2.0;
  const Eigen::Vector3d m0(0.5, -1.0, 2.0);
  const Eigen::Vector3d m1(1.0, 0.0, 1.5);
  const Eigen::Vector3d dm = m1 - m0;
  const double ref = 0.5 * (std::log(S1.determinant() / S0.determinant()) +
                            (S1.inverse() * S0).trace() + dm.dot(S1.inverse() * dm) - 3.0);

  mrpt::math::CMatrixDouble c0(3, 3);
  mrpt::math::CMatrixDouble c1(3, 3);
  mrpt::math::CVectorDouble v0(3);
  mrpt::math::CVectorDouble v1(3);
  c0.asEigen() = S0;
  c1.asEigen() = S1;
  for (int i = 0; i < 3; i++)
  {
    v0[i] = m0[i];
    v1[i] = m1[i];
  }
  EXPECT_NEAR(mrpt::math::KLD_Gaussians(v0, c0, v1, c1), ref, 1e-12);
}
