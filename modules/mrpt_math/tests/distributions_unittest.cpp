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
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

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
