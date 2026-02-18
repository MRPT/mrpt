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

#include <mrpt/math/ops_matrices.h>
#include <mrpt/vision/utils.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

/*-------------------------------------------------------------
          pixelTo3D
-------------------------------------------------------------*/
TPoint3D vision::pixelTo3D(const TPixelCoordf& xy, const CMatrixDouble33& A)
{
  TPoint3D res;

  // Build the vector:
  res.x = xy.x - A(0, 2);
  res.y = xy.y - A(1, 2);
  res.z = A(0, 0);

  // Normalize:
  const double u = res.norm();
  ASSERT_(u != 0);
  res *= 1.0 / u;

  return res;
}

/*-------------------------------------------------------------
          buildIntrinsicParamsMatrix
-------------------------------------------------------------*/
CMatrixDouble33 vision::buildIntrinsicParamsMatrix(
    const double focalLengthX,
    const double focalLengthY,
    const double centerX,
    const double centerY)
{
  CMatrixDouble33 A;

  A(0, 0) = focalLengthX;
  A(1, 1) = focalLengthY;
  A(2, 2) = 1;

  A(0, 2) = centerX;
  A(1, 2) = centerY;

  return A;
}

/*-------------------------------------------------------------
          normalizeImage
-------------------------------------------------------------*/
void vision::normalizeImage(const CImage& image, CImage& nimage)
{
  ASSERT_(image.channels() == 1);
  nimage.resize(image.getWidth(), image.getHeight(), image.channels());

  CMatrixFloat im, nim;
  nim.resize(image.getHeight(), image.getWidth());

  image.getAsMatrix(im);

  double m, s;
  mrpt::math::meanAndStd(im, m, s);

  for (int k1 = 0; k1 < (int)nim.cols(); ++k1)
    for (int k2 = 0; k2 < (int)nim.rows(); ++k2) nim(k2, k1) = (im(k2, k1) - m) / s;

  nimage.setFromMatrix(nim);
}

double vision::computeSAD(const CImage& p1, const CImage& p2)
{
  MRPT_START
  ASSERT_(p1.getSize() == p2.getSize());
  const auto w = p1.getWidth(), h = p1.getHeight();
  double res = 0.0;
  for (unsigned int ii = 0; ii < h; ++ii)
    for (unsigned int jj = 0; jj < w; ++jj)
      res += std::abs(
          static_cast<double>(p1.at<uint8_t>(jj, ii)) -
          static_cast<double>(p2.at<uint8_t>(jj, ii)));

  return res / (255.0 * w * h);
  MRPT_END
}

/*-------------------------------------------------------------
      TStereoSystemParams
-------------------------------------------------------------*/
TStereoSystemParams::TStereoSystemParams()
{
  F.setZero();
  F(1, 2) = -1;
  F(2, 1) = 1;
}

void TStereoSystemParams::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
  int unc;
  unc = iniFile.read_int(section.c_str(), "uncPropagation", uncPropagation);
  switch (unc)
  {
    case 0:
      uncPropagation = Prop_Linear;
      break;
    case 1:
      uncPropagation = Prop_UT;
      break;
    case 2:
      uncPropagation = Prop_SUT;
      break;
  }

  CVectorDouble k_vec(9);
  iniFile.read_vector(section.c_str(), "k_vec", CVectorDouble(), k_vec, false);
  for (unsigned int ii = 0; ii < 3; ++ii)
    for (unsigned int jj = 0; jj < 3; ++jj) K(ii, jj) = k_vec[ii * 3 + jj];

  CVectorDouble f_vec(9);
  iniFile.read_vector(section.c_str(), "f_vec", CVectorDouble(), f_vec, false);
  for (unsigned int ii = 0; ii < 3; ++ii)
    for (unsigned int jj = 0; jj < 3; ++jj) F(ii, jj) = f_vec[ii * 3 + jj];

  baseline = iniFile.read_float(section.c_str(), "baseline", baseline);
  stdPixel = iniFile.read_float(section.c_str(), "stdPixel", stdPixel);
  stdDisp = iniFile.read_float(section.c_str(), "stdDisp", stdDisp);
  maxZ = iniFile.read_float(section.c_str(), "maxZ", maxZ);
  minZ = iniFile.read_float(section.c_str(), "minZ", minZ);
  maxY = iniFile.read_float(section.c_str(), "maxY", maxY);
  factor_k = iniFile.read_float(section.c_str(), "factor_k", factor_k);
  factor_a = iniFile.read_float(section.c_str(), "factor_a", factor_a);
  factor_b = iniFile.read_float(section.c_str(), "factor_b", factor_b);
}

void TStereoSystemParams::dumpToTextStream(std::ostream& out) const
{
  out << "\n----------- [vision::TStereoSystemParams] ------------ \n";
  out << "Method for 3D Uncert. \t= ";
  switch (uncPropagation)
  {
    case Prop_Linear:
      out << "Linear propagation\n";
      break;
    case Prop_UT:
      out << "Unscented Transform\n";
      break;
    case Prop_SUT:
      out << "Scaled Unscented Transform\n";
      break;
  }

  out << mrpt::format("K\t\t\t= [%f\t%f\t%f]\n", K(0, 0), K(0, 1), K(0, 2));
  out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", K(1, 0), K(1, 1), K(1, 2));
  out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", K(2, 0), K(2, 1), K(2, 2));

  out << mrpt::format("F\t\t\t= [%f\t%f\t%f]\n", F(0, 0), F(0, 1), F(0, 2));
  out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", F(1, 0), F(1, 1), F(1, 2));
  out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", F(2, 0), F(2, 1), F(2, 2));

  out << mrpt::format("Baseline \t\t= %f\n", baseline);
  out << mrpt::format("Pixel std \t\t= %f\n", stdPixel);
  out << mrpt::format("Disparity std\t\t= %f\n", stdDisp);
  out << mrpt::format("Z maximum\t\t= %f\n", maxZ);
  out << mrpt::format("Z minimum\t\t= %f\n", minZ);
  out << mrpt::format("Y maximum\t\t= %f\n", maxY);

  out << mrpt::format("k Factor [UT]\t\t= %f\n", factor_k);
  out << mrpt::format("a Factor [UT]\t\t= %f\n", factor_a);
  out << mrpt::format("b Factor [UT]\t\t= %f\n", factor_b);
  out << "-------------------------------------------------------- \n";
}

void TMatchingOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
  int mm = iniFile.read_int(section.c_str(), "matching_method", matching_method);
  switch (mm)
  {
    case 0:
      matching_method = mmCorrelation;
      break;
    case 1:
      matching_method = mmDescriptorSIFT;
      break;
    case 2:
      matching_method = mmDescriptorSURF;
      break;
    case 3:
      matching_method = mmSAD;
      break;
    case 4:
      matching_method = mmDescriptorORB;
      break;
  }

  useEpipolarRestriction =
      iniFile.read_bool(section.c_str(), "useEpipolarRestriction", useEpipolarRestriction);
  hasFundamentalMatrix =
      iniFile.read_bool(section.c_str(), "hasFundamentalMatrix", hasFundamentalMatrix);
  parallelOpticalAxis =
      iniFile.read_bool(section.c_str(), "parallelOpticalAxis", parallelOpticalAxis);
  useXRestriction = iniFile.read_bool(section.c_str(), "useXRestriction", useXRestriction);
  addMatches = iniFile.read_bool(section.c_str(), "addMatches", addMatches);
  useDisparityLimits = iniFile.read_bool(section.c_str(), "useDisparityLimits", useDisparityLimits);

  min_disp = iniFile.read_float(section.c_str(), "min_disp", min_disp);
  max_disp = iniFile.read_float(section.c_str(), "max_disp", max_disp);

  epipolar_TH = iniFile.read_float(section.c_str(), "epipolar_TH", epipolar_TH);
  maxEDD_TH = iniFile.read_float(section.c_str(), "maxEDD_TH", maxEDD_TH);
  EDD_RATIO = iniFile.read_float(section.c_str(), "minDIF_TH", EDD_RATIO);
  minCC_TH = iniFile.read_float(section.c_str(), "minCC_TH", minCC_TH);
  minDCC_TH = iniFile.read_float(section.c_str(), "minDCC_TH", minDCC_TH);
  rCC_TH = iniFile.read_float(section.c_str(), "rCC_TH", rCC_TH);
  maxEDSD_TH = iniFile.read_float(section.c_str(), "maxEDSD_TH", maxEDSD_TH);
  EDSD_RATIO = iniFile.read_float(section.c_str(), "EDSD_RATIO", EDSD_RATIO);
  maxSAD_TH = iniFile.read_float(section.c_str(), "maxSAD_TH", maxSAD_TH);
  SAD_RATIO = iniFile.read_float(section.c_str(), "SAD_RATIO", SAD_RATIO);
  maxORB_dist = iniFile.read_float(section.c_str(), "maxORB_dist", maxORB_dist);

  estimateDepth = iniFile.read_bool(section.c_str(), "estimateDepth", estimateDepth);
  maxDepthThreshold = iniFile.read_float(section.c_str(), "maxDepthThreshold", maxDepthThreshold);
}

void TMatchingOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n----------- [vision::TMatchingOptions] ------------ \n";
  out << "Matching method:                ";
  switch (matching_method)
  {
    case mmCorrelation:
      out << "Cross Correlation\n";
      out << mrpt::format("  Min. CC. Threshold:           %f\n", minCC_TH);
      out << mrpt::format("  Min. Dif. CC Threshold:       %f\n", minDCC_TH);
      out << mrpt::format("  Max. Ratio CC Threshold:      %f\n", rCC_TH);
      break;
    case mmDescriptorSIFT:
      out << "SIFT descriptor\n";
      out << mrpt::format("  Max. EDD Threshold:           %f\n", maxEDD_TH);
      out << mrpt::format("  EDD Ratio:                    %f\n", EDD_RATIO);
      break;
    case mmDescriptorSURF:
      out << "SURF descriptor\n";
      out << mrpt::format("  EDD Ratio:                    %f\n", maxEDSD_TH);
      out << mrpt::format("  Min. CC Threshold:            %f\n", EDSD_RATIO);
      break;
    case mmSAD:
      out << "SAD\n";
      out << mrpt::format("  Max. Dif. SAD Threshold:      %f\n", maxSAD_TH);
      out << mrpt::format("  Ratio SAD Threshold:          %f\n", SAD_RATIO);
      break;
    case mmDescriptorORB:
      out << "ORB\n";
      out << mrpt::format("  Max. distance between desc:   %f\n", maxORB_dist);
      break;
  }
  out << mrpt::format("Epipolar Thres:                 %.2f px\n", epipolar_TH);
  out << "Using epipolar restriction?:    " << (useEpipolarRestriction ? "Yes\n" : "No\n");
  out << "Has Fundamental Matrix?:        " << (hasFundamentalMatrix ? "Yes\n" : "No\n");
  out << "Are camera axis parallel?:      " << (parallelOpticalAxis ? "Yes\n" : "No\n");
  out << "Use X-coord restriction?:       " << (useXRestriction ? "Yes\n" : "No\n");
  out << "Use disparity limits?:       " << (useDisparityLimits ? "Yes\n" : "No\n");
  if (useDisparityLimits)
  {
    out << mrpt::format("  Min/max disp limits:          %.2f/%.2f px\n", min_disp, max_disp);
  }
  out << "Estimate depth?:                " << (estimateDepth ? "Yes\n" : "No\n");
  if (estimateDepth)
  {
    out << mrpt::format("  Maximum depth allowed:        %f m\n", maxDepthThreshold);
  }
  out << "Add matches to list?:           ";
  out << (addMatches ? "Yes\n" : "No\n");
  out << "-------------------------------------------------------- \n";
}
