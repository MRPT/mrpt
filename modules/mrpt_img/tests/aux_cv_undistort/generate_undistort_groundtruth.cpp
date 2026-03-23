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

/**
 * Standalone tool: generate undistortion ground truth using OpenCV.
 *
 * Usage:
 *   ./generate_undistort_groundtruth <input_image> <output_undistorted_image>
 *
 * The camera parameters are hard-coded below to match the unit test values in
 * CUndistortMap_unittest.cpp. If you change them there, update them here too.
 */

#include <cstdlib>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <input_image> <output_undistorted_image>\n";
    return 1;
  }

  const std::string inFile = argv[1];
  const std::string outFile = argv[2];

  cv::Mat img = cv::imread(inFile, cv::IMREAD_UNCHANGED);
  if (img.empty())
  {
    std::cerr << "Error: cannot read image: " << inFile << "\n";
    return 1;
  }

  // Camera intrinsics (must match unit test values)
  const double fx = static_cast<double>(img.cols) * 1.2;
  const double fy = fx;
  const double cx = static_cast<double>(img.cols) * 0.5;
  const double cy = static_cast<double>(img.rows) * 0.5;

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  // Plumb-bob distortion coefficients (must match unit test values)
  const double k1 = -0.28;
  const double k2 = 0.07;
  const double p1 = 0.0002;
  const double p2 = 0.0002;
  const double k3 = 0.0;

  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

  cv::Mat undistorted;
  cv::undistort(img, undistorted, K, distCoeffs);

  if (!cv::imwrite(outFile, undistorted))
  {
    std::cerr << "Error: cannot write output image: " << outFile << "\n";
    return 1;
  }

  std::cout << "Camera parameters used:\n"
            << "  Image size: " << img.cols << " x " << img.rows << "\n"
            << "  fx=" << fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << "\n"
            << "  k1=" << k1 << " k2=" << k2 << " p1=" << p1 << " p2=" << p2 << " k3=" << k3 << "\n"
            << "  distortion model: plumb_bob\n"
            << "Undistorted image written to: " << outFile << "\n";

  return 0;
}
