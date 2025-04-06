/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers
//
#include <mrpt/math/geometry.h>  // crossProduct3D()
#include <mrpt/vision/chessboard_find_corners.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

/** Look for the corners of a chessboard in the image
 * \param cornerCoords [OUT] The pixel coordinates of all the corners.
 * \param check_size_x [IN] The number of squares, in the X direction
 * \param check_size_y [IN] The number of squares, in the Y direction
 * \param normalize_image [IN] Whether to normalize the image before detection
 * \param useScaramuzzaMethod [IN] Whether to use the alternative, more robust
 * method by M. Rufli, D. Scaramuzza, and R. Siegwart.
 *
 * \return true on success
 */
bool mrpt::vision::findChessboardCorners(
    const mrpt::img::CImage& in_img,
    std::vector<TPixelCoordf>& cornerCoords,
    unsigned int check_size_x,
    unsigned int check_size_y,
    bool normalize_image)
{
#if MRPT_HAS_OPENCV
  MRPT_START

  ASSERT_(check_size_y > 0 && check_size_x > 0);

  // Grayscale version:
  const CImage img(in_img, FAST_REF_OR_CONVERT_TO_GRAY);

  // Try with expanded versions of the image if it fails to detect the
  // checkerboard:
  int corners_count;
  bool corners_found = false;

  const CvSize check_size = cvSize(check_size_x, check_size_y);

  const int CORNERS_COUNT = check_size_x * check_size_y;

  vector<cv::Point2f> corners_list;
  corners_count = CORNERS_COUNT;
  corners_list.resize(CORNERS_COUNT);

  cornerCoords.clear();

  int find_chess_flags = cv::CALIB_CB_ADAPTIVE_THRESH;
  if (normalize_image) find_chess_flags |= cv::CALIB_CB_NORMALIZE_IMAGE;

  cv::Mat cvImg = img.asCvMat<cv::Mat>(SHALLOW_COPY);
  vector<cv::Point2f> pointbuf;

  // Standard OpenCV's function:
  corners_found = 0 != cv::findChessboardCorners(cvImg, check_size, pointbuf, find_chess_flags);

  corners_list.resize(pointbuf.size());
  for (size_t i = 0; i < pointbuf.size(); i++)
  {
    corners_list[i].x = pointbuf[i].x;
    corners_list[i].y = pointbuf[i].y;
  }

  // Check # of corners:
  if (corners_found && corners_count != CORNERS_COUNT) corners_found = false;

  if (corners_found)
  {
    // Refine corners:
    cv::cornerSubPix(
        cvImg, corners_list, cv::Size(5, 5),  // window
        cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 10, 0.01f));

    // save the corners in the data structure:
    int y;
    unsigned int k;
    for (y = 0, k = 0; y < check_size.height; y++)
      for (int x = 0; x < check_size.width; x++, k++)
        cornerCoords.emplace_back(corners_list[k].x, corners_list[k].y);
  }

  return corners_found;

  MRPT_END
#else
  return false;
#endif
}
