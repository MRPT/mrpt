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

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/vision/types.h>

namespace mrpt::vision
{
/** \addtogroup mrpt_vision_grp
 *  @{ */

/** Extract a UNITARY 3D vector in the direction of a 3D point, given from its
 * (x,y) pixels coordinates, and the camera intrinsic coordinates.
 *  \param xy  [IN]   Pixels coordinates, from the top-left corner of the
 * image.
 *  \param A   [IN]   The 3x3 intrinsic parameters matrix for the camera.
 *  \return The mrpt::math::TPoint3D containing the output unitary vector.
 * \sa buildIntrinsicParamsMatrix, TPixelCoordf
 */
mrpt::math::TPoint3D pixelTo3D(
    const mrpt::img::TPixelCoordf& xy, const mrpt::math::CMatrixDouble33& A);

/** Builds the intrinsic parameters matrix A from parameters:
 * \param focalLengthX [IN]   The focal length, in X (horizontal) pixels
 * \param focalLengthY [IN]   The focal length, in Y (vertical) pixels
 * \param centerX      [IN]   The image center, horizontal, in pixels
 * \param centerY      [IN]   The image center, vertical, in pixels
 * \sa pixelTo3D
 */
mrpt::math::CMatrixDouble33 buildIntrinsicParamsMatrix(
    double focalLengthX, double focalLengthY, double centerX, double centerY);

/** Normalizes the brightness and contrast of an image by setting its mean value
 * to zero and its standard deviation to unit.
 * \param image        [IN]        The input image.
 * \param nimage       [OUTPUT]    The new normalized image.
 */
void normalizeImage(const mrpt::img::CImage& image, mrpt::img::CImage& nimage);

/** Calculates the Sum of Absolutes Differences (range [0,1]) between two
 * patches. Both patches must have the same size.
 * \param patch1 [IN]  One patch.
 * \param patch2 [IN]  The other patch.
 * \return The value of computed SAD normalized to [0,1]
 */
double computeSAD(const mrpt::img::CImage& patch1, const mrpt::img::CImage& patch2);

/** @} */  // end of grouping

}  // namespace mrpt::vision
