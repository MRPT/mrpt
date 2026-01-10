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

#include <mrpt/opengl/CSetOfObjects.h>

namespace mrpt::opengl
{
/** @name Functions to obtain a 3D representation of a pose PDF
  @{  */

/** Returns a representation of a the PDF - this is just an auxiliary function,
 * it's more natural to call mrpt::poses::CPosePDF::getAs3DObject     */
template <class POSE_PDF>
inline CSetOfObjects::Ptr posePDF2opengl(const POSE_PDF& o)
{
  return CSetOfObjects::posePDF2opengl(o);
}

/**  @}  */
}  // namespace mrpt::opengl
