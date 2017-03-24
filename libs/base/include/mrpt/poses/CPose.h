/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSE_H
#define CPOSE_H

#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt
{
	namespace poses
	{
		/** A base class for representing a pose in 2D or 3D.
		 *   For more information refer to the <a href="http://www.mrpt.org/2D_3D_Geometry"> 2D/3D Geometry tutorial</a> online.
		 * \note This class is based on the CRTP design pattern
		 * \sa CPoseOrPoint, CPoint
		 * \ingroup poses_grp
		 */
		template <class DERIVEDCLASS>
		class CPose : public CPoseOrPoint<DERIVEDCLASS>
		{


		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
