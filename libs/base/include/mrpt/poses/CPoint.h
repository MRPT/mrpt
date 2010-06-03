/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CPOINT_H
#define CPOINT_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt
{
namespace poses
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoint, mrpt::utils::CSerializable )

	/** A virtual base class to represent a point in 2D or 3D.
	 *
	 *
	 *   For more information refer to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> in the wiki.
	 *
	 * \sa CPoseOrPoint,CPose, CPoint
	 */
	class BASE_IMPEXP CPoint : public CPoseOrPoint
	{
		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CPoint )

	public:
		 /** Scalar sum of components: This is diferent from poses
		  *    composition, which is implemented as "+" operators in "CPose" derived classes.
		  */
		 void AddComponents(CPoseOrPoint &p);

		 /** Scalar multiplication.
		  */
		 void operator *=(const double s);

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
