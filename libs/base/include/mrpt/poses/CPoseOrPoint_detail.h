/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef CPOSEORPOINT_DETAIL_H
#define CPOSEORPOINT_DETAIL_H

namespace mrpt
{
	namespace poses
	{
		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;
		class CPose3DQuat;

		/** Internal, auxiliary templates for MRPT classes */
		namespace detail
		{
			template <class POSEORPOINT>  struct T3DTypeHelper; // generic version. Specialized below.

			template <>  struct T3DTypeHelper<CPoint2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPoint3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPose3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose3DQuat> { enum { is_3D_val = 1 }; };



			template <class DERIVEDCLASS, int IS3D> struct pose_point_impl;  // generic template, specialized below:

			// Extra members for 3D implementation:
			template <class DERIVEDCLASS> struct pose_point_impl<DERIVEDCLASS,1>
			{
				inline double z() const /*!< Get Z coord. */ { return static_cast<const DERIVEDCLASS*>(this)->m_coords[2]; }
				inline double &z() /*!< Get ref to Z coord. */ { return static_cast<DERIVEDCLASS*>(this)->m_coords[2]; }
				inline void z(const double v) /*!< Set Z coord. */ { static_cast<DERIVEDCLASS*>(this)->m_coords[2]=v; }
				inline void z_incr(const double v) /*!< Z+=v */ { static_cast<DERIVEDCLASS*>(this)->m_coords[2]+=v; }
			};

			// Extra members for 2D implementation:
			template <class DERIVEDCLASS> struct pose_point_impl<DERIVEDCLASS,0>
			{
			};

		} // End of namespace
	} // End of namespace
} // End of namespace

#endif
