/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSEORPOINT_DETAIL_H
#define CPOSEORPOINT_DETAIL_H

#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
	namespace poses
	{
		/** Internal, auxiliary templates for MRPT classes */
		namespace detail
		{
			template <class POSEORPOINT>  struct T3DTypeHelper; // generic version. Specialized below.

			template <>  struct T3DTypeHelper<CPoint2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPoint3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose2D> { enum { is_3D_val = 0 }; };
			template <>  struct T3DTypeHelper<CPose3D> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose3DQuat> { enum { is_3D_val = 1 }; };
			template <>  struct T3DTypeHelper<CPose3DRotVec> { enum { is_3D_val = 1 }; };


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
