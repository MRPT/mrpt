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
#ifndef CPOINT2D_H
#define CPOINT2D_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPoint.h>

namespace mrpt
{
namespace poses
{
	class CPose2D;

	DEFINE_SERIALIZABLE_PRE( CPoint2D )

	/** A class used to store a 2D point.
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry" >2D/3D Geometry tutorial</a> in the wiki.
	 *
	 *  <div align=center>
	 *   <img src="CPoint2D.gif">
	 *  </div>
	 *
	 * \sa CPoseOrPoint,CPose, CPoint
	 * \ingroup poses_grp
	 */
	class BASE_IMPEXP CPoint2D : public CPoint<CPoint2D>, public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoint2D )

	public:
		mrpt::math::CArrayDouble<2>   m_coords; //!< [x,y]

	public:
		 /** Constructor for initializing point coordinates. */
		 inline CPoint2D(double x=0,double y=0) { m_coords[0]=x; m_coords[1]=y; }

		 /** Constructor from x/y coordinates given from other pose. */
		template <class OTHERCLASS>
		inline explicit CPoint2D(const CPoseOrPoint<OTHERCLASS> &b)
		{
			m_coords[0]=b.x();
			m_coords[1]=b.y();
		}

		/** Implicit constructor from lightweight type. */
		inline CPoint2D(const mrpt::math::TPoint2D &o) { m_coords[0]=o.x; m_coords[1]=o.y; }

		/** Explicit constructor from lightweight type (loses the z coord). */
		inline explicit CPoint2D(const mrpt::math::TPoint3D &o) { m_coords[0]=o.x; m_coords[1]=o.y; m_coords[2]=0; }

		/** The operator D="this"-b is the pose inverse compounding operator,
		*   the resulting points "D" fulfils: "this" = b + D, so that: b == a + (b-a)
		*/
		CPoint2D  operator - (const CPose2D& b) const;

		enum { is_3D_val = 0 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { is_PDF_val = 0 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

		 /** @name STL-like methods and typedefs
		   @{   */
		typedef double         value_type;		//!< The type of the elements
		typedef double&        reference;
		typedef const double&  const_reference;
		typedef std::size_t    size_type;
		typedef std::ptrdiff_t difference_type;

		// size is constant
		enum { static_size = 2 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPoint2D to %u.",static_cast<unsigned>(n))); }
		/** @} */

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
