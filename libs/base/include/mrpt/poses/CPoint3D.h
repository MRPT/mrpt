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
#ifndef CPOINT3D_H
#define CPOINT3D_H

#include <mrpt/poses/CPoint.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE( CPoint3D )

	/** A class used to store a 3D point.
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry" >2D/3D Geometry tutorial</a> in the wiki.
	 *
	 *  <div align=center>
	 *   <img src="CPoint3D.gif">
	 *  </div>
	 *
	 * \ingroup poses_grp
	 * \sa CPoseOrPoint,CPose, CPoint
	 */
	class BASE_IMPEXP  CPoint3D : public CPoint<CPoint3D>, public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoint3D )

	public:
		mrpt::math::CArrayDouble<3>   m_coords; //!< [x,y,z]

	public:
		/** Constructor for initializing point coordinates. */
		inline CPoint3D(const double x=0,const double y=0,const double z=0) { m_coords[0]= x; m_coords[1]=y; m_coords[2]=z; }

		/** Constructor from a XYZ 3-vector */
		explicit inline CPoint3D(const mrpt::math::CArrayDouble<3> &xyz) : m_coords(xyz) { }

		/** Constructor from an CPoint2D object. */
		CPoint3D( const CPoint2D &p);

		/** Constructor from an CPose3D object. */
		explicit inline CPoint3D( const CPose3D &p) { m_coords[0]=p.x(); m_coords[1]=p.y(); m_coords[2]=p.z(); }

		/** Constructor from an CPose2D object. */
		explicit CPoint3D( const CPose2D &p);

		/** Constructor from lightweight object. */
		inline CPoint3D(const mrpt::math::TPoint3D &p) { m_coords[0]=p.x; m_coords[1]=p.y; m_coords[2]=p.z; }

		/** Returns this point as seen from "b", i.e. result = this - b */
		CPoint3D  operator - (const CPose3D& b) const;

		/** Returns this point minus point "b", i.e. result = this - b */
		CPoint3D  operator - (const CPoint3D& b) const;

		/** Returns this point plus point "b", i.e. result = this + b */
		CPoint3D  operator + (const CPoint3D& b) const;

		/** Returns this point plus pose "b", i.e. result = this + b  */
		CPose3D	operator + (const CPose3D& b) const;


		enum { is_3D_val = 1 };
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
		enum { static_size = 3 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPoint3D to %u.",static_cast<unsigned>(n))); }
		/** @} */

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
