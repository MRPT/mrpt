/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
