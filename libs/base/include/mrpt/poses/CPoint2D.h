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
