/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef CPOINT_H
#define CPOINT_H

#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt
{
	namespace poses
	{
		/** A base class for representing a point in 2D or 3D.
		 *   For more information refer to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> online.
		 * \note This class is based on the CRTP design pattern
		 * \sa CPoseOrPoint, CPose
		 * \ingroup poses_grp
		 */
		template <class DERIVEDCLASS>
		class CPoint : public CPoseOrPoint<DERIVEDCLASS>
		{
		public:
			/** @name Methods common to all 2D or 3D points
			    @{ */

			/** Scalar addition of all coordinates.
			  * This is diferent from poses/point composition, which is implemented as "+" operators in classes derived from "CPose"
			  */
			template <class OTHERCLASS>
			inline void AddComponents(const OTHERCLASS &b)
			{
				const int dims = std::min( size_t(DERIVEDCLASS::static_size), size_t(OTHERCLASS::is3DPoseOrPoint() ? 3:2));
				for (int i=0;i<dims;i++)
					static_cast<DERIVEDCLASS*>(this)->m_coords[i]+= static_cast<const OTHERCLASS*>(&b)->m_coords[i];
			}

			/** Scalar multiplication. */
			inline void operator *=(const double s)
			{
				for (int i=0;i<DERIVEDCLASS::static_size;i++)
					static_cast<DERIVEDCLASS*>(this)->m_coords[i] *= s;
			}

			/** Return the pose or point as a 1x2 or 1x3 vector [x y] or [x y z] */
			inline void getAsVector(vector_double &v) const
			{
				v.resize(DERIVEDCLASS::static_size);
				for (int i=0;i<DERIVEDCLASS::static_size;i++)
					v[i] = static_cast<const DERIVEDCLASS*>(this)->m_coords[i];
			}
			//! \overload
			inline vector_double getAsVector() const { vector_double v; getAsVector(v); return v; }

			/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
			  * \sa getInverseHomogeneousMatrix
			  */
			void getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const
			{
				out_HM.unit(4,1.0);
				out_HM.get_unsafe(0,3)= static_cast<const DERIVEDCLASS*>(this)->x();
				out_HM.get_unsafe(1,3)= static_cast<const DERIVEDCLASS*>(this)->y();
				if (DERIVEDCLASS::is3DPoseOrPoint())
					out_HM.get_unsafe(2,3)= static_cast<const DERIVEDCLASS*>(this)->m_coords[2];
			}

			/** Returns a human-readable textual representation of the object (eg: "[0.02 1.04]" )
			* \sa fromString
			*/
			void asString(std::string &s) const
			{
				s = (!DERIVEDCLASS::is3DPoseOrPoint()) ?
					mrpt::format("[%f %f]", static_cast<const DERIVEDCLASS*>(this)->x(), static_cast<const DERIVEDCLASS*>(this)->y()) :
					mrpt::format("[%f %f %f]",static_cast<const DERIVEDCLASS*>(this)->x(), static_cast<const DERIVEDCLASS*>(this)->y(), static_cast<const DERIVEDCLASS*>(this)->m_coords[2]);
			}
			inline std::string asString() const { std::string s; asString(s); return s; }

			/** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04]" )
			* \sa asString
			* \exception std::exception On invalid format
			*/
			void fromString(const std::string &s)
			{
				CMatrixDouble  m;
				if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
				ASSERT_EQUAL_(mrpt::math::size(m,1),1)
				ASSERT_EQUAL_(mrpt::math::size(m,2),DERIVEDCLASS::static_size)
				for (int i=0;i<DERIVEDCLASS::static_size;i++)
					static_cast<DERIVEDCLASS*>(this)->m_coords[i] = m.get_unsafe(0,i);
			}

			inline const double &operator[](unsigned int i) const { return static_cast<const DERIVEDCLASS*>(this)->m_coords[i]; }
			inline       double &operator[](unsigned int i)       { return static_cast<DERIVEDCLASS*>(this)->m_coords[i]; }

			/** @} */

		}; // End of class def.

	/** Dumps a point as a string [x,y] or [x,y,z]  */
	template <class DERIVEDCLASS>
	std::ostream &operator << (std::ostream& o, const CPoint<DERIVEDCLASS>& p)
	{
		o << "(" << p[0] << "," << p[1];
		if (p.is3DPoseOrPoint())	o << "," << p[2];
		o <<")";
		return o;
	}

	/** Used by STL algorithms */
	template <class DERIVEDCLASS>
	bool operator < (const CPoint<DERIVEDCLASS> &a, const CPoint<DERIVEDCLASS> &b)
	{
		if (a.x()<b.x()) return true;
		else
		{
			if (!a.is3DPoseOrPoint())
				return a.y()<b.y();
			else  if (a.y()<b.y())
				return true;
			else return a[2]<b[2];
		}
	}

	template <class DERIVEDCLASS>
	bool operator==(const CPoint<DERIVEDCLASS> &p1,const CPoint<DERIVEDCLASS> &p2)
	{
		for (int i=0;i<DERIVEDCLASS::static_size;i++)
			if (p1[i]!=p2[i])	return false;
		return true;
	}

	template <class DERIVEDCLASS>
	bool operator!=(const CPoint<DERIVEDCLASS> &p1,const CPoint<DERIVEDCLASS> &p2)
	{
		for (int i=0;i<DERIVEDCLASS::static_size;i++)
			if (p1[i]!=p2[i])	return true;
		return false;
	}



	} // End of namespace
} // End of namespace

#endif
