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
#ifndef CSplineInterpolator1D_H
#define CSplineInterpolator1D_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace math
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CSplineInterpolator1D, mrpt::utils::CSerializable )

		/** A (persistent) sequence of (x,y) coordinates, allowing queries of intermediate points through spline interpolation, where possible.
		  *  This class internally relies on mrpt::math::spline. Optionally the y coordinate can be set as wrapped in ]-pi,pi].
		  *  For querying interpolated points, see
		  * \ sa mrpt::math::spline, mrpt::poses::CPose3DInterpolator
		 * \ingroup interpolation_grp
		 */
		class BASE_IMPEXP CSplineInterpolator1D : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CSplineInterpolator1D )

		private:
			/** The placeholders for the data */
			std::map<double,double>	m_x2y;

			bool	m_wrap2pi;		//!< Whether to wrap "y"

		public:
			/** Constructor with optional initial values. */
			template <class VECTOR>
			inline CSplineInterpolator1D(
				const VECTOR &initial_x,
				const VECTOR &initial_y,
				bool  wrap2pi = false ) : m_wrap2pi(wrap2pi)
			{
				setXY(initial_x, initial_y);
			}

			/** Constructor */
			CSplineInterpolator1D( bool  wrap2pi = false );

			/** If set to true, the interpolated data will be wrapped to ]-pi,pi] */
			void setWrap2pi(bool wrap) { m_wrap2pi=wrap; }

			/** Return the wrap property */
			bool getWrap2pi() { return m_wrap2pi; }

			/** Set all the data at once .
			  *  The vectors must have the same length.
			 */
			template <class VECTOR>
			void setXY( const VECTOR &x, const VECTOR &y, bool clearPreviousContent = true )
			{
				MRPT_START
				if (clearPreviousContent) m_x2y.clear();
				ASSERT_EQUAL_(x.size(),y.size())
				const size_t n = size_t(x.size());
				for (size_t i=0;i<n;i++)
					m_x2y[ x[i] ] = y[i];
				MRPT_END
			}

			/** Append a new point: */
			void appendXY( double x, double y );

			/** Clears all stored points */
			void clear() { m_x2y.clear(); }

			/** Query an interpolation of the curve at some "x".
			  *   The result is stored in "y". If the "x" point is out of range, "valid_out" is set to false.
			  *  \return A reference to "y"
			  * \sa queryVector
			  */
			double &query( double x, double &y, bool &out_valid ) const;

			/** As query, but for a whole vector at once.
			  *  \return false if there is at least one value that couldn't be interpolated (in this case the output is indeterminate).
			  * \sa query
			  */
			template <class VECTOR1,class VECTOR2>
			bool queryVector( const VECTOR1 &x, VECTOR2 &out_y ) const
			{
				const size_t n = size_t(x.size());
				out_y.resize(n);
				bool valid, anyValid=false;
				for (size_t i =0;i<n;i++)
				{
					query( x[i], out_y[i], valid );
					if (valid) anyValid=true;
				}
				return anyValid;
			}

		};

	} // End of namespace
} // End of namespace
#endif
