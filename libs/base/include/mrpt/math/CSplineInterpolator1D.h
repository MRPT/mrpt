/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
