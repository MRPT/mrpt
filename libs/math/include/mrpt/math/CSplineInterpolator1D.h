/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <map>

namespace mrpt::math
{
/** A (persistent) sequence of (x,y) coordinates, allowing queries of
 * intermediate points through spline interpolation, where possible.
 *  This class internally relies on mrpt::math::spline. Optionally the y
 * coordinate can be set as wrapped in ]-pi,pi].
 *  For querying interpolated points, see
 * \ sa mrpt::math::spline, mrpt::poses::CPose3DInterpolator
 * \ingroup interpolation_grp
 */
class CSplineInterpolator1D : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CSplineInterpolator1D)

   private:
	/** The placeholders for the data */
	std::map<double, double> m_x2y;

	/** Whether to wrap "y" */
	bool m_wrap2pi;

   public:
	/** Constructor with optional initial values. */
	template <class VECTOR>
	inline CSplineInterpolator1D(
		const VECTOR& initial_x, const VECTOR& initial_y, bool wrap2pi = false)
		: m_wrap2pi(wrap2pi)
	{
		setXY(initial_x, initial_y);
	}

	/** Constructor */
	CSplineInterpolator1D(bool wrap2pi = false);

	/** If set to true, the interpolated data will be wrapped to ]-pi,pi] */
	void setWrap2pi(bool wrap) { m_wrap2pi = wrap; }
	/** Return the wrap property */
	bool getWrap2pi() { return m_wrap2pi; }
	/** Set all the data at once .
	 *  The vectors must have the same length.
	 */
	template <class VECTOR>
	void setXY(
		const VECTOR& x, const VECTOR& y, bool clearPreviousContent = true)
	{
		MRPT_START
		if (clearPreviousContent) m_x2y.clear();
		ASSERT_EQUAL_(x.size(), y.size());
		const size_t n = size_t(x.size());
		for (size_t i = 0; i < n; i++) m_x2y[x[i]] = y[i];
		MRPT_END
	}

	/** Append a new point: */
	void appendXY(double x, double y);

	/** Clears all stored points */
	void clear() { m_x2y.clear(); }
	/** Query an interpolation of the curve at some "x".
	 *   The result is stored in "y". If the "x" point is out of range,
	 * "valid_out" is set to false.
	 *  \return A reference to "y"
	 * \sa queryVector
	 */
	double& query(double x, double& y, bool& out_valid) const;

	/** As query, but for a whole vector at once.
	 *  \return false if there is at least one value that couldn't be
	 * interpolated (in this case the output is indeterminate).
	 * \sa query
	 */
	template <class VECTOR1, class VECTOR2>
	bool queryVector(const VECTOR1& x, VECTOR2& out_y) const
	{
		const size_t n = size_t(x.size());
		out_y.resize(n);
		bool valid, anyValid = false;
		for (size_t i = 0; i < n; i++)
		{
			query(x[i], out_y[i], valid);
			if (valid) anyValid = true;
		}
		return anyValid;
	}
};

}  // namespace mrpt::math
